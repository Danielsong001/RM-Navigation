/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "modules/perception/detection/armor_detection/armor_detection_node.h"
#include "modules/driver/serial/proto/serial_com_config.pb.h"
#include <math.h>


namespace rrts{
namespace perception {
namespace detection {

ArmorDetectionNode::ArmorDetectionNode(std::string name):
    node_state_(rrts::common::IDLE),
    demensions_(3),
    initialized_(false),
    rrts::common::RRTS::RRTS(name),
    as_(nh_, name+"_action", boost::bind(&ArmorDetectionNode::ActionCB, this, _1), false) {
  initialized_ = false;
  if (Init().IsOK()) {
    initialized_ = true;
    node_state_ = rrts::common::IDLE;
  } else {
    LOG_ERROR << "armor_detection_node initalized failed!";
    node_state_ = rrts::common::FAILURE;
  }
  NOTICE("Waiting for input command in armor_detection_client...")
  as_.start();
}

ErrorInfo ArmorDetectionNode::Init() {
  enemy_info_pub_ = nh_.advertise<messages::EnemyPos>("enemy_pos", 100);

  ArmorDetectionAlgorithms armor_detection_algorithms;
  std::string file_name = "modules/perception/detection/armor_detection/config/armor_detection.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &armor_detection_algorithms);
  if (!read_state) {
    LOG_ERROR << "Cannot open " << file_name;
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  }

  //create the selected algorithms
  std::string selected_algorithm = armor_detection_algorithms.selected_algorithm();
  armor_detector_ = rrts::common::AlgorithmFactory<ArmorDetectionBase>::CreateAlgorithm(selected_algorithm);

  //KalmanFilter Initialization
  KalmanReset();

  if (armor_detector_ == nullptr) {
    LOG_ERROR << "Create armor_detector_ pointer failed!";
    return ErrorInfo(ErrorCode::DETECTION_INIT_ERROR);
  } else
    return ErrorInfo(ErrorCode::OK);
}
void ArmorDetectionNode::KalmanReset(){
    KF.init(4,2,0);
    KF.transitionMatrix = (cv::Mat_<float>(4,4) << 0,0,1,0,0,0,0,1,0,0,1,0,0,0,0,1);
    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov,cv::Scalar::all(1e-5));
    cv::setIdentity(KF.measurementNoiseCov,cv::Scalar::all(1e-5));
    cv::setIdentity(KF.errorCovPost,cv::Scalar::all(1));
    cv::RNG r;
//    KF.statePost = cv::Mat::zeros(cv::Size(4,1),CV_32F);
    r.fill(KF.statePost,cv::RNG::UNIFORM,0,200);
    meassurement = cv::Mat::zeros(2,1,CV_32F);
//    std::cout<<KF.transitionMatrix<<std::endl;
//    std::cout<<meassurement<<std::endl;

}
void ArmorDetectionNode::GimbleRealInfo(const messages::EnemyPosConstPtr &msg){
    gimbal_info.enemy_pitch = -msg->enemy_pitch;
    gimbal_info.enemy_yaw = -msg->enemy_yaw;
//    std::cout<<"*********'''''''''''''''Gimbal_Info''''''''''''*******"<<std::endl;
//    std::cout<<"gimbal_info.pitch:"<<gimbal_info.pitch<<std::endl;
//    std::cout<<"gimbal_info.yaw:"<<gimbal_info.yaw<<std::endl;
    received_gimbal_info_state = true;
}
void ArmorDetectionNode::ActionCB(const messages::ArmorDetectionGoal::ConstPtr &data) {
  messages::ArmorDetectionFeedback feedback;
  messages::ArmorDetectionResult result;

  if(!initialized_){
    feedback.error_code = error_info_.error_code();
    feedback.error_msg  = error_info_.error_msg();
    as_.publishFeedback(feedback);
    as_.setAborted(result, feedback.error_msg);
    LOG_INFO<<"Initialization Failed, Failed to execute action!";
    return;
  }

  switch (data->command) {
    case 1:
      StartThread();
      break;
    case 2:
      PauseThread();
      break;
    case 3:
      StopThread();
      break;
    default:
      break;
  }

  while(ros::ok()) {
    if(as_.isPreemptRequested()) {
      as_.setPreempted();
      return;
    }

    if (distance_ != -1.0) {
      {
        std::lock_guard<std::mutex> guard(mutex_);
        feedback.error_code = error_info_.error_code();
        feedback.error_msg = error_info_.error_msg();

        feedback.enemy_d = distance_;

        feedback.enemy_pitch = pitch_;
        feedback.enemy_yaw   = yaw_;
        as_.publishFeedback(feedback);
        distance_ = -1.0;
      }
    }
  }
}

void ArmorDetectionNode::ExecuteLoop() {
  double distance;
  double pitch = 0;
  double yaw = 0;

//  unsigned int count = 0;
  //lzh
  bool gimbal_restart_state = false;

  while(running_) {

        if (node_state_ == NodeState::RUNNING) {

          distance = -1.0;
          ErrorInfo error_info = armor_detector_->DetectArmor(distance, pitch, yaw);
          {
            std::lock_guard<std::mutex> guard(mutex_);
            distance_ = distance;
            pitch_    = pitch;
            yaw_      = yaw;
            error_info_ = error_info;
          }
             // std::cout<<"distance:"<<distance<<std::endl;
             //  std::cout<<"pitch+gimbal_info.enemy_pitch:"<<pitch+gimbal_info.enemy_pitch<<std::endl;
          //The Kalman Filter
          if(distance == -1 || pitch+gimbal_info.enemy_pitch > -3 || distance > 3000 || distance < 300){
              detected_enemy_ = false;
              if(gimbal_restart >= 50){
                  gimbal_restart_state = true;
                  gimbal_restart = 0;
              }
              gimbal_restart ++;
          }else{
		if((distance < 1500 && pitch+gimbal_info.enemy_pitch < -10)||
		(distance > 1500 && pitch+gimbal_info.enemy_pitch < -3))
		{
		      detected_enemy_ = true;
		      gimbal_restart_state = false;
		      gimbal_restart = 0;
		}
          }


//	 if(detected_enemy_){
	              
//	  enemy_pos.enemy_dist  = distance;
//          enemy_pos.enemy_pitch =  pitch+gimbal_info.enemy_pitch;
//          enemy_pos.enemy_yaw =  yaw+gimbal_info.enemy_yaw;
//	PublishMsgs();
//	}



          if(detected_enemy_ && received_gimbal_info_state)
          {
              received_gimbal_info_state=false;
              detected_enemy_ = false;
              cv::Mat prediction = KF.predict();

              meassurement.at<float>(0) =  pitch+gimbal_info.enemy_pitch ;
              meassurement.at<float>(1) =  yaw+gimbal_info.enemy_yaw;

              cv::Mat corection = KF.correct(meassurement);
//              std::cout<<"$$$$$$$$$$$$$$$$$$$$$$$Detected Angle$$$$$$$$$$$$$$$$$$$$$"<<std::endl;
//              std::cout<<"pitch:"<<pitch<<std::endl;
//              std::cout<<"yaw:"<<yaw<<std::endl;
//              std::cout<<"*********'''''''''''''''Gimbal_Info''''''''''''*******"<<std::endl;
//              std::cout<<"gimbal_info.pitch:"<<gimbal_info.pitch<<std::endl;
//              std::cout<<"gimbal_info.yaw:"<<gimbal_info.enemy_yaw<<std::endl;
//              std::cout<<"===============================KF point=========================================="<<std::endl;
//              std::cout<<"KF.statePost.at<float>(0):"<<KF.statePost.at<float>(0)<<std::endl;
//              std::cout<<"KF.statePost.at<float>(1):"<<KF.statePost.at<float>(1)<<std::endl;
              enemy_pos.enemy_dist  = distance;

              enemy_pos.enemy_pitch =  KF.statePost.at<float>(0);
              enemy_pos.enemy_yaw =  KF.statePost.at<float>(1);
              if(std::abs(yaw) < 1 && std::abs(pitch) < 0.5 )
                  KalmanReset();
            }
          else if(gimbal_restart_state) {
              gimbal_restart_state = false;
              KalmanReset();
              enemy_pos.enemy_dist = 0;
              enemy_pos.enemy_pitch = 0;
              enemy_pos.enemy_yaw = 0;

            }
          else if (gimbal_restart > 10)
          {
              enemy_pos.enemy_dist = dis_record;
              enemy_pos.enemy_pitch = pi_record ;
              enemy_pos.enemy_yaw = ya_record;
          }

          PublishMsgs();

          //TODO(noah.guo): coordinat transformation, data fitting.
        }else if(node_state_ == NodeState::PAUSE) {
          std::unique_lock<std::mutex> lock(mutex_);
          condition_var_.wait(lock);
        }
  }
}



void ArmorDetectionNode::PublishMsgs() {
  std::cout<<"enemy_pos.enemy_dist:"<<enemy_pos.enemy_dist<<std::endl;
  std::cout<<"enemy_pos.enemy_pitch:"<<enemy_pos.enemy_pitch<<std::endl;
  std::cout<<"enemy_pos.enemy_yaw:"<<enemy_pos.enemy_yaw<<std::endl;
  dis_record = enemy_pos.enemy_dist;
  pi_record = enemy_pos.enemy_pitch;
  ya_record = enemy_pos.enemy_yaw;
  enemy_info_pub_.publish(enemy_pos);
}

void ArmorDetectionNode::StartThread() {
  LOG_INFO << "Armor detection node started!";
  running_ = true;
  if(node_state_ == NodeState::IDLE) {
    armor_detection_thread_ = std::thread(&ArmorDetectionNode::ExecuteLoop, this);
  }

  //subscribe the gimbal_info ____lzh
  gimbal_info_pub_ = nh_.subscribe("gimbalpos",1,&ArmorDetectionNode::GimbleRealInfo,this);


  node_state_ = NodeState::RUNNING;
  condition_var_.notify_one();
}

void ArmorDetectionNode::PauseThread() {
  LOG_INFO << "Armor detection thread paused!";
  node_state_ = NodeState::PAUSE;
}

void ArmorDetectionNode::StopThread() {
  node_state_ = NodeState::IDLE;
  running_ = false;
  if (armor_detection_thread_.joinable()) {
    armor_detection_thread_.join();
  }
}

ArmorDetectionNode::~ArmorDetectionNode() {
  StopThread();
}



} //namespace detection
} //namespace perception
} //namespace rrts



MAIN(rrts::perception::detection::ArmorDetectionNode, "armor_detection_node")
//int main(int argc, char **argv){
//  rrts::common::GLogWrapper glog_wrapper(argv[0]);
//  signal(SIGINT, SignalHandler);
//  signal(SIGTERM,SignalHandler);
//  ros::init(argc, argv, "armor_detection_node", ros::init_options::NoSigintHandler);
//  rrts::perception::detection::ArmorDetectionNode rrts("armor_detection_node");
//  rrts.Run();
//  return 0;
//}
