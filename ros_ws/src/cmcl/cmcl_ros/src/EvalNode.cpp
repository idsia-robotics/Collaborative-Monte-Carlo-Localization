/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: EvalNode.cpp.cpp                                                        #
# ##############################################################################
**/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "RosUtils.h"
#include "Utils.h"
#include "cmcl_msgs/msg/prediction.hpp"  


#include <iostream>
#include <fstream>


//typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseWithCovarianceStamped, geometry_msgs::msg::PoseStamped> PoseSyncPolicy;

typedef message_filters::sync_policies::ApproximateTime<cmcl_msgs::msg::Prediction, geometry_msgs::msg::PoseStamped> PoseSyncPolicy;


class EvalNode : public rclcpp::Node
{
  public:
    EvalNode()
    : Node("EvalNode")
    {
      std::string gtTopic;
      std::string poseTopic;
      std::string csvPath;
      std::string predTopic;


      this->declare_parameter("gtTopic", "");
      this->declare_parameter("poseTopic", "");
      this->declare_parameter("predTopic", "");
      this->declare_parameter("csvPath", "");
    
      this->get_parameter("gtTopic", gtTopic);
      RCLCPP_INFO(this->get_logger(), "gtTopic %s", gtTopic.c_str());
      this->get_parameter("poseTopic", poseTopic);
      RCLCPP_INFO(this->get_logger(), "poseTopic %s", poseTopic.c_str());
      this->get_parameter("predTopic", predTopic);
      RCLCPP_INFO(this->get_logger(), "predTopic %s", predTopic.c_str());
      this->get_parameter("csvPath", csvPath);
      RCLCPP_INFO(this->get_logger(), "csvPath %s", csvPath.c_str());


      o_csv.open(csvPath);

      //o_csv << "t,gt_x,gt_y,gt_yaw,pose_x,pose_y,pose_yaw" << std::endl;
      o_csv << "t,gt_x,gt_y,gt_yaw,pose_x,pose_y,pose_yaw,source" << std::endl;
      
      
      gtSub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, gtTopic);
      //poseSub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>>(this, poseTopic);
      predSub = std::make_shared<message_filters::Subscriber<cmcl_msgs::msg::Prediction>>(this, predTopic);

      o_poseSync = std::make_shared<message_filters::Synchronizer<PoseSyncPolicy>>(PoseSyncPolicy(10), *predSub, *gtSub);
      o_poseSync->registerCallback(std::bind(&EvalNode::callback, this, std::placeholders::_1, std::placeholders::_2)); 


       RCLCPP_INFO(this->get_logger(), "Ready!");
      
      
    }

   /* void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& poseMsg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& gtMsg)
    {

      float yaw = Wrap2Pi(2 * atan2(poseMsg->pose.pose.orientation.z, poseMsg->pose.pose.orientation.w));
      Eigen::Vector3f pose = Eigen::Vector3f(poseMsg->pose.pose.position.x, poseMsg->pose.pose.position.y, yaw);

      yaw = Wrap2Pi(2 * atan2(gtMsg->pose.orientation.z, gtMsg->pose.orientation.w));
      Eigen::Vector3f gt = Eigen::Vector3f(gtMsg->pose.position.x, gtMsg->pose.position.y, yaw);

      //double stamp = double(poseMsg->header.stamp.sec) + double(poseMsg->header.stamp.nanosec) * 1e-9;

      o_csv << std::to_string(long(poseMsg->header.stamp.sec)) <<"." << std::to_string(long(poseMsg->header.stamp.nanosec)) << "," << gt(0) << "," 
      << gt(1) << "," << gt(2) << "," << pose(0) << "," << pose(1) << "," << pose(2) << std::endl;

      //RCLCPP_INFO(this->get_logger(), "Logging!");

    }*/

    void callback(const cmcl_msgs::msg::Prediction::ConstSharedPtr& predMsg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& gtMsg)
    {
      geometry_msgs::msg::PoseWithCovarianceStamped poseMsg = predMsg->pose;
      float yaw = Wrap2Pi(2 * atan2(poseMsg.pose.pose.orientation.z, poseMsg.pose.pose.orientation.w));
      Eigen::Vector3f pose = Eigen::Vector3f(poseMsg.pose.pose.position.x, poseMsg.pose.pose.position.y, yaw);

      yaw = Wrap2Pi(2 * atan2(gtMsg->pose.orientation.z, gtMsg->pose.orientation.w));
      Eigen::Vector3f gt = Eigen::Vector3f(gtMsg->pose.position.x, gtMsg->pose.position.y, yaw);

      //double stamp = double(poseMsg->header.stamp.sec) + double(poseMsg->header.stamp.nanosec) * 1e-9;

      o_csv << std::to_string(long(poseMsg.header.stamp.sec)) <<"." << std::to_string(long(poseMsg.header.stamp.nanosec)) << "," << gt(0) << "," 
      << gt(1) << "," << gt(2) << "," << pose(0) << "," << pose(1) << "," << pose(2) << "," << std::to_string(predMsg->type) << std::endl;

      //RCLCPP_INFO(this->get_logger(), "Logging %d", predMsg->type);

    }

  private:
   
   std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> gtSub;
    //std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped>> poseSub;
    std::shared_ptr<message_filters::Subscriber<cmcl_msgs::msg::Prediction>> predSub;

    std::shared_ptr<message_filters::Synchronizer<PoseSyncPolicy>> o_poseSync;
    std::ofstream o_csv;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvalNode>());
  rclcpp::shutdown();
  return 0;
}