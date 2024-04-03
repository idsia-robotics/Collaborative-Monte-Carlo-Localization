/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: RelativePosedNode.cpp                                             #
# ##############################################################################
**/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/qos.hpp>
#include "rclcpp/wait_for_message.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "Utils.h"
#include "RosUtils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <robomaster_msgs/msg/detection.hpp>
#include <robomaster_msgs/msg/detected_robot.hpp>
#include "cmcl_msgs/msg/robot_detection.hpp"   


typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped, robomaster_msgs::msg::Detection> PoseSyncPolicy;


class RelativePosedNode : public rclcpp::Node
{
  public:
    RelativePosedNode()
    : Node("RelativePosedNode")
    {
      std::string visionTopic;
      std::string robotName1;
      std::string robotName2;
      std::string detectedRobot;

      this->declare_parameter("visionTopic" ,"");
      this->declare_parameter("robotName1" ,"");
      this->declare_parameter("robotName2", "");
      this->declare_parameter("detectedRobot", "");
      this->declare_parameter("robotID", 0);

    
      this->get_parameter("visionTopic", visionTopic);
      RCLCPP_INFO(this->get_logger(), "visionTopic %s", visionTopic.c_str());
      this->get_parameter("robotName1", robotName1);
      RCLCPP_INFO(this->get_logger(), "robotName1 %s", robotName1.c_str());
      this->get_parameter("robotName2", robotName2);
      RCLCPP_INFO(this->get_logger(), "robotName2 %s", robotName2.c_str());
      this->get_parameter("detectedRobot", detectedRobot);
      RCLCPP_INFO(this->get_logger(), "detectedRobot %s", detectedRobot.c_str());
      this->get_parameter("robotID", o_robotID);
      RCLCPP_INFO(this->get_logger(), "robotID %d", o_robotID);

      o_detectionPub = this->create_publisher<cmcl_msgs::msg::RobotDetection>(detectedRobot, 10);
      r1Sub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, "/optitrack" + robotName1);
      r2Sub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(this, "/optitrack" + robotName2);
      detectionSub = std::make_shared<message_filters::Subscriber<robomaster_msgs::msg::Detection>>(this, visionTopic);
      o_depthSync = std::make_shared<message_filters::Synchronizer<PoseSyncPolicy>>(PoseSyncPolicy(1), *r1Sub, *r2Sub, *detectionSub);
      o_depthSync->registerCallback(std::bind(&RelativePosedNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)); 

     RCLCPP_INFO(this->get_logger(), "RelativePosedNode running!");
    }

    void callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& r1_msg, 
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr& r2_msg,
      const robomaster_msgs::msg::Detection::ConstSharedPtr& detection_msg)
    {

      if (detection_msg->robots.size())
      {
        Eigen::Vector3f pose1 = Eigen::Vector3f(r1_msg->pose.position.x, r1_msg->pose.position.y, GetYaw(r1_msg->pose.orientation.z, r1_msg->pose.orientation.w));
        Eigen::Vector3f pose2 = Eigen::Vector3f(r2_msg->pose.position.x, r2_msg->pose.position.y, GetYaw(r2_msg->pose.orientation.z, r2_msg->pose.orientation.w));

        Eigen::Matrix3f trans = Vec2Trans(pose1);
        Eigen::Matrix3f invTrans = trans.inverse();

        Eigen::Vector3f relPose = invTrans * Eigen::Vector3f(pose2(0), pose2(1), 1.0);
        relPose(2) = Wrap2Pi(pose2(2) - pose1(2));

        if(abs(atan2(relPose(1), relPose(0))) > 0.9) return;

        cmcl_msgs::msg::RobotDetection msg;
        msg.header.stamp = detection_msg->header.stamp;
        msg.observer_id = o_robotID;
        msg.detected_id = 1 - o_robotID;
        msg.type = 1;
        msg.pose.position.x = relPose(0);
        msg.pose.position.y = relPose(1);
        msg.pose.position.z = 0.0;
        o_detectionPub->publish(msg);  

        //RCLCPP_INFO(this->get_logger(), "Detection!");
      }
    }


   private:

    int o_robotID = 0;
    rclcpp::Publisher<cmcl_msgs::msg::RobotDetection>::SharedPtr o_detectionPub;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> r1Sub;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> r2Sub;
    std::shared_ptr<message_filters::Subscriber<robomaster_msgs::msg::Detection>> detectionSub;
    std::shared_ptr<message_filters::Synchronizer<PoseSyncPolicy>> o_depthSync;


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelativePosedNode>());
  rclcpp::shutdown();
  return 0;
}