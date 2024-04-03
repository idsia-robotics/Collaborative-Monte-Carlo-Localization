/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: OprtitrackPublisherNode.cpp                                             #
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


class OprtitrackPublisherNode : public rclcpp::Node
{
  public:
    OprtitrackPublisherNode()
    : Node("OprtitrackPublisherNode")
    {
      std::string robotName;
     

     
      this->declare_parameter("robotName" ,"");
      this->get_parameter("robotName", robotName);
      RCLCPP_INFO(this->get_logger(), "robotName %s", robotName.c_str());
      rclcpp::QoS qos(10);
      qos.best_effort();
      o_gtPub = this->create_publisher<geometry_msgs::msg::PoseStamped>(robotName + "/GT", 10);
      o_optitrackSub = create_subscription<geometry_msgs::msg::PoseStamped>("/optitrack" + robotName, qos, std::bind(&OprtitrackPublisherNode::callback, this, std::placeholders::_1));

      
     RCLCPP_INFO(this->get_logger(), "OprtitrackPublisherNode running!");
    }

    void callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& optitrack_msg)
    {
    	geometry_msgs::msg::PoseStamped gt_msg;

    	gt_msg.pose = optitrack_msg->pose;
    	gt_msg.header = optitrack_msg->header;
    	gt_msg.header.frame_id = "map";

    	o_gtPub->publish(gt_msg);
    }
private:

	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr o_optitrackSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr o_gtPub;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OprtitrackPublisherNode>());
  rclcpp::shutdown();
  return 0;
}