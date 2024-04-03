/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: DetectionNoiseNode.cpp                                                #
# ##############################################################################
**/  

#include <chrono>   
#include <functional> 
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "cmcl_msgs/msg/robot_detection.hpp"  
#include <math.h>
#include "Utils.h"


class DetectionNoiseNode : public rclcpp::Node
{      
  public:
    DetectionNoiseNode() : Node("DetectionNoiseNode")
    {
    	std::string detectionTopic;
        this->declare_parameter("detectionTopic", "");
        this->get_parameter("detectionTopic", detectionTopic);
        RCLCPP_INFO(this->get_logger(), "detectionTopic %s", detectionTopic.c_str());

        std::string noisyDetectionTopic;
        this->declare_parameter("noisyDetectionTopic", "");
        this->get_parameter("noisyDetectionTopic", noisyDetectionTopic);
        RCLCPP_INFO(this->get_logger(), "noisyDetectionTopic %s", noisyDetectionTopic.c_str());

        this->declare_parameter("detectionNoise", std::vector<double>(2));
        rclcpp::Parameter dblArrParam =this->get_parameter("detectionNoise");
      	std::vector<double> detectionNoise = dblArrParam.as_double_array();
      	RCLCPP_INFO(this->get_logger(), "detectionNoise %f %f", detectionNoise[0], detectionNoise[1]);
      	o_detectionNoise = Eigen::Vector2f(detectionNoise[0], detectionNoise[1]);

      	rclcpp::QoS qos(10);
        qos.best_effort(); 
  

        o_detectionSub = create_subscription<cmcl_msgs::msg::RobotDetection>(detectionTopic, qos, std::bind(&DetectionNoiseNode::detectionCallback, this, std::placeholders::_1));
        o_detectPub = this->create_publisher<cmcl_msgs::msg::RobotDetection>(noisyDetectionTopic, 10); 

    }

    void detectionCallback(const cmcl_msgs::msg::RobotDetection::SharedPtr det_msg) 
    {
    	cmcl_msgs::msg::RobotDetection noisy_msg;
    	noisy_msg.header = det_msg->header;
    	noisy_msg.detected_id = det_msg->detected_id;
        noisy_msg.observer_id = det_msg->observer_id;
        noisy_msg.pose = det_msg->pose;
        noisy_msg.type = det_msg->type;

    	float x = det_msg->pose.position.x;
    	float y = det_msg->pose.position.y;

    	float r = sqrt(x * x + y * y);
    	float theta = atan2(y, x);

    	float dr = abs(r * SampleGuassian(o_detectionNoise(0)));
    	float dt = Wrap2Pi(SampleGuassian(o_detectionNoise(1)));

    	float x_n = (r + dr) * cos(theta + dt);
    	float y_n = (r + dr) * sin(theta + dt);

    	noisy_msg.pose.position.x = x_n;
    	noisy_msg.pose.position.y = y_n;

    	//RCLCPP_INFO(this->get_logger(), "orig %f %f, noisy %f %f", x, y, x_n, y_n);

    	o_detectPub->publish(noisy_msg);
    }


   private:

   	    rclcpp::Subscription<cmcl_msgs::msg::RobotDetection>::SharedPtr o_detectionSub;
   	    rclcpp::Publisher<cmcl_msgs::msg::RobotDetection>::SharedPtr o_detectPub;
   	    Eigen::Vector2f o_detectionNoise = Eigen::Vector2f(0.15, 0.15);




};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionNoiseNode>());
  rclcpp::shutdown();
  return 0;
}

