/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: LidarScan2PointCloudNode.cpp                                          #
# ##############################################################################
**/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"


#include "Utils.h"
#include "RosUtils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>



class LidarScan2PointCloudNode : public rclcpp::Node
{
  public:
    LidarScan2PointCloudNode()
    : Node("LidarScan2PointCloudNode")
    {
      std::string lidarScanTopic;
      std::string lidarTF;

      this->declare_parameter("lidarScanTopic" ,"");
      this->declare_parameter("baseLinkTF", "");
      this->declare_parameter("pointCloudTopic", "");
      this->declare_parameter("lidarTF", "");

      this->get_parameter("lidarScanTopic", lidarScanTopic);
      RCLCPP_INFO(this->get_logger(), "lidarScanTopic %s", lidarScanTopic.c_str());
      this->get_parameter("pointCloudTopic", o_pointCloudTopic);
      RCLCPP_INFO(this->get_logger(), "pointCloudTopic %s", o_pointCloudTopic.c_str());
      this->get_parameter("baseLinkTF", o_baseLinkTF);
      RCLCPP_INFO(this->get_logger(), "baseLinkTF %s", o_baseLinkTF.c_str());
      this->get_parameter("lidarTF", lidarTF);
      RCLCPP_INFO(this->get_logger(), "lidarTF %s", lidarTF.c_str());
    
      

      o_pointCloudPub =  this->create_publisher<sensor_msgs::msg::PointCloud2>(o_pointCloudTopic, 1);
      tf_scan = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      geometry_msgs::msg::TransformStamped t;

      bool getTF = true;
      Eigen::Vector3f origin;
      while(getTF)
      {
        try 
        {
          //t = tf_buffer->lookupTransform(lidarTF.c_str(), o_baseLinkTF.c_str(), tf2::TimePointZero, tf2::Duration(1000000000));
          t = tf_buffer->lookupTransform(o_baseLinkTF.c_str(), lidarTF.c_str(), tf2::TimePointZero, tf2::Duration(1000000000));
          RCLCPP_INFO(
          this->get_logger(), "Transform %s to %s: %f %f %f", o_baseLinkTF.c_str(), lidarTF.c_str(), 
            t.transform.translation.x, t.transform.translation.y, GetYaw(t.transform.rotation.z, t.transform.rotation.w));

          origin = Eigen::Vector3f(t.transform.translation.x, t.transform.translation.y, GetYaw(t.transform.rotation.z, t.transform.rotation.w));
          getTF = false;
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
              o_baseLinkTF.c_str(), lidarTF.c_str(), ex.what());
        }
      }

      //origin = Eigen::Vector3f(0, 0,0);

      o_trans = Vec2Trans(origin);
      //o_trans = o_trans.inverse();


      rclcpp::QoS qos(10);
      qos.best_effort(); 
      //qos.durability_volatile();

      o_lidrScanSub = create_subscription<sensor_msgs::msg::LaserScan>(lidarScanTopic, qos, std::bind(&LidarScan2PointCloudNode::callback, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "LidarScan2PointCloudNode running!");
      
      
    }

    void callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& laserMsg)
    {
      std::vector<float> ranges = laserMsg->ranges;
      float angleMin = laserMsg->angle_min;
      float angleMax = laserMsg->angle_max;
      float angIncr = laserMsg->angle_increment;

      int numBeams = fabs(angleMax - angleMin) / angIncr;
      std::vector<float> heading(numBeams);

      for(int i = 0; i < numBeams; i++)
      {
        heading[i] = angleMin + i * angIncr;
      }

      // here add masking code
      std::vector<Eigen::Vector3f> points_3d = Ranges2Points(ranges, heading);

      int n = points_3d.size(); 

      for(int i = 0; i < n; i++)
      {
        Eigen::Vector3f p = points_3d[i];
        Eigen::Vector3f p_trans = o_trans * p;
        points_3d[i] = p_trans;
      }


      //std::vector<Eigen::Vector3f> points_3d = l2d->Center(ranges);


      // geometry_msgs::msg::TransformStamped t;
      // t.header.stamp = rclcpp::Time();  
      // t.header.frame_id =  o_baseLinkTF.c_str();
      // t.child_frame_id = "scan_merged"; 
      // t.transform.translation.x = 0.0;
      // t.transform.translation.y = 0.0;
      // t.transform.translation.z = 0.0;
      // tf2::Quaternion q;
      // q.setRPY(0, 0, 0);
      // t.transform.rotation.x = q.x();
      // t.transform.rotation.y = q.y();
      // t.transform.rotation.z = q.z();
      // t.transform.rotation.w = q.w();
      // tf_scan->sendTransform(t); 

      pcl::PointCloud<pcl::PointXYZ> pcl = Vec2PointCloud(points_3d);
      sensor_msgs::msg::PointCloud2 pcl_msg;
      pcl::toROSMsg(pcl, pcl_msg);
      pcl_msg.header.stamp = laserMsg->header.stamp;
      pcl_msg.header.frame_id = o_pointCloudTopic;
      o_pointCloudPub->publish(pcl_msg);


      tf2::Transform tf_orig;
      tf_orig.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      tf_orig.setRotation(q);
      tf2::Transform tf_inv = tf_orig.inverse();
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = laserMsg->header.stamp;  
      t.header.frame_id = o_baseLinkTF.c_str();
      t.child_frame_id = o_pointCloudTopic; 
      t.transform.translation.x = tf_inv.getOrigin().x();
      t.transform.translation.y = tf_inv.getOrigin().y();
      t.transform.translation.z = tf_inv.getOrigin().z();
      t.transform.rotation.x = tf_inv.getRotation().x();
      t.transform.rotation.y = tf_inv.getRotation().y();
      t.transform.rotation.z = tf_inv.getRotation().z();
      t.transform.rotation.w = tf_inv.getRotation().w();
      tf_scan->sendTransform(t);

    }

  private:

    std::string o_baseLinkTF;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr o_pointCloudPub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr o_lidrScanSub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_scan;
    Eigen::Matrix3f o_trans;
    std::string o_pointCloudTopic;



   

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScan2PointCloudNode>());
  rclcpp::shutdown();
  return 0;
}