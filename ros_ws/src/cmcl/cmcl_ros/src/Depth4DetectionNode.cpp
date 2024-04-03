/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Depth4DetectionNode.cpp                                               #
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


#include "Utils.h"
#include "RosUtils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <robomaster_msgs/msg/detection.hpp>
#include <robomaster_msgs/msg/detected_robot.hpp>
#include "cmcl_msgs/msg/robot_detection.hpp"   



typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, robomaster_msgs::msg::Detection> DepthSyncPolicy;



class Depth4DetectionNode : public rclcpp::Node
{
  public:
    Depth4DetectionNode()
    : Node("Depth4DetectionNode")
    {
      std::string detectionCameraTopic;
      std::string detectionInfoTopic;
      std::string detectionPoseTopic;
      std::string pointCloudTopic;

      this->declare_parameter("detectionCameraTopic" ,"");
      this->declare_parameter("detectionInfoTopic" ,"");
      this->declare_parameter("detectionPoseTopic" ,"");
      this->declare_parameter("baseLinkTF", "");
      this->declare_parameter("pointCloudTopic", "");
      this->declare_parameter("cameraTF", "");
      this->declare_parameter("robotID", 0);


      this->get_parameter("detectionCameraTopic", detectionCameraTopic);
      RCLCPP_INFO(this->get_logger(), "detectionCameraTopic %s", detectionCameraTopic.c_str());
      this->get_parameter("detectionInfoTopic", detectionInfoTopic);
      RCLCPP_INFO(this->get_logger(), "detectionInfoTopic %s", detectionInfoTopic.c_str());
      this->get_parameter("detectionPoseTopic", detectionPoseTopic);
      RCLCPP_INFO(this->get_logger(), "detectionPoseTopic %s", detectionPoseTopic.c_str());

      this->get_parameter("pointCloudTopic", pointCloudTopic);
      RCLCPP_INFO(this->get_logger(), "pointCloudTopic %s", pointCloudTopic.c_str());
      this->get_parameter("baseLinkTF", o_baseLinkTF);
      RCLCPP_INFO(this->get_logger(), "baseLinkTF %s", o_baseLinkTF.c_str());
      this->get_parameter("cameraTF", o_cameraTF);
      RCLCPP_INFO(this->get_logger(), "cameraTF %s", o_cameraTF.c_str());

      this->get_parameter("robotID", o_robotID);
      RCLCPP_INFO(this->get_logger(), "robotID %d", o_robotID);

      rclcpp::QoS qos(10);
      qos.best_effort();
      o_camInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(detectionInfoTopic, qos, std::bind(&Depth4DetectionNode::calibrationCallback, this, std::placeholders::_1));
      
      o_normTH = 0.99;
    
      // rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
      // custom_qos.depth=10;
      // custom_qos.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      // custom_qos.durability=RMW_QOS_POLICY_DURABILITY_VOLATILE;

      o_pointCloudPub =  this->create_publisher<sensor_msgs::msg::PointCloud2>("/RM0001/debug_cloud", 1);


      o_detectionPub = this->create_publisher<cmcl_msgs::msg::RobotDetection>(detectionPoseTopic, 10);

      scanSub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, pointCloudTopic);
      detectionSub = std::make_shared<message_filters::Subscriber<robomaster_msgs::msg::Detection>>(this, detectionCameraTopic);

      o_depthSync = std::make_shared<message_filters::Synchronizer<DepthSyncPolicy>>(DepthSyncPolicy(10), *scanSub, *detectionSub);
      o_depthSync->registerCallback(std::bind(&Depth4DetectionNode::callback, this, std::placeholders::_1, std::placeholders::_2)); 


      RCLCPP_INFO(this->get_logger(), "Depth4DetectionNode running!");
      
      
    }

    void calibrationCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg)
    {
      
      //RCLCPP_INFO(this->get_logger(), "camInf (w, h) = (%d %d)", cam_info_msg->width, cam_info_msg->height);

      o_height = cam_info_msg->height;
      o_width = cam_info_msg->width;
      auto k = cam_info_msg->k.data();
      Eigen::Matrix3d K = Eigen::Matrix<double,3,3,Eigen::RowMajor>(k);
      //RCLCPP_INFO(this->get_logger(), "k = %f", K(0,0));
      o_invK = K.inverse();

      //load T matrix 
      std::vector<double> t = {0, 0, 1, -1, 0, 0, 0, -1, 0};
      o_T = Eigen::Matrix<double,3,3,Eigen::RowMajor>(t.data());

      o_calib = true;
      o_camInfoSub.reset();
    }

    Eigen::Matrix3f getCameraTF()
    {
      std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      geometry_msgs::msg::TransformStamped t;

      bool getTF = true;
      Eigen::Vector3f camPose;
      while(getTF)
      {
        try 
        {
          t = tf_buffer->lookupTransform(o_baseLinkTF.c_str(), o_cameraTF.c_str(), tf2::TimePointZero, tf2::Duration(1000000000));
          RCLCPP_INFO(
          this->get_logger(), "Transform %s to %s: %f %f %f", o_baseLinkTF.c_str(), o_cameraTF.c_str(),
            t.transform.translation.x, t.transform.translation.y, GetYaw(t.transform.rotation.z, t.transform.rotation.w));

          camPose = Eigen::Vector3f(t.transform.translation.x, t.transform.translation.y, GetYaw(t.transform.rotation.z, t.transform.rotation.w));
          getTF = false;
        }
        catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
              o_baseLinkTF.c_str(), o_cameraTF.c_str(), ex.what());
        }
      }

      Eigen::Matrix3f trans = Vec2Trans(camPose);

      return trans;
    }

    Eigen::Vector3d  UV2CameraFrame(Eigen::Vector2f q1)
    {
      Eigen::Vector3d p1(q1(0), q1(1), 1);

      // multiply by inverse calibration matrix
      Eigen::Vector3d p1k_ = o_invK * p1;

      // divide by z component to homogenize it
      Eigen::Vector3d p1k = p1k_ / p1k_(2);

      // go from image frame to camera frame
      Eigen::Vector3d p1c = o_T * p1k;

      return p1c;
    }

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl_msg, const robomaster_msgs::msg::Detection::ConstSharedPtr& detection_msg)
    {

        if (!o_calib) return;

        Eigen::Matrix3f trans = getCameraTF();

        int scanSize = pcl_msg->width;
        std::vector<Eigen::Vector3f> points_3d(scanSize);
        std::vector<Eigen::Vector3f> points_3d_rgb(scanSize);
        std::vector<Eigen::Vector3f> rgb(scanSize);
        pcl::PCLPointCloud2 pcl;
        pcl_conversions::toPCL(*pcl_msg, pcl);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(pcl, cloud);

        // Bring the baseLink point cloud to the camera frame
        for(int i = 0; i < scanSize ; ++i)
        {
          Eigen::Vector3f p(cloud.points[i].x, cloud.points[i].y, 1.0); 
          points_3d_rgb[i] = p;
          rgb[i] = Eigen::Vector3f(0.0, 0.0, 255.0);
          Eigen::Vector3f p_trans = trans * p;
          points_3d[i] = p_trans;
        }

        std::vector<robomaster_msgs::msg::DetectedRobot> robots = detection_msg->robots;
        int robotNum = robots.size();
        std::vector<Eigen::Vector3f> robotPoses(robotNum);
        std::vector<int> detectedIndices;

        for (int r = 0; r < robotNum; ++r)
        {
          float x = robots[r].roi.x_offset * o_width;
          float y = robots[r].roi.y_offset * o_height;
          float h = robots[r].roi.height * o_height;
          float w = robots[r].roi.width * o_width;

          Eigen::Vector2f center(x, y);
          Eigen::Vector3d cameraRay = UV2CameraFrame(center);
          Eigen::Vector2f cameraRay2d(cameraRay(0), cameraRay(1));

          RCLCPP_INFO(this->get_logger(), "cameraRay2d (%f, %f)", cameraRay(0), cameraRay(1));

          float sim_max = 0.0;
          int sim_max_ind = -1;

          for(int i = 0; i < scanSize ; ++i)
          {
            // can consider a more sophisticated weighted average based on cosine similarity
            Eigen::Vector2f pnt(points_3d[i].head(2));
            pnt = pnt.normalized();
            float sim = cameraRay2d.dot(pnt);

            if (sim > o_normTH)
            {
                if(sim > sim_max)
                {
                  sim_max_ind = i;
                  sim_max = sim;
                  RCLCPP_INFO(this->get_logger(), "found match (%f, %f)", pnt(0), pnt(1));
                }
            }
          }
          if (sim_max_ind > -1)
          {
            robotPoses[r] = Eigen::Vector3f(cloud.points[sim_max_ind].x, cloud.points[sim_max_ind].y, 1.0);
            detectedIndices.push_back(sim_max_ind);
          }

        }

        for (int r = 0; r < robotNum; ++r)
        {
          cmcl_msgs::msg::RobotDetection msg;
          msg.header.stamp = detection_msg->header.stamp;
          msg.observer_id = o_robotID;
          msg.detected_id = -1;
          //msg.type = 0;
          msg.pose.position.x = robotPoses[r](0);
          msg.pose.position.y = robotPoses[r](1);
          o_detectionPub->publish(msg);
        }


        for (int i = 0;  i < detectedIndices.size() ; ++i)
        {
          rgb[detectedIndices[i]] = Eigen::Vector3f(255.0, 0.0, 0.0);
        }

        
        pcl::PointCloud<pcl::PointXYZRGB> pcl_rgb = Vec2RGBPointCloud(points_3d_rgb, rgb);
        sensor_msgs::msg::PointCloud2 rgb_pcl_msg;
        pcl::toROSMsg(pcl_rgb, rgb_pcl_msg);
        rgb_pcl_msg.header.stamp = detection_msg->header.stamp;
        rgb_pcl_msg.header.frame_id = "/RM0001/lidar";
        o_pointCloudPub->publish(rgb_pcl_msg);
    }

  private:

    std::string o_baseLinkTF;
    std::string o_cameraTF;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> scanSub;
    std::shared_ptr<message_filters::Subscriber<robomaster_msgs::msg::Detection>> detectionSub;
    std::shared_ptr<message_filters::Synchronizer<DepthSyncPolicy>> o_depthSync;
    rclcpp::Publisher<cmcl_msgs::msg::RobotDetection>::SharedPtr o_detectionPub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr o_camInfoSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr o_pointCloudPub;



    Eigen::Matrix3d o_invK;
    Eigen::Matrix3d o_T;
    int o_height = 480;
    int o_width = 640;
    int o_robotID = 0;
    float o_normTH;
    bool o_calib = false;



   

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Depth4DetectionNode>());
  rclcpp::shutdown();
  return 0;
}