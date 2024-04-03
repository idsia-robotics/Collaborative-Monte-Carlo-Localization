/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: UVDetection2WorldNode.cpp                                             #
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
#include <sensor_msgs/msg/image.hpp>
#include <robomaster_msgs/msg/detection.hpp>
#include <robomaster_msgs/msg/detected_robot.hpp>
#include "cmcl_msgs/msg/robot_detection.hpp"   
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, robomaster_msgs::msg::Detection> DepthSyncPolicy;


class UVDetection2WorldNode : public rclcpp::Node
{
  public:
    UVDetection2WorldNode()
    : Node("UVDetection2WorldNode")
    {
      std::string visionTopic;
      std::string cameraInfoTopic;
      std::string cameraColorTopic;
      std::string estimated3DDetectionTopic;
      std::string pointCloudTopic;

      this->declare_parameter("visionTopic" ,"");
      // this->declare_parameter("cameraInfoTopic" ,"");
      // this->declare_parameter("cameraColorTopic" ,"");
      this->declare_parameter("estimated3DDetectionTopic" ,"");
      this->declare_parameter("baseLinkTF", "");
      this->declare_parameter("cameraTF", "");
      this->declare_parameter("robotID", 0);


      this->get_parameter("visionTopic", visionTopic);
      RCLCPP_INFO(this->get_logger(), "visionTopic %s", visionTopic.c_str());
      // this->get_parameter("cameraInfoTopic", cameraInfoTopic);
      // RCLCPP_INFO(this->get_logger(), "cameraInfoTopic %s", cameraInfoTopic.c_str());
      //  this->get_parameter("cameraColorTopic", cameraColorTopic);
      // RCLCPP_INFO(this->get_logger(), "cameraColorTopic %s", cameraColorTopic.c_str());
      this->get_parameter("estimated3DDetectionTopic", estimated3DDetectionTopic);
      RCLCPP_INFO(this->get_logger(), "estimated3DDetectionTopic %s", estimated3DDetectionTopic.c_str());

      this->get_parameter("baseLinkTF", o_baseLinkTF);
      RCLCPP_INFO(this->get_logger(), "baseLinkTF %s", o_baseLinkTF.c_str());
      this->get_parameter("cameraTF", o_cameraTF);
      RCLCPP_INFO(this->get_logger(), "cameraTF %s", o_cameraTF.c_str());

      this->get_parameter("robotID", o_robotID);
      RCLCPP_INFO(this->get_logger(), "robotID %d", o_robotID);

      rclcpp::QoS qos(10);
      qos.best_effort();
      //o_camInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(cameraInfoTopic, qos, std::bind(&UVDetection2WorldNode::calibrationCallback, this, std::placeholders::_1));
          
      // rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
      // custom_qos.depth=10;
      // custom_qos.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      // custom_qos.durability=RMW_QOS_POLICY_DURABILITY_VOLATILE;




      o_height = 360;
      o_width = 640;
      RCLCPP_INFO(this->get_logger(), "camInf (w, h) = (%d %d)", o_width, o_height);

      Eigen::Matrix3f K;
      K << 317.027917, 0.000000, 317.258112, 0.000000, 316.260383, 182.581540, 0.000000, 0.000000, 1.000000;
      o_invK = K.inverse();

      o_fy = K(1,1);
      //load T matrix 
      std::vector<double> t = {0, 0, 1, -1, 0, 0, 0, -1, 0};
      Eigen::Matrix3d T = Eigen::Matrix<double,3,3,Eigen::RowMajor>(t.data());
      o_T = T.cast<float>(); 
      o_calib = true;

      o_detectionPub = this->create_publisher<cmcl_msgs::msg::RobotDetection>(estimated3DDetectionTopic, 10);
      o_detectSub = create_subscription<robomaster_msgs::msg::Detection>(visionTopic, qos, std::bind(&UVDetection2WorldNode::callback, this, std::placeholders::_1));


      //imgSub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, cameraColorTopic);
      //detectionSub = std::make_shared<message_filters::Subscriber<robomaster_msgs::msg::Detection>>(this, visionTopic);

     // o_depthSync = std::make_shared<message_filters::Synchronizer<DepthSyncPolicy>>(DepthSyncPolicy(1), *imgSub, *detectionSub);
     // o_depthSync->registerCallback(std::bind(&UVDetection2WorldNode::callback, this, std::placeholders::_1, std::placeholders::_2)); 


      RCLCPP_INFO(this->get_logger(), "UVDetection2WorldNode running!");
    }

    void calibrationCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cam_info_msg)
    {
      
      RCLCPP_INFO(this->get_logger(), "camInf (w, h) = (%d %d)", cam_info_msg->width, cam_info_msg->height);

      o_height = cam_info_msg->height;
      o_width = cam_info_msg->width;
      auto k = cam_info_msg->k.data();
      Eigen::Matrix3d K_ = Eigen::Matrix<double,3,3,Eigen::RowMajor>(k);
      Eigen::Matrix3f K = K_.cast<float>(); 
      o_invK = K.inverse();

      o_fy = K(1,1);

      //load T matrix 
      std::vector<double> t = {0, 0, 1, -1, 0, 0, 0, -1, 0};
      Eigen::Matrix3d T = Eigen::Matrix<double,3,3,Eigen::RowMajor>(t.data());
      o_T = T.cast<float>(); 


      o_calib = true;
      o_camInfoSub.reset();
    }

    void getCameraTF()
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
          t = tf_buffer->lookupTransform(o_cameraTF.c_str(), o_baseLinkTF.c_str(), tf2::TimePointZero, tf2::Duration(00000000));
          /*RCLCPP_INFO(
          this->get_logger(), "Transform %s to %s: %f %f %f", o_cameraTF.c_str(), o_baseLinkTF.c_str(),
            t.transform.translation.x, t.transform.translation.y, GetYaw(t.transform.rotation.z, t.transform.rotation.w));
*/
          camPose = Eigen::Vector3f(t.transform.translation.x, t.transform.translation.y, GetYaw(t.transform.rotation.z, t.transform.rotation.w));
          o_trans = Vec2Trans(camPose);
          getTF = false;
          break;
        }
        catch (const tf2::TransformException & ex) {
            /*RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
              o_cameraTF.c_str(), o_baseLinkTF.c_str(), ex.what());*/
        }
      }
    }

    Eigen::Vector3f  UV2CameraFrame(Eigen::Vector2f q1)
    {
      Eigen::Vector3f p1(q1(0), q1(1), 1);

      // multiply by inverse calibration matrix
      Eigen::Vector3f p1k_ = o_invK * p1;

      // divide by z component to homogenize it
      Eigen::Vector3f p1k = p1k_ / p1k_(2);

      // go from image frame to camera frame
      Eigen::Vector3f p1c = o_T * p1k;

      return p1c;
    }

    int identifyRobot(cv::Mat& img)
    {
        cv::imwrite("frame" + std::to_string(frame) + ".png", img);
        

        ++frame;
    }


    void callback(const robomaster_msgs::msg::Detection::ConstSharedPtr& detection_msg)
    //void callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const robomaster_msgs::msg::Detection::ConstSharedPtr& detection_msg)
    {

        if (!o_calib) return;

        getCameraTF();

        std::vector<robomaster_msgs::msg::DetectedRobot> robots = detection_msg->robots;
        int robotNum = robots.size();
        std::vector<Eigen::Vector3f> robotPoses(robotNum);

        //cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::TYPE_8UC3);
        //cv::Mat frame = cvPtr->image;

        for (int r = 0; r < robotNum; ++r)
        {
          float x = robots[r].roi.x_offset * o_width;
          float y = robots[r].roi.y_offset * o_height;
          float h = robots[r].roi.height * o_height;
          float w = robots[r].roi.width * o_width;

          //Create the rectangle
          Eigen::Vector4f xywh(x - 0.5 * w, y - 0.5 * h, w, h);
          if (xywh(0) < 0)  xywh(0) = 0;
          if (xywh(1) < 0)  xywh(1) = 0;
          if (xywh(0) + xywh(2) >= o_width)  xywh(2) = o_width - xywh(0);
          if (xywh(1) + xywh(3) >= o_height)  xywh(3) = o_height - xywh(1);


          //RCLCPP_INFO(this->get_logger(), "roi (%f, %f, %f, %f)", x - 0.5 * w, y - 0.5 * h, w, h);
          //cv::Rect roi(xywh(0), xywh(1), xywh(2), xywh(3));
          //cv::Mat patch = frame(roi);
          //int robot_ID = identifyRobot(patch);

          Eigen::Vector2f center(x, y);
          Eigen::Vector3f cameraRay = UV2CameraFrame(center);
          //Eigen::Vector2f cameraRay2d(cameraRay(0), cameraRay(1));
          cameraRay = cameraRay.normalized();
          //RCLCPP_INFO(this->get_logger(), "cameraRay (%f, %f)", cameraRay(0), cameraRay(1));

          double distance = o_fy * o_robotH / h;
          // get pose in camera frame
          Eigen::Vector3f xyz_cam = distance * cameraRay;
          xyz_cam(2) = 1.0;
          // move to base_link frame
          Eigen::Vector3f xyz_base = o_trans * xyz_cam / 1000.0; 

          //RCLCPP_INFO(this->get_logger(), "xyz_base (%f, %f)", xyz_base(0), xyz_base(1)); 

          RCLCPP_INFO(this->get_logger(), "Detection!"); 

          cmcl_msgs::msg::RobotDetection msg;
          msg.header.stamp = detection_msg->header.stamp;
          msg.observer_id = o_robotID;
          msg.detected_id = 1 - o_robotID;
          msg.type = 1;
          msg.pose.position.x = xyz_base(0);
          msg.pose.position.y = xyz_base(1);
          o_detectionPub->publish(msg);       
        }

    }

  private:

    std::string o_baseLinkTF;
    std::string o_cameraTF;
    rclcpp::Subscription<robomaster_msgs::msg::Detection>::SharedPtr o_detectSub;
    rclcpp::Publisher<cmcl_msgs::msg::RobotDetection>::SharedPtr o_detectionPub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr o_camInfoSub;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> imgSub;
    std::shared_ptr<message_filters::Subscriber<robomaster_msgs::msg::Detection>> detectionSub;
    std::shared_ptr<message_filters::Synchronizer<DepthSyncPolicy>> o_depthSync;



    Eigen::Matrix3f o_invK;
    Eigen::Matrix3f o_trans;
    Eigen::Matrix3f o_T;
    int o_height = 480;
    int o_width = 640;
    int o_robotID = 0;
    bool o_calib = false;
    float o_robotH = 270; // in mm 
    float o_fy = 1.0;

    int frame = 0;



   

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UVDetection2WorldNode>());
  rclcpp::shutdown();
  return 0;
}