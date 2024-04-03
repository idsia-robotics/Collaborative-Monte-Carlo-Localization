/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MCLNode.cpp                                                    #
# ##############################################################################
**/  

#include <chrono>      
#include <functional> 
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "std_msgs/msg/string.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp> 

#include "cmcl_msgs/msg/det.hpp"
#include "cmcl_msgs/msg/particle_filter.hpp"
#include "cmcl_msgs/msg/cluster.hpp"
#include "cmcl_msgs/msg/coreset.hpp"    
#include "cmcl_msgs/msg/distribution_exchange.hpp"
#include "cmcl_msgs/msg/robot_detection.hpp"  
#include "cmcl_msgs/msg/prediction.hpp"  


#include <mutex> 
#include <sstream>  
#include <fstream>

#include "Utils.h"
#include "MCL.h"
#include "RosUtils.h"
#include <nlohmann/json.hpp>
#include "LidarData.h"
#include "Lidar2D.h"
#include "SetStatistics.h"

class MCLNode : public rclcpp::Node
{      
  public:
    MCLNode() : Node("MCLNode")
    {

      std::string mapDir;
      std::string cmclConfigPath;
      std::string mapName;
      std::string scanTopic;
      std::string odomTopic;
      std::string poseTopic;
      std::string predTopic;
      bool detection;
      std::string particleTopic;
      std::string gtTopic;
      std::string distExPubTopic;
      std::string distExSubTopic;    

      this->declare_parameter("mapDir", "");
      this->declare_parameter("mapName", "");
      this->declare_parameter("scanTopic", "");
      this->declare_parameter("particleTopic", "");
      this->declare_parameter("odomTopic", "");
      this->declare_parameter("odomNoise", std::vector<double>(3)); 
      this->declare_parameter("mapTopic", "");
      this->declare_parameter("cmclConfigPath", "");
      this->declare_parameter("triggerDist", 0.1);
      this->declare_parameter("triggerAngle", 0.03);
      this->declare_parameter("poseTopic", "");
      this->declare_parameter("predTopic", "");
      this->declare_parameter("baseLinkTF", "");
      this->declare_parameter("maxRange", 12.0); 
      this->declare_parameter("minRange", 0.05);
      this->declare_parameter("detection", false); 
      this->declare_parameter("robotID", 0);
 
      this->get_parameter("mapDir", mapDir);
      RCLCPP_INFO(this->get_logger(), "mapDir %s", mapDir.c_str());
      this->get_parameter("mapName", mapName);
      RCLCPP_INFO(this->get_logger(), "mapName %s", mapName.c_str());
      this->get_parameter("cmclConfigPath", cmclConfigPath);
      RCLCPP_INFO(this->get_logger(), "cmclConfigPath %s", cmclConfigPath.c_str());
      this->get_parameter("robotID", o_robotID);
      RCLCPP_INFO(this->get_logger(), "robotID %d", o_robotID);
     
      this->get_parameter("detection", detection);
      RCLCPP_INFO(this->get_logger(), "detection %d", detection);

      this->get_parameter("scanTopic", scanTopic);
      RCLCPP_INFO(this->get_logger(), "scanTopic %s", scanTopic.c_str());
      this->get_parameter("particleTopic", particleTopic);
      RCLCPP_INFO(this->get_logger(), "particleTopic %s", particleTopic.c_str());
      
      this->get_parameter("odomTopic", odomTopic);
      RCLCPP_INFO(this->get_logger(), "odomTopic %s", odomTopic.c_str());
      this->get_parameter("mapTopic", o_mapTopic);
      RCLCPP_INFO(this->get_logger(), "mapTopic %s", o_mapTopic.c_str());
      this->get_parameter("poseTopic", poseTopic);
      RCLCPP_INFO(this->get_logger(), "poseTopic %s", poseTopic.c_str());
      this->get_parameter("predTopic", predTopic);
      RCLCPP_INFO(this->get_logger(), "predTopic %s", predTopic.c_str());
      this->get_parameter("triggerDist", o_triggerDist);
      RCLCPP_INFO(this->get_logger(), "triggerDist %f", o_triggerDist);
      this->get_parameter("triggerAngle", o_triggerAngle);
      RCLCPP_INFO(this->get_logger(), "triggerAngle %f", o_triggerAngle);
      this->get_parameter("baseLinkTF", o_baseLinkTF);
      RCLCPP_INFO(this->get_logger(), "baseLinkTF %s", o_baseLinkTF.c_str());
      rclcpp::Parameter dblArrParam =this->get_parameter("odomNoise");
      std::vector<double> odomNoise = dblArrParam.as_double_array();
      RCLCPP_INFO(this->get_logger(), "odomNoise %f %f %f", odomNoise[0], odomNoise[1], odomNoise[2]);
      o_odomNoise = Eigen::Vector3f(odomNoise[0], odomNoise[1], odomNoise[2]); 


      //rclcpp::wait_for_message<nav_msgs::msg::OccupancyGrid>(mapMsg, this, "/map", std::chrono::duration<float, std::milli>(1000));

 
      o_mtx = new std::mutex();  
      std::string mapPath = mapName + std::string(".yaml");
      RCLCPP_INFO(this->get_logger(), "mapPath %s", mapPath.c_str());
      o_mcl = std::make_shared<MCL>(MCL(mapDir + cmclConfigPath));


      rclcpp::QoS qos(10);
      qos.best_effort();  

    
      o_posePub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(poseTopic, 10);
      o_predPub = this->create_publisher<cmcl_msgs::msg::Prediction>(predTopic, 10);

      o_particlePub = this->create_publisher<geometry_msgs::msg::PoseArray>(particleTopic, 10);

      o_odomSub = create_subscription<nav_msgs::msg::Odometry>(odomTopic, qos, std::bind(&MCLNode::motionCallback, this, std::placeholders::_1));
      o_scanSub = create_subscription<sensor_msgs::msg::PointCloud2>(scanTopic, qos, std::bind(&MCLNode::observationCallback, this, std::placeholders::_1));

      if (detection)
      {
        int distType;
        this->declare_parameter("distType", 0);
        this->get_parameter("distType", distType);
        o_distType = static_cast<DISTRIBUTION>(distType);
        RCLCPP_INFO(this->get_logger(), "distributionType %d", (uint8_t)o_distType);

        std::string detectionTopic;
        this->declare_parameter("detectionTopic", "");
        this->get_parameter("detectionTopic", detectionTopic);
        RCLCPP_INFO(this->get_logger(), "detectionTopic %s", detectionTopic.c_str());

        o_detectionSub = create_subscription<cmcl_msgs::msg::RobotDetection>(detectionTopic, qos, std::bind(&MCLNode::detectionCallback, this, std::placeholders::_1));

          this->declare_parameter("distExSubTopic", "");
        this->get_parameter("distExSubTopic", distExSubTopic);
        RCLCPP_INFO(this->get_logger(), "distExSubTopic %s", distExSubTopic.c_str());

          this->declare_parameter("distExPubTopic", "");
        this->get_parameter("distExPubTopic", distExPubTopic);
        RCLCPP_INFO(this->get_logger(), "distExPubTopic %s", distExPubTopic.c_str());

        o_distExSub = create_subscription<cmcl_msgs::msg::DistributionExchange>(distExSubTopic, qos, std::bind(&MCLNode::exchangeCallback, this, std::placeholders::_1));
        o_distExPub = this->create_publisher<cmcl_msgs::msg::DistributionExchange>(distExPubTopic, 10); 
      }   
      
      o_tfPub =  std::make_unique<tf2_ros::TransformBroadcaster>(*this);   
 
      RCLCPP_INFO(this->get_logger(), "Localization engine running!");     
    } 

    void detectionCallback(const cmcl_msgs::msg::RobotDetection::SharedPtr det_msg)       
    {        

      Eigen::Vector3f delta = o_prevDetectPose - o_prevPose;  

      Eigen::Vector2f dir1(cos(o_prevDetectPose(2)), sin(o_prevDetectPose(2)));
      Eigen::Vector2f dir2(cos(o_prevPose(2)), sin(o_prevPose(2)));
      double sim = dir1.dot(dir2) / (dir2.norm() * dir1.norm());

 
      if((((sqrt(delta(0) * delta(0) + delta(1) * delta(1))) > 1.0) || (sim < 0.9)))
      {

        cmcl_msgs::msg::DistributionExchange dist_msg;    
        dist_msg.header.stamp = det_msg->header.stamp;
        dist_msg.header.frame_id = o_mapTopic;

        dist_msg.detected_id = det_msg->detected_id;
        dist_msg.observer_id = det_msg->observer_id;
        dist_msg.observer_id = o_robotID;
        dist_msg.type = (uint8_t)o_distType;


        float yaw = 2 * atan2(det_msg->pose.orientation.z, det_msg->pose.orientation.w);
        Eigen::Vector3f transform = Eigen::Vector3f(det_msg->pose.position.x, det_msg->pose.position.y, yaw);

        if(transform.head(2).norm() > 10.0) return;

        RCLCPP_INFO(this->get_logger(), "Detection!"); 
        
        o_mtx->lock();
        std::shared_ptr<DistributionData> distData = o_mcl->CreateDistribution(o_distType, transform);
        SetStatistics stats = o_mcl->Stats();
        o_mtx->unlock(); 

        switch(o_distType) 
        {

            case DISTRIBUTION::PARTICLES:
            {
              std::vector<Particle> particles = distData->Particles();
              cmcl_msgs::msg::ParticleFilter pf;   

              pf.particles = std::vector<geometry_msgs::msg::Pose>(particles.size());
              pf.weights = std::vector<float>(particles.size());

              std::vector<float> variances(2); 
              Eigen::Matrix3d cov = stats.Cov();
              variances[0] = cov(0,0);
              variances[1] = cov(1,1);
              pf.variance = variances; 

              for (int i = 0; i < particles.size(); ++i)
              {
                geometry_msgs::msg::Pose p;
                p.position.x = particles[i].pose(0); 
                p.position.y = particles[i].pose(1);
                p.position.z = 0.1; 
                tf2::Quaternion q;
                q.setRPY( 0, 0, particles[i].pose(2)); 
                p.orientation.x = q[0];
                p.orientation.y = q[1];
                p.orientation.z = q[2];
                p.orientation.w = q[3];
                pf.particles[i] = p; 
                pf.weights[i] = particles[i].weight;
              } 

              dist_msg.filter = pf;
              break;
            }
            case DISTRIBUTION::DET:
            {
              cmcl_msgs::msg::DET det;
              det.tree = distData->DET();

              std::vector<float> variances(2); 
              Eigen::Matrix3d cov = stats.Cov();
              variances[0] = cov(0,0);
              variances[1] = cov(1,1);
              det.variance = variances; 
              dist_msg.det = det;
              break;    
            }  
            case DISTRIBUTION::KMEANS:      
            {
              std::vector<Cluster> clusters = distData->KMeans();
              int numClusters = clusters.size(); 
              std::vector<cmcl_msgs::msg::Cluster> kmeans(numClusters);

              for(int k = 0; k < numClusters; ++k)
              {
                std::vector<float> mean = {clusters[k].mean(0), clusters[k].mean(1), clusters[k].mean(2)};
                std::vector<float> variance = {clusters[k].variance(0), clusters[k].variance(1), 0.0};
                kmeans[k].centroid = mean;
                kmeans[k].variance = variance;
                kmeans[k].weight = clusters[k].totalWeight;
                //RCLCPP_INFO(this->get_logger(), "kmeans %d: %f %f %f %f", k, clusters[k].mean(0), clusters[k].mean(1), clusters[k].mean(2), clusters[k].totalWeight); 
              } 
              dist_msg.clusters = kmeans;
              break;
            }
 
            case DISTRIBUTION::PROROK:    
            {
              std::vector<ClusterAbstraction> abstractions = distData->Prorok();
              int numClusters = abstractions.size(); 
              std::vector<cmcl_msgs::msg::Cluster> prorok(numClusters);

              for(int k = 0; k < numClusters; ++k)      
              {
                std::vector<float> centroid = {abstractions[k].centroid(0),abstractions[k].centroid(1),abstractions[k].centroid(2)}; 
                std::vector<float> mu = {abstractions[k].mu(0), abstractions[k].mu(1)};
                std::vector<float> variance = {abstractions[k].covariance(0,0), abstractions[k].covariance(1,1)};

                prorok[k].centroid = centroid; 
                prorok[k].mu = mu;
                prorok[k].variance = variance;
                prorok[k].weight = abstractions[k].weight;
              }
              dist_msg.clusters = prorok;    

              break; 
            } 
            case DISTRIBUTION::GOODPOINTS:     
            {
              std::vector<Particle> particles = distData->Particles(); 
              cmcl_msgs::msg::Coreset coreset;   

              coreset.particles = std::vector<geometry_msgs::msg::Pose>(particles.size());
              coreset.weights = std::vector<float>(particles.size());

              std::vector<float> variances(2);  
              Eigen::Matrix3d cov = stats.Cov();
              variances[0] = cov(0,0);
              variances[1] = cov(1,1);
              coreset.variance = variances; 

              for (int i = 0; i < particles.size(); ++i)
              {
                geometry_msgs::msg::Pose p;
                p.position.x = particles[i].pose(0); 
                p.position.y = particles[i].pose(1);
                p.position.z = 0.1; 
                tf2::Quaternion q;
                q.setRPY( 0, 0, particles[i].pose(2)); 
                p.orientation.x = q[0];
                p.orientation.y = q[1];
                p.orientation.z = q[2];
                p.orientation.w = q[3];
                coreset.particles[i] = p; 
                coreset.weights[i] = particles[i].weight;
              }

              dist_msg.coreset = coreset;
              break;
            }
        }

        o_distExPub->publish(dist_msg);
        o_prevDetectPose =  o_prevPose;

      }
  
    }          

    void exchangeCallback(const cmcl_msgs::msg::DistributionExchange::SharedPtr dist_msg)
    {
        if (dist_msg->observer_id == o_robotID) return;

        std::shared_ptr<DistributionData> distData;
        DISTRIBUTION distType = static_cast<DISTRIBUTION>(dist_msg->type);

        switch(distType)  
        {
            case DISTRIBUTION::PARTICLES: 
            {
                int particleNum = dist_msg->filter.particles.size();
                std::vector<Particle> observerParticles = std::vector<Particle>(particleNum);

                for (int i = 0; i < particleNum; ++i)
                {
                  geometry_msgs::msg::Pose p = dist_msg->filter.particles[i];
                  float yaw = 2 * atan2(p.orientation.z, p.orientation.w);
                  observerParticles[i].pose = Eigen::Vector3f(p.position.x, p.position.y, yaw);
                  observerParticles[i].weight = dist_msg->filter.weights[i];
                }
                std::vector<float> variance = dist_msg->filter.variance;

                distData = std::make_shared<DistributionData>(DistributionData(distType, observerParticles, Eigen::Vector2f(variance[0], variance[1])));
                break;
            }
            case DISTRIBUTION::DET:
            {
              std::vector<float> variance = dist_msg->det.variance;
              distData = std::make_shared<DistributionData>(DistributionData(distType, dist_msg->det.tree, Eigen::Vector2f(variance[0], variance[1])));
              break;
            }
            case DISTRIBUTION::KMEANS:  
            {
              int numClusters = dist_msg->clusters.size();
              std::vector<Cluster> clusters(numClusters);
    
              for(int k = 0; k < numClusters; ++k)
              {
                std::vector<float> mean = dist_msg->clusters[k].centroid;
                std::vector<float> variance = dist_msg->clusters[k].variance;

                clusters[k].mean = Eigen::Vector3f(mean[0], mean[1], mean[2]);
                clusters[k].variance = Eigen::Vector3f(variance[0], variance[1], variance[2]);
                clusters[k].totalWeight = dist_msg->clusters[k].weight;
              }

              distData = std::make_shared<DistributionData>(DistributionData(distType, clusters));
              break;
            } 
            case DISTRIBUTION::PROROK:
            {
              int numClusters = dist_msg->clusters.size();

              std::vector<ClusterAbstraction> clusterAbstractions(numClusters);
 
              for(int k = 0; k < numClusters; ++k)
              {
                  std::vector<float> centroid = dist_msg->clusters[k].centroid;
                  std::vector<float> mu = dist_msg->clusters[k].mu;
                  std::vector<float> variance = dist_msg->clusters[k].variance;
                  Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
                  covariance(0,0) = variance[0];
                  covariance(1,1) = variance[1]; 

                  clusterAbstractions[k].centroid = Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
                  clusterAbstractions[k].mu = Eigen::Vector2f(mu[0], mu[1]);
                  clusterAbstractions[k].covariance = covariance;
                  clusterAbstractions[k].weight = dist_msg->clusters[k].weight;

              }

              distData = std::make_shared<DistributionData>(DistributionData(distType, clusterAbstractions));
              break;
            }
            case DISTRIBUTION::GOODPOINTS:     
            {
              int particleNum = dist_msg->coreset.particles.size();
              std::vector<Particle> observerParticles = std::vector<Particle>(particleNum);

              for (int i = 0; i < particleNum; ++i)
              {
                geometry_msgs::msg::Pose p = dist_msg->coreset.particles[i];
                float yaw = 2 * atan2(p.orientation.z, p.orientation.w);
                observerParticles[i].pose = Eigen::Vector3f(p.position.x, p.position.y, yaw);
                observerParticles[i].weight = dist_msg->coreset.weights[i];
              }
              std::vector<float> variance = dist_msg->coreset.variance; 

              distData = std::make_shared<DistributionData>(DistributionData(distType, observerParticles, Eigen::Vector2f(variance[0], variance[1])));

              break;   
            }   
        }     

        o_mtx->lock();
        o_mcl->FuseDistribution(distData);    
        std::vector<Particle> particles = o_mcl->Particles();     
        SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
        o_mtx->unlock(); 

        o_pred = stats.Mean();   

        if(o_pred.array().isNaN().any() || o_cov.array().isNaN().any() || o_cov.array().isInf().any())
        { 
          RCLCPP_FATAL(this->get_logger(), "Exchange::MCL fails to Localize!");
          o_mcl->Recover();
        }
        else
        {
            publishPose(dist_msg->header.stamp, 2);
            publishParticles(particles, dist_msg->header.stamp);
            publishTF(dist_msg->header.stamp);
        } 
        RCLCPP_INFO(this->get_logger(), "Fusing!");

    }



    void motionCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
      if (o_first)
      {
        o_first = false;
        o_prevPose = OdomMsg2Pose2D(odom_msg);
        return;
      }

      Eigen::Vector3f currPose = OdomMsg2Pose2D(odom_msg);
      Eigen::Vector3f delta = currPose - o_prevPose;


      if((((sqrt(delta(0) * delta(0) + delta(1) * delta(1))) > o_triggerDist) || (fabs(delta(2)) > o_triggerAngle)) || o_first)
      {

        Eigen::Vector3f u = o_mcl->Backward(o_prevPose, currPose);

        std::vector<Eigen::Vector3f> command{u};
        o_mtx->lock();
        o_mcl->Predict(command, o_odomWeights, o_odomNoise);
        //SetStatistics stas = o_mcl->Stats();
        std::vector<Particle> particles = o_mcl->Particles();   
        SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
        o_mtx->unlock(); 

        o_prevPose = currPose; 
        o_pred = stats.Mean();   
        o_cov = stats.Cov();

        if(o_pred.array().isNaN().any() || o_cov.array().isNaN().any() || o_cov.array().isInf().any())
        { 
          RCLCPP_FATAL(this->get_logger(), "Motion::MCL fails to Localize!");
          o_mcl->Recover();
        }
        else
        {

          publishPose(odom_msg->header.stamp, 0);
          publishParticles(particles, odom_msg->header.stamp);
          }

        o_step = true; 
      }
      else
      {
        if(o_pred.array().isNaN().any() || o_cov.array().isNaN().any() || o_cov.array().isInf().any())
        { 
        }
        else publishTF(odom_msg->header.stamp);
      }
    }
 
    void observationCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
    {
      if (o_firstScan)
      {
        o_firstScan = false;
        int scanSize = pcl_msg->width;
        o_scanMask = std::vector<double>(666, 1.0);
        return; 
      }
      if (o_step) 
      { 
      
        int scanSize = pcl_msg->width;
        std::vector<Eigen::Vector3f> points_3d(scanSize);

        pcl::PCLPointCloud2 pcl;
        pcl_conversions::toPCL(*pcl_msg, pcl);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(pcl, cloud);

        for(int i = 0; i < scanSize ; ++i)
        {
          points_3d[i] = Eigen::Vector3f(cloud.points[i].x, cloud.points[i].y, 1.0); 
          Eigen::Vector2f point_2d = Eigen::Vector2f(cloud.points[i].x, cloud.points[i].y);
          float dist = point_2d.norm();
          if ((dist > o_maxRange) || (dist < o_minRange))
          {
            o_scanMask[i] = 0.0;
          }
          else o_scanMask[i] = 1.0;
        }
        
        LidarData data = LidarData(points_3d, o_scanMask);
        o_mtx->lock();
        o_mcl->Correct(std::make_shared<LidarData>(data)); 
        std::vector<Particle> particles = o_mcl->Particles();    
        SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);
        o_mtx->unlock();    
        o_pred = stats.Mean();  
        o_cov = stats.Cov();

        if(o_pred.array().isNaN().any() || o_cov.array().isNaN().any() || o_cov.array().isInf().any())
        { 
          RCLCPP_FATAL(this->get_logger(), "Observation::MCL fails to Localize!");
          o_mcl->Recover();
        }
        else
        {
            publishPose(pcl_msg->header.stamp, 1);
            publishParticles(particles, pcl_msg->header.stamp);
            publishTF(pcl_msg->header.stamp);
        } 
 

        o_step = false;  
      }  
      if(o_firstParticles)  
      {
        o_firstParticles = false;
        o_mtx->lock();
        std::vector<Particle> particles = o_mcl->Particles();  
        SetStatistics stats = SetStatistics::ComputeParticleSetStatistics(particles);  
        o_mtx->unlock();    
        // o_pred = stats.Mean(); 
        // o_cov = stats.Cov();
        // publishPose(pcl_msg->header.stamp, -1);
        publishParticles(particles, pcl_msg->header.stamp);
      }
    }
 
     void publishPose(const rclcpp::Time& stamp, int type)  
     {
        geometry_msgs::msg::PoseWithCovarianceStamped poseStamped = Pred2PoseWithCov(o_pred, o_cov);
        poseStamped.header.frame_id = o_mapTopic;
        poseStamped.header.stamp = stamp; 


        cmcl_msgs::msg::Prediction pred_msg;
        pred_msg.header.stamp = stamp; 
        pred_msg.header.frame_id = o_mapTopic;
        pred_msg.pose = poseStamped;
        pred_msg.type = type;

         o_posePub->publish(poseStamped); 
         o_predPub->publish(pred_msg);

     }

     void publishParticles(const std::vector<Particle>& particles, const rclcpp::Time& stamp)
     {
        geometry_msgs::msg::PoseArray posearray;
        posearray.header.stamp = stamp;  
        posearray.header.frame_id = o_mapTopic;
        posearray.poses = std::vector<geometry_msgs::msg::Pose>(particles.size());

        for (int i = 0; i < particles.size(); ++i)
        {   
          geometry_msgs::msg::Pose p;
          p.position.x = particles[i].pose(0); 
          p.position.y = particles[i].pose(1);
          p.position.z = 0.1; 
          tf2::Quaternion q;
          q.setRPY( 0, 0, particles[i].pose(2)); 
          p.orientation.x = q[0];
          p.orientation.y = q[1];
          p.orientation.z = q[2];
          p.orientation.w = q[3];

          posearray.poses[i] = p;
        }

        o_particlePub->publish(posearray);
     }

     void publishTF(const rclcpp::Time& stamp)
     {
        tf2::Transform tf_orig;
        tf2::Quaternion q;
        tf_orig.setOrigin(tf2::Vector3(o_pred(0), o_pred(1), 0.0));
        q.setRPY(0, 0, o_pred(2));

        tf_orig.setRotation(q);
        tf2::Transform tf_inv = tf_orig.inverse();
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;  
        t.header.frame_id = o_baseLinkTF.c_str();
        t.child_frame_id = o_mapTopic.c_str(); 
        t.transform.translation.x = tf_inv.getOrigin().x();
        t.transform.translation.y = tf_inv.getOrigin().y();
        t.transform.translation.z = tf_inv.getOrigin().z();
        t.transform.rotation.x = tf_inv.getRotation().x(); 
        t.transform.rotation.y = tf_inv.getRotation().y();
        t.transform.rotation.z = tf_inv.getRotation().z();
        t.transform.rotation.w = tf_inv.getRotation().w();
        o_tfPub->sendTransform(t);     
     }


  private:   
 
    

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr o_posePub;
    rclcpp::Publisher<cmcl_msgs::msg::Prediction>::SharedPtr o_predPub;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr o_gtPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr o_particlePub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr o_odomSub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr o_scanSub;

    rclcpp::Subscription<cmcl_msgs::msg::RobotDetection>::SharedPtr o_detectionSub;
    rclcpp::Subscription<cmcl_msgs::msg::DistributionExchange>::SharedPtr o_distExSub;
    rclcpp::Publisher<cmcl_msgs::msg::DistributionExchange>::SharedPtr o_distExPub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> o_tfPub;

    //tf2_ros::TransformBroadcaster o_tfPub;


    
    std::shared_ptr<MCL> o_mcl;  
    DISTRIBUTION o_distType = DISTRIBUTION::PARTICLES;
    std::vector<float> o_odomWeights = {1.0};
    Eigen::Vector3f o_prevPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f o_prevTriggerPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f o_prevDetectPose = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f o_odomNoise = Eigen::Vector3f(0.02, 0.02, 0.02);
    Eigen::Vector3d o_pred;
    Eigen::Matrix3d o_cov;
    std::string o_maskTopic;

  
    int o_dsFactor = 1;
    float o_minRange = 0.05;
    float o_maxRange = 12.0;
    std::vector<double> o_scanMask;
    std::string o_mapTopic;
    std::string o_baseLinkTF;
    bool o_first = true;
    bool o_firstDetect =true;
    bool o_firstParticles = true;
    bool o_step = false;
    bool o_firstScan = true;
    int o_robotID = 0;
    int debugCounter = 0;

    std::mutex* o_mtx;

    float o_triggerDist = 0.05;
    float o_triggerAngle = 0.05;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCLNode>());
  rclcpp::shutdown();
  return 0;
}