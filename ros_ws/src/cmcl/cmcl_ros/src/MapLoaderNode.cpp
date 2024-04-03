/**
# ##############################################################################
#  Copyright (c) 2023- University of Lugano                                    #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MapLoaderNode.cpp                                                     #
# ##############################################################################
**/  

#include <chrono>     
#include <functional> 
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav2_map_server/map_server.hpp"
#include "nav2_map_server/map_io.hpp"
#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>
#include <exception>


using namespace nav2_map_server; 
class MapLoaderNode : public rclcpp::Node
{      
  public:
    MapLoaderNode() : Node("MapLoaderNode")
    {

    	std::string cmclConfigPath;
      	std::string mapTopic;
      	this->declare_parameter("mapTopic", "");
        this->declare_parameter("cmclConfigPath", "");
        this->get_parameter("mapTopic", mapTopic);
		RCLCPP_INFO(this->get_logger(), "mapTopic %s", mapTopic.c_str());
		this->get_parameter("cmclConfigPath", cmclConfigPath);
		RCLCPP_INFO(this->get_logger(), "cmclConfigPath %s", cmclConfigPath.c_str());

    	//std::string cmclConfigPath = "/ros_ws/data/cmcl.config";
    	using json = nlohmann::json;
		std::ifstream file(cmclConfigPath);
		json config;
		try
		{
			file >> config;
		}
		catch(std::exception& e)
		{
			std::cerr << "Failed to load config fail" << std::endl;
			std::cout << e.what() << '\n';
		}

		std::string folderPath = boost::filesystem::path(cmclConfigPath).parent_path().string() + "/";
		std::string mapConfigPath = folderPath + std::string(config["mapPath"]);

    	nav_msgs::msg::OccupancyGrid mapMsg;
    	LoadParameters params = loadMapYaml(mapConfigPath);
    	//params.image_file_name = params.image_file_name;
    	loadMapFromFile(params, mapMsg);
    	o_mapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mapTopic, 1);


    	int cnt = 0;
    	while(cnt < 100000000)
    	{
      		o_mapPub->publish(mapMsg); 
      		++cnt;
      	}

      	RCLCPP_INFO(this->get_logger(), "MapLoaderNode running!");     


    }

   private:

   	    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr o_mapPub;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLoaderNode>());
  rclcpp::shutdown();
  return 0;
}