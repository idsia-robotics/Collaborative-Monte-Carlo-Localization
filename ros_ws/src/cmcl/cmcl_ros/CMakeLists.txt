cmake_minimum_required(VERSION 3.5)
project(cmcl_ros)

SET(OpenCV_MIN_VERSION "4.2.0")


#colcon build --allow-overriding cv_bridge image_geometry

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/lib/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ncore/include/)


set(RESULTS_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/results)
file(MAKE_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RESULTS_OUTPUT_DIRECTORY}/bin)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/ncore/build/lib)




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV ${OpenCV_MIN_VERSION} REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cmcl_msgs REQUIRED)
find_package(robomaster_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(pybind11_DIR /usr/local/lib/python3.10/dist-packages/)
set(PYTHON_INCLUDE_DIR "/usr/include/python3.10")
find_package (Python COMPONENTS Interpreter Development)
if(Python_FOUND)
  message(STATUS "Found Python version ${Python_VERSION}")
  include_directories(${PYTHON_INCLUDE_DIR})
endif(Python_FOUND)
find_package(pybind11 REQUIRED)

find_package(Armadillo REQUIRED)
if (ARMADILLO_FOUND)
   include_directories(${ARMADILLO_INCLUDE_DIRS})
endif()


find_package(LAPACK REQUIRED)
include_directories(${LAPACK_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_subdirectory("src")
add_library(NCORE STATIC IMPORTED)
set_target_properties(NCORE PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/ncore/build/lib/libNCORE.a)

add_executable(MCLNode src/MCLNode.cpp src/RosUtils.cpp)
add_executable(LidarScan2PointCloudNode src/LidarScan2PointCloudNode.cpp src/RosUtils.cpp)
add_executable(DistExNode src/DistExNode.cpp src/RosUtils.cpp)
add_executable(EvalNode src/EvalNode.cpp src/RosUtils.cpp)
add_executable(Depth4DetectionNode src/Depth4DetectionNode.cpp src/RosUtils.cpp)
add_executable(UVDetection2WorldNode src/UVDetection2WorldNode.cpp src/RosUtils.cpp)
add_executable(DetectionNoiseNode src/DetectionNoiseNode.cpp src/RosUtils.cpp)
add_executable(RelativePosedNode src/RelativePosedNode.cpp src/RosUtils.cpp)
add_executable(OprtitrackPublisherNode src/OprtitrackPublisherNode.cpp src/RosUtils.cpp)
add_executable(MapLoaderNode src/MapLoaderNode.cpp src/RosUtils.cpp)



target_link_libraries(MCLNode NCORE ${Boost_SYSTEM_LIBRARY} ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} pybind11::embed)
target_link_libraries(LidarScan2PointCloudNode NCORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
target_link_libraries(DistExNode NCORE ${Boost_SYSTEM_LIBRARY} ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} pybind11::embed)
target_link_libraries(EvalNode NCORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
target_link_libraries(Depth4DetectionNode NCORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
target_link_libraries(UVDetection2WorldNode NCORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
target_link_libraries(DetectionNoiseNode NCORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
target_link_libraries(RelativePosedNode NCORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
target_link_libraries(OprtitrackPublisherNode NCORE ${Boost_SYSTEM_LIBRARY} pybind11::embed)
target_link_libraries(MapLoaderNode NCORE ${Boost_SYSTEM_LIBRARY})


ament_target_dependencies(MCLNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(LidarScan2PointCloudNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(DistExNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(EvalNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(Depth4DetectionNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs robomaster_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(UVDetection2WorldNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs robomaster_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros cv_bridge pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(DetectionNoiseNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs robomaster_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros cv_bridge pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(RelativePosedNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs robomaster_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros cv_bridge pcl_conversions nlohmann_json OpenCV)


ament_target_dependencies(OprtitrackPublisherNode rclcpp std_msgs sensor_msgs nav_msgs cmcl_msgs robomaster_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros cv_bridge pcl_conversions nlohmann_json OpenCV)

ament_target_dependencies(MapLoaderNode rclcpp std_msgs sensor_msgs nav2_map_server nav_msgs cmcl_msgs robomaster_msgs geometry_msgs message_filters tf2 tf2_ros tf2_msgs pcl_ros cv_bridge pcl_conversions nlohmann_json OpenCV)


install(TARGETS
 MCLNode
 LidarScan2PointCloudNode
 DistExNode
 EvalNode
 Depth4DetectionNode
 UVDetection2WorldNode
 DetectionNoiseNode
 RelativePosedNode
 OprtitrackPublisherNode
 MapLoaderNode
DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
 python_cmcl
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
