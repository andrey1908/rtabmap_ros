#pragma once

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <colored_occupancy_grid_msgs/ColoredOccupancyGrid.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>

#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Grid.h>

namespace rtabmap_ros {

rtabmap::Time fromRos(const ros::Time& rosTime);
ros::Time toRosTime(const rtabmap::Time& time);
ros::Duration toRosDuration(const rtabmap::Time& duration);

rtabmap::Transform fromRos(const geometry_msgs::Pose& msg);
rtabmap::Transform fromRos(const geometry_msgs::Transform& msg);
geometry_msgs::Pose toRosPose(const rtabmap::Transform& pose);
geometry_msgs::Transform toRosTransform(const rtabmap::Transform& transform);

rtabmap::SensorData::CameraParameters fromRos(
    const sensor_msgs::CameraInfo& msg);
rtabmap::SensorData::CameraData fromRos(
    const sensor_msgs::CameraInfo& infoMsg,
    const sensor_msgs::Image& msg,
    const tf2_ros::Buffer& tfBuffer,
    const std::string& baseFrame,
    const ros::Duration& timeout);
rtabmap::SensorData::LidarData fromRos(
    const sensor_msgs::PointCloud2& msg,
    const tf2_ros::Buffer& tfBuffer,
    const std::string& baseFrame,
    const ros::Duration& timeout);

nav_msgs::OccupancyGrid toRos(
    const rtabmap::OccupancyGrid& grid,
    const rtabmap::Time& time,
    const std::string& frameId,
    float cellSize);
colored_occupancy_grid_msgs::ColoredOccupancyGrid toRos(
    const rtabmap::OccupancyGrid& grid,
    const rtabmap::ColorGrid& colors,
    const rtabmap::Time& time,
    const std::string& frameId,
    float cellSize);

}
