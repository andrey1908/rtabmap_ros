#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <colored_occupancy_grid_msgs/ColoredOccupancyGrid.h>
#include <optimization_results_msgs/OptimizationResults.h>

#include <rtabmap/core/TimedOccupancyGridMap.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/ObjectTracking.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMutex.h>

#include "rtabmap_ros/CommonDataSubscriber.h"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <optional>

namespace rtabmap_ros {

class OccupancyGridMapWrapper {
public:
    OccupancyGridMapWrapper(int argc, char** argv);
    ~OccupancyGridMapWrapper();

private:
    void readRosParameters(const ros::NodeHandle& pnh, const YAML::Node& params);
    std::string occupancyGridTopicPostfix(int index, int numBuilders);

    void updatePoses(const optimization_results_msgs::OptimizationResults::ConstPtr& optimizationResults);

    void commonLaserScanCallback(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const sensor_msgs::LaserScan& scanMsg,
        const sensor_msgs::PointCloud2& scan3dMsg,
        bool temporaryMapping);
    void commonRGBCallback(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
        const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
        const sensor_msgs::LaserScan& scanMsg,
        const sensor_msgs::PointCloud2& scan3dMsg,
        bool temporaryMapping);

    void mappingPipeline(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const sensor_msgs::PointCloud2& scan3dMsg,
        const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
        const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
        bool temporaryMapping);

    rtabmap::Signature createSignature(
        const rtabmap::Transform& pose,
        const ros::Time& time,
        const sensor_msgs::PointCloud2& scan3dMsg,
        const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
        const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs);
    void addSignatureToOccupancyGrid(const rtabmap::Signature& signature,
        bool temporary = false);
    void publishOccupancyGridMaps(const ros::Time& stamp);
    void publishLastDilatedSemantic(const ros::Time& stamp,
        const std::string& frame_id);
    void publishTrackedObjects(const ros::Time& stamp,
        const std::vector<rtabmap::ObjectTracking::TrackedObject>& trackedObjects);

    nav_msgs::OccupancyGrid getOccupancyGridMsg(const ros::Time& stamp, int index);
    void fillColorsInColoredOccupancyGridMsg(
        colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg,
        int index);

private:
    CommonDataSubscriber commonDataSubscriber_;
    CommonDataSubscriber temporaryCommonDataSubscriber_;

    std::vector<ros::Publisher> occupancyGridPubs_;
    std::vector<ros::Publisher> coloredOccupancyGridPubs_;
    ros::Publisher dilatedSemanticPub_;
    ros::Publisher trackedObjectsPub_;
    ros::Subscriber optimizationResultsSub_;

    tf::TransformListener tfListener_;

    int nodeId_;
    std::unique_ptr<rtabmap::TimedOccupancyGridMap> timedOccupancyGridMap_;
    rtabmap::Transform globalToOdometry_;

    UMutex mutex_;

    std::string mapFrame_;
    std::string odomFrame_;
    std::string baseLinkFrame_;
    std::string updatedPosesFrame_;

    std::string configPath_;
    std::string loadMapPath_;
    std::string saveMapPath_;

    bool needsLocalization_;
    bool accumulativeMapping_;
    bool temporaryMapping_;
};

}
