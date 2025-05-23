#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
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
#include <slam_communication_msgs/OptimizationResults.h>
#include <slam_communication_msgs/NodesToRemove.h>

#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/ObjectTracking.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/core/Serialization.h>

#include <rtabmap_ros/DataSubscriber.h>

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
    void readRosParameters(const ros::NodeHandle& pnh, const YAML::Node& config);
    std::string occupancyGridTopicPostfix(int index, int numBuilders);

    void updatePoses(const slam_communication_msgs::OptimizationResults::ConstPtr& optimizationResults);

    void dataCallback(
        const nav_msgs::Odometry& globalOdometryMsg,
        const nav_msgs::Odometry& localOdometryMsg,
        const sensor_msgs::PointCloud2& pointCloudMsg,
        const std::vector<sensor_msgs::CameraInfoConstPtr>& cameraInfoMsgs,
        const std::vector<sensor_msgs::ImageConstPtr>& imageMsgs,
        bool temporaryMapping);

    rtabmap::SensorData createSensorData(
        const sensor_msgs::PointCloud2& scan3dMsg,
        const std::vector<sensor_msgs::CameraInfoConstPtr>& cameraInfoMsgs,
        const std::vector<sensor_msgs::ImageConstPtr>& imageMsgs);
    void addSensorDataToOccupancyGrid(const rtabmap::SensorData& sensorData,
        const rtabmap::Time& time,
        const rtabmap::Transform& localPose, const rtabmap::Transform& globalPose,
        bool temporary);
    void publishOccupancyGridMaps(const rtabmap::Time& time);
    void publishLastDilatedSemantic(const ros::Time& stamp,
        const std::string& frame_id);
    void publishTrackedObjects(const ros::Time& stamp,
        const std::vector<rtabmap::ObjectTracking::TrackedObject>& trackedObjects);
    void publishSensorIgnoreAreas(const ros::Time& stamp, const std::string& sensorFrame,
        const std::vector<rtabmap::LocalMapBuilder2d::Area>& sensorIgnoreAreas);
    static std::vector<geometry_msgs::Point> createCube(float length, float width, float height);

private:
    DataSubscriber temporaryDataSubscriber_;
    DataSubscriber dataSubscriber_;

    std::vector<ros::Publisher> occupancyGridPubs_;
    std::vector<ros::Publisher> coloredOccupancyGridPubs_;
    ros::Publisher dilatedSemanticPub_;
    ros::Publisher trackedObjectsPub_;
    ros::Publisher sensorIgnoreAreasPub_;

    ros::Subscriber optimizationResultsSub_;
    ros::Publisher nodesToRemovePub_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    std::unique_ptr<rtabmap::OccupancyGridMap> occupancyGridMap_;

    std::unique_ptr<rtabmap::RawDataSerialization> rawDataWriter_;

    std::string mapFrame_;
    std::string odomFrame_;
    std::string baseLinkFrame_;
    std::string updatedPosesFrame_;

    std::vector<std::string> configPaths_;
    std::string loadMapPath_;
    std::string saveMapPath_;
    std::string saveTrackingResultsPath_;
    std::string saveRawDataPath_;

    bool needsLocalization_;
    bool temporaryMapping_;
    bool accumulativeMapping_;
};

}
