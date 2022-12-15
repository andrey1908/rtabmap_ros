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
#include <geometry_msgs/TransformStamped.h>
#include <colored_occupancy_grid_msgs/ColoredOccupancyGrid.h>
#include <optimization_results_msgs/OptimizationResults.h>
#include <rtabmap_ros_msgs/DoorCorners.h>

#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/DoorTracking.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMutex.h>

#include "rtabmap_ros/CommonDataSubscriber.h"

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <optional>

namespace rtabmap_ros {

class OccupancyGridBuilder {
public:
	OccupancyGridBuilder(int argc, char** argv);
	~OccupancyGridBuilder();

private:
	rtabmap::ParametersMap readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh);
	void readRtabmapRosParameters(const ros::NodeHandle& pnh);

	void save();
	void load();

	void updatePoses(const optimization_results_msgs::OptimizationResults::ConstPtr& optimizationResults);
	std::optional<rtabmap::Transform> getOptimizedPose(ros::Time time,
		const rtabmap::Transform* odometryPose = nullptr, ros::Duration maxExtrapolationTime = ros::Duration(0),
		bool defaultIdentityOdometryCorrection = false);

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
		const rtabmap::Transform& odometryPose, bool temporary = false);
	void publishOccupancyGridMaps(const ros::Time& stamp);
	void publishLastDilatedSemantic(const ros::Time& stamp, const std::string& frame_id);
	void tryToPublishDoorCorners(const ros::Time& stamp);

	nav_msgs::OccupancyGrid getOccupancyGridMsg(const ros::Time& stamp);
	void fillColorsInColoredOccupancyGridMsg(
		colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg);
	void maybeDrawDoorOnColoredOccupancyGridMsg(
		colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg);
	void drawDoorCenterUsedAsEstimationOnColoredOccupancyGrid(
		colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg,
		const cv::Vec3b& color);
	void drawDoorCornersOnColoredOccupancyGrid(
		colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg,
		const cv::Vec3b& color);

	void startDoorTracking(const geometry_msgs::PointConstPtr& doorCenterEstimation);
	void trackDoor();
	void stopDoorTracking(const std_msgs::EmptyConstPtr& empty);

private:
	CommonDataSubscriber commonDataSubscriber_;
	CommonDataSubscriber temporaryCommonDataSubscriber_;

	ros::Publisher occupancyGridPub_;
	ros::Publisher coloredOccupancyGridPub_;
	ros::Publisher dilatedSemanticPub_;
	ros::Publisher doorCornersPub_;
	ros::Subscriber optimizationResultsSub_;
	ros::Subscriber doorCenterEstimationSub_;
	ros::Subscriber stopDoorTrackingSub_;

	tf::TransformListener tfListener_;

	int nodeId_;
	rtabmap::OccupancyGridMap occupancyGridMap_;

	std::map<int, ros::Time> times_;
	std::map<int, rtabmap::Transform> posesAfterLastUpdate_;
	std::list<std::pair<ros::Time, rtabmap::Transform>> temporaryTimesPoses_;

	ros::Time lastOptimizedPoseTime_;
	std::set<ros::Time> optimizedPosesTimes_;
	std::list<tf2_ros::Buffer> trajectoryBuffers_;
	std::optional<rtabmap::Transform> odometryCorrection_;

	rtabmap::DoorTracking doorTracking_;
	rtabmap::DoorTracking::Cell doorCenterInMapFrame_;
	rtabmap::DoorTracking::Cell doorCenterUsedAsEstimationInMapFrame_;  // for visualization
	std::pair<rtabmap::DoorTracking::Cell, rtabmap::DoorTracking::Cell> doorCornersInMapFrame_;

	UMutex mutex_;

	std::string mapFrame_;
	std::string odomFrame_;
	std::string baseLinkFrame_;

	std::string loadMapPath_;
	std::string saveMapPath_;

	bool cacheMap_;
	bool needsLocalization_;
	double maxInterpolationTimeError_;
	double updateMaxExtrapolationTime_;

	bool accumulativeMapping_;
	bool temporaryMapping_;

	double doorTrackingSmallRadius_;
	double doorTrackingLargeRadius_;
	bool drawDoorCenterEstimation_;
	bool drawDoorCorners_;

	static constexpr int mapVersionWithoutSensorBlindRange = 1;
	static constexpr int latestMapVersion = 2;
};

}
