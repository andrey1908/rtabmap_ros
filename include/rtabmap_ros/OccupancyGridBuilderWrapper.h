#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <optimization_results_msgs/OptimizationResults.h>

#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMutex.h>

#include "rtabmap_ros/CommonDataSubscriber.h"

#include <colored_occupancy_grid/ColoredOccupancyGrid.h>

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

	void load();
	void save();

	void updatePoses(const optimization_results_msgs::OptimizationResults::ConstPtr& optimizationResults);
	std::optional<rtabmap::Transform> getOptimizedPose(ros::Time time,
		const rtabmap::Transform* odometryPose = nullptr, ros::Duration maxExtrapolationTime = ros::Duration(0, 0),
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

	rtabmap::Signature createSignature(
		const rtabmap::Transform& pose,
		const ros::Time& time,
		const sensor_msgs::PointCloud2& scan3dMsg,
		const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
		const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs);

	void addSignatureToOccupancyGrid(const rtabmap::Signature& signature,
		const rtabmap::Transform& odometryPose, bool temporary = false);
	nav_msgs::OccupancyGrid getOccupancyGridMsg();
	void publishOccupancyGridMaps(ros::Time stamp, const std::string& frame_id);

	void publishLastDilatedSemantic(ros::Time stamp, const std::string& frame_id);

private:
	CommonDataSubscriber commonDataSubscriber_;
	CommonDataSubscriber temporaryCommonDataSubscriber_;

	ros::Publisher occupancyGridPub_;
	ros::Publisher coloredOccupancyGridPub_;
	ros::Publisher dilatedSemanticPub_;
	ros::Subscriber optimizationResultsSub_;

	tf::TransformListener tfListener_;

	int nodeId_;
	rtabmap::OccupancyGridBuilder occupancyGridBuilder_;

	std::map<int, ros::Time> times_;
	std::list<ros::Time> temporaryTimes_;
	std::list<rtabmap::Transform> temporaryOdometryPoses_;

	ros::Time lastOptimizationResultsTime_;
	std::list<tf2_ros::Buffer> trajectoryBuffers_;
	std::optional<rtabmap::Transform> odometryCorrection_;

	UMutex mutex_;

	std::string loadMapPath_;
	std::string saveMapPath_;

	bool cacheMap_;
	bool needsLocalization_;

	bool accumulativeMapping_;
	bool temporaryMapping_;

	std::string mapFrame_;
	std::string odomFrame_;
	std::string baseLinkFrame_;
};

}
