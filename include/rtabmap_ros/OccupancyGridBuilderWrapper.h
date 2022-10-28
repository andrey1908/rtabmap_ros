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

#include <rtabmap/core/OccupancyGrid.h>
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

class OccupancyGridBuilder : public CommonDataSubscriber {
public:
	OccupancyGridBuilder(int argc, char** argv);
	~OccupancyGridBuilder();

private:
	rtabmap::ParametersMap readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh);
	void readRtabmapRosParameters(const ros::NodeHandle& pnh);

	void load();
	void save();

	void updatePoses(const optimization_results_msgs::OptimizationResults::ConstPtr& optimizationResults);
	std::optional<rtabmap::Transform> getPose(ros::Time time,
		const rtabmap::Transform* odometryPose = nullptr, ros::Duration maxDistance = ros::Duration(0, 0),
		bool defaultIdentityOdometryCorrection = false);

	virtual void commonDepthCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
			const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
			const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
			const sensor_msgs::LaserScan& scanMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
			const std::vector<rtabmap_ros::GlobalDescriptor>& globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
			const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPoints = std::vector<std::vector<rtabmap_ros::KeyPoint>>(),
			const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3d = std::vector<std::vector<rtabmap_ros::Point3f>>(),
			const std::vector<cv::Mat>& localDescriptors = std::vector<cv::Mat>());
	virtual void commonStereoCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const cv_bridge::CvImageConstPtr& leftImageMsg,
			const cv_bridge::CvImageConstPtr& rightImageMsg,
			const sensor_msgs::CameraInfo& leftCamInfoMsg,
			const sensor_msgs::CameraInfo& rightCamInfoMsg,
			const sensor_msgs::LaserScan& scanMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
			const std::vector<rtabmap_ros::GlobalDescriptor>& globalDescriptorMsgs = std::vector<rtabmap_ros::GlobalDescriptor>(),
			const std::vector<rtabmap_ros::KeyPoint>& localKeyPoints = std::vector<rtabmap_ros::KeyPoint>(),
			const std::vector<rtabmap_ros::Point3f>& localPoints3d = std::vector<rtabmap_ros::Point3f>(),
			const cv::Mat& localDescriptors = cv::Mat()) {};
	virtual void commonLaserScanCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const sensor_msgs::LaserScan& scanMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
			const rtabmap_ros::GlobalDescriptor& globalDescriptor = rtabmap_ros::GlobalDescriptor());
	virtual void commonOdomCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg) {};

	rtabmap::Signature createSignature(
			const rtabmap::Transform& pose,
			const ros::Time& time,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
			const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
			const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs);

	void addSignatureToOccupancyGrid(const rtabmap::Signature& signature,
			const rtabmap::Transform& odometryPose, bool temporary = false);
	nav_msgs::OccupancyGrid getOccupancyGridMap();
	void publishOccupancyGridMaps(ros::Time stamp, const std::string& frame_id);

	void publishLastDilatedSemantic(ros::Time stamp, const std::string& frame_id);

private:
	ros::Publisher occupancyGridPub_;
	ros::Publisher coloredoccupancyGridPub_;
	ros::Publisher dilatedSemanticPub_;
	ros::Subscriber optimizationResultsSub_;

	tf::TransformListener tfListener_;

	int nodeId_;
	rtabmap::OccupancyGrid occupancyGrid_;

	std::map<int, ros::Time> times_;

	std::list<rtabmap::Transform> temporaryOdometryPoses_;
	std::list<ros::Time> temporaryTimes_;

	ros::Time lastOptimizationResultsTime_;
	std::list<tf2_ros::Buffer> trajectoryBuffers_;
	std::unique_ptr<geometry_msgs::TransformStamped> odometryCorrection_;

	UMutex mutex_;

	std::string mapPath_;
	bool loadMap_;
	bool saveMap_;

	bool cacheLoadedMap_;
	bool needsLocalization_;

	bool temporaryMapping_;

	std::string mapFrame_;
	std::string odomFrame_;
	std::string baseLinkFrame_;
};

}
