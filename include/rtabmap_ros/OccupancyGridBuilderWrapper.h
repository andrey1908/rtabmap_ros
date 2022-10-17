#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

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

namespace rtabmap_ros {

class OccupancyGridBuilder : public CommonDataSubscriber {
public:
	OccupancyGridBuilder(int argc, char** argv);
	~OccupancyGridBuilder();

private:
	rtabmap::ParametersMap readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh);
	void readRtabmapRosParameters(const ros::NodeHandle& pnh);

	void load();
	void loadAssembledOccupancyGrid();
	void loadOccupancyGridCache();
	void save();
	void saveAssembledOccupancyGrid();
	void saveOccupancyGridCache();

	void updatePoses(const nav_msgs::Path::ConstPtr& optimizedPoses);
	nav_msgs::OdometryConstPtr correctOdometry(nav_msgs::OdometryConstPtr odomMsg);

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

	rtabmap::Signature createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
									   const sensor_msgs::PointCloud2& scan3dMsg,
									   const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
									   const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
									   const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs);

	void addSignatureToOccupancyGrid(const rtabmap::Signature& signature);
	void publishOccupancyGridMaps(double stamp, const std::string& frame_id);
	nav_msgs::OccupancyGrid getOccupancyGridMap();

private:
	ros::Publisher occupancyGridPub_;
	ros::Publisher coloredOccupancyGridPub_;
	ros::Subscriber optimizedPosesSub_;

	tf::TransformListener tfListener_;

	int nodeId_;
	rtabmap::OccupancyGrid occupancyGrid_;

	std::map<int, rtabmap::Transform> poses_;
	std::map<int, ros::Time> times_;

	ros::Time lastOptimizedPoseTime_;
	std::map<int, tf2_ros::Buffer> optimizedPosesBuffers_;

	UMutex mutex_;

	std::string mapPath_;
	bool loadMap_;
	bool saveMap_;

	bool needsLocalization_;
	bool cacheLoadedMap_;

	std::string mapFrame_;
	std::string baseLinkFrame_;
};

}
