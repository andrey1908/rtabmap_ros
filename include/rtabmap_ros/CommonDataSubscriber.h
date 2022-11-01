/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_
#define INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>

#include <rtabmap_ros/CommonDataSubscriberDefines.h>

#include <boost/thread.hpp>

#include <fstream>
#include <functional>

namespace rtabmap_ros {

class CommonDataSubscriber {
public:
	using CommonOdomCallback = std::function<void(
		const nav_msgs::OdometryConstPtr & odomMsg)>;
	using CommonLaserScanCallback = std::function<void(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::LaserScan& scanMsg,
		const sensor_msgs::PointCloud2& scan3dMsg)>;
	using CommonRGBCallback = std::function<void(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
		const sensor_msgs::LaserScan& scanMsg,
		const sensor_msgs::PointCloud2& scan3dMsg)>;
	using CommonDepthCallback = std::function<void(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const std::vector<cv_bridge::CvImageConstPtr> & imageMsgs,
		const std::vector<cv_bridge::CvImageConstPtr> & depthMsgs,
		const std::vector<sensor_msgs::CameraInfo> & cameraInfoMsgs,
		const std::vector<sensor_msgs::CameraInfo> & depthCamInfoMsgs,
		const sensor_msgs::LaserScan& scanMsg,
		const sensor_msgs::PointCloud2& scan3dMsg)>;
	using CommonStereoCallback = std::function<void(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const cv_bridge::CvImageConstPtr& leftImageMsg,
		const cv_bridge::CvImageConstPtr& rightImageMsg,
		const sensor_msgs::CameraInfo& leftCamInfoMsg,
		const sensor_msgs::CameraInfo& rightCamInfoMsg,
		const sensor_msgs::LaserScan& scanMsg,
		const sensor_msgs::PointCloud2& scan3dMsg)>;

public:
	CommonDataSubscriber();
	virtual ~CommonDataSubscriber();

	bool isSubscribedToOdom() const  {return subscribedToOdom_;}
	bool isSubscribedToScan2d() const {return subscribedToScan2d_;}
	bool isSubscribedToScan3d() const {return subscribedToScan3d_;}
	bool isSubscribedToRGB() const  {return subscribedToRGB_;}
	bool isSubscribedToDepth() const  {return subscribedToDepth_;}
	bool isSubscribedToStereo() const {return subscribedToStereo_;}
	bool isDataSubscribed() const {return isSubscribedToDepth() || isSubscribedToStereo() || isSubscribedToScan2d() || isSubscribedToScan3d() || isSubscribedToRGB() || isSubscribedToOdom();}
	int getQueueSize() const {return queueSize_;}
	bool isApproxSync() const {return approxSync_;}
	const std::string & name() const {return name_;}

	void setCommonOdomCallback(CommonOdomCallback commonOdomCallback);
	void setCommonLaserScanCallback(CommonLaserScanCallback commonLaserScanCallback);
	void setCommonRGBCallback(CommonRGBCallback commonRGBCallback);
	void setCommonDepthCallback(CommonDepthCallback commonDepthCallback);
	void setCommonStereoCallback(CommonStereoCallback commonStereoCallback);

	void setupCallbacks(
			ros::NodeHandle & nh,
			ros::NodeHandle & pnh,
			const std::string & name);

private:
	void warningLoop();
	void callbackCalled() {callbackCalled_ = true;}
	void setupOdomCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		int queueSize,
		bool approxSync);
	void setupScanCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool scan2dTopic,
		int queueSize,
		bool approxSync);
	void setupRGBCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeScan2d,
		bool subscribeScan3d,
		int queueSize,
		bool approxSync);
	void setupDepthCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeScan2d,
		bool subscribeScan3d,
		int queueSize,
		bool approxSync);
	void setupStereoCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		int queueSize,
		bool approxSync);

private:
	CommonOdomCallback commonOdomCallback_;
	CommonLaserScanCallback commonLaserScanCallback_;
	CommonRGBCallback commonRGBCallback_;
	CommonDepthCallback commonDepthCallback_;
	CommonStereoCallback commonStereoCallback_;

	std::string subscribedTopicsMsg_;
	int queueSize_;
	bool approxSync_;
	boost::thread* warningThread_;
	bool callbackSetup_;
	bool callbackCalled_;
	bool subscribedToOdom_;
	bool subscribedToScan2d_;
	bool subscribedToScan3d_;
	bool subscribedToRGB_;
	bool subscribedToDepth_;
	bool subscribedToStereo_;
	std::string name_;

	ros::Subscriber odomSubOnly_;
	ros::Subscriber scan2dSubOnly_;
	ros::Subscriber scan3dSubOnly_;

	// for rgb-only and depth callbacks
	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> depthCamInfoSub_;

	// stereo callback
	image_transport::SubscriberFilter imageRectLeft_;
	image_transport::SubscriberFilter imageRectRight_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoLeft_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoRight_;

	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> scanSub_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> scan3dSub_;

	// Odom
	void odomCallback(const nav_msgs::OdometryConstPtr&);

	// Scan
	void scan2dCallback(const sensor_msgs::LaserScanConstPtr&);
	void scan3dCallback(const sensor_msgs::PointCloud2ConstPtr&);

	// Scan + Odom
	DATA_SYNCS2(odomScan2d, nav_msgs::Odometry, sensor_msgs::LaserScan);
	DATA_SYNCS2(odomScan3d, nav_msgs::Odometry, sensor_msgs::PointCloud2);

	// RGB-only
	DATA_SYNCS2(rgb, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS3(rgbScan2d, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS3(rgbScan3d, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);

	// RGB-only + Odom
	DATA_SYNCS3(rgbOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo);
	DATA_SYNCS4(rgbOdomScan2d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS4(rgbOdomScan3d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);

	// RGB + Depth
	DATA_SYNCS4(depth, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);
	DATA_SYNCS5(depthScan2d, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS5(depthScan3d, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);

	// RGB + Depth + Odom
	DATA_SYNCS5(depthOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);
	DATA_SYNCS6(depthOdomScan2d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::LaserScan);
	DATA_SYNCS6(depthOdomScan3d, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2);

	// Stereo
	DATA_SYNCS4(stereo, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);

	// Stereo + Odom
	DATA_SYNCS5(stereoOdom, nav_msgs::Odometry, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo);
};

} /* namespace rtabmap_ros */

#endif /* INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBER_H_ */
