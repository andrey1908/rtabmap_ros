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

#include <rtabmap_ros/CommonDataSubscriber.h>

namespace rtabmap_ros {

// RGB + Depth
void CommonDataSubscriber::depthCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& depthCamInfoMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonDepthCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {cv_bridge::toCvShare(depthMsg)}, {*cameraInfoMsg}, {*depthCamInfoMsg}, scanMsg, scan3dMsg);
}
void CommonDataSubscriber::depthScan2dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& depthCamInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonDepthCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {cv_bridge::toCvShare(depthMsg)}, {*cameraInfoMsg}, {*depthCamInfoMsg}, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::depthScan3dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& depthCamInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	commonDepthCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {cv_bridge::toCvShare(depthMsg)}, {*cameraInfoMsg}, {*depthCamInfoMsg}, scanMsg, *scan3dMsg);
}

// RGB + Depth + Odom
void CommonDataSubscriber::depthOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& depthCamInfoMsg)
{
	callbackCalled();
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	commonDepthCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {cv_bridge::toCvShare(depthMsg)}, {*cameraInfoMsg}, {*depthCamInfoMsg}, scanMsg, scan3dMsg);
}
void CommonDataSubscriber::depthOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& depthCamInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	callbackCalled();
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonDepthCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {cv_bridge::toCvShare(depthMsg)}, {*cameraInfoMsg}, {*depthCamInfoMsg}, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::depthOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::ImageConstPtr& depthMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::CameraInfoConstPtr& depthCamInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	callbackCalled();
	sensor_msgs::LaserScan scanMsg; // Null
	commonDepthCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {cv_bridge::toCvShare(depthMsg)}, {*cameraInfoMsg}, {*depthCamInfoMsg}, scanMsg, *scan3dMsg);
}

void CommonDataSubscriber::setupDepthCallback(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeScan2d,
		bool subscribeScan3d,
		int queueSize,
		bool approxSync)
{
	ROS_INFO("Setup depth callback");

	std::string rgbPrefix = "rgb";
	std::string depthPrefix = "depth";
	ros::NodeHandle rgb_nh(nh, rgbPrefix);
	ros::NodeHandle depth_nh(nh, depthPrefix);
	ros::NodeHandle rgb_pnh(pnh, rgbPrefix);
	ros::NodeHandle depth_pnh(pnh, depthPrefix);
	image_transport::ImageTransport rgb_it(rgb_nh);
	image_transport::ImageTransport depth_it(depth_nh);
	image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
	image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

	imageSubs_.resize(1);
	imageSubs_[0] = std::make_unique<image_transport::SubscriberFilter>();
	depthSubs_.resize(1);
	depthSubs_[0] = std::make_unique<image_transport::SubscriberFilter>();
	cameraInfoSubs_.resize(1);
	cameraInfoSubs_[0] = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>();
	depthCamInfoSubs_.resize(1);
	depthCamInfoSubs_[0] = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>();
	imageSubs_[0]->subscribe(rgb_it, rgb_nh.resolveName("image"), queueSize, hintsRgb);
	depthSubs_[0]->subscribe(depth_it, depth_nh.resolveName("image"), queueSize, hintsDepth);
	cameraInfoSubs_[0]->subscribe(rgb_nh, "camera_info", queueSize);
	depthCamInfoSubs_[0]->subscribe(depth_nh, "camera_info", queueSize);
	if(subscribeOdom)
	{
		odomSub_.subscribe(nh, "odom", queueSize);
	}
	if(subscribeScan2d)
	{
		subscribedToScan2d_ = true;
		scanSub_.subscribe(nh, "scan", queueSize);
	}
	else if(subscribeScan3d)
	{
		subscribedToScan3d_ = true;
		scan3dSub_.subscribe(nh, "scan_cloud", queueSize);
	}

	if(subscribeOdom)
	{
		if(subscribeScan2d)
		{
			SYNC_DECL6(CommonDataSubscriber, depthOdomScan2d, approxSync, queueSize, odomSub_, (*imageSubs_[0]), (*depthSubs_[0]), (*cameraInfoSubs_[0]), (*depthCamInfoSubs_[0]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			SYNC_DECL6(CommonDataSubscriber, depthOdomScan3d, approxSync, queueSize, odomSub_, (*imageSubs_[0]), (*depthSubs_[0]), (*cameraInfoSubs_[0]), (*depthCamInfoSubs_[0]), scan3dSub_);
		}
		else
		{
			SYNC_DECL5(CommonDataSubscriber, depthOdom, approxSync, queueSize, odomSub_, (*imageSubs_[0]), (*depthSubs_[0]), (*cameraInfoSubs_[0]), (*depthCamInfoSubs_[0]));
		}
	}
	else
	{
		if(subscribeScan2d)
		{
			SYNC_DECL5(CommonDataSubscriber, depthScan2d, approxSync, queueSize, (*imageSubs_[0]), (*depthSubs_[0]), (*cameraInfoSubs_[0]), (*depthCamInfoSubs_[0]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			SYNC_DECL5(CommonDataSubscriber, depthScan3d, approxSync, queueSize, (*imageSubs_[0]), (*depthSubs_[0]), (*cameraInfoSubs_[0]), (*depthCamInfoSubs_[0]), scan3dSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, depth, approxSync, queueSize, (*imageSubs_[0]), (*depthSubs_[0]), (*cameraInfoSubs_[0]), (*depthCamInfoSubs_[0]));
		}
	}
}

} /* namespace rtabmap_ros */
