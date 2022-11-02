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

// RGB
void CommonDataSubscriber::rgb2Callback(
		const sensor_msgs::ImageConstPtr& imageMsg1,
		const sensor_msgs::ImageConstPtr& imageMsg2,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg1), cv_bridge::toCvShare(imageMsg2)},
		{*cameraInfoMsg1, *cameraInfoMsg2}, scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgb2Scan2dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg1,
		const sensor_msgs::ImageConstPtr& imageMsg2,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg1), cv_bridge::toCvShare(imageMsg2)},
		{*cameraInfoMsg1, *cameraInfoMsg2}, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgb2Scan3dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg1,
		const sensor_msgs::ImageConstPtr& imageMsg2,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg1), cv_bridge::toCvShare(imageMsg2)},
		{*cameraInfoMsg1, *cameraInfoMsg2}, scanMsg, *scan3dMsg);
}

// RGB + Odom
void CommonDataSubscriber::rgb2OdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg1,
		const sensor_msgs::ImageConstPtr& imageMsg2,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2)
{
	callbackCalled();
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg1), cv_bridge::toCvShare(imageMsg2)},
		{*cameraInfoMsg1, *cameraInfoMsg2}, scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgb2OdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg1,
		const sensor_msgs::ImageConstPtr& imageMsg2,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	callbackCalled();
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg1), cv_bridge::toCvShare(imageMsg2)},
		{*cameraInfoMsg1, *cameraInfoMsg2}, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgb2OdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg1,
		const sensor_msgs::ImageConstPtr& imageMsg2,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg1,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg2,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	callbackCalled();
	sensor_msgs::LaserScan scanMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg1), cv_bridge::toCvShare(imageMsg2)},
		{*cameraInfoMsg1, *cameraInfoMsg2}, scanMsg, *scan3dMsg);
}

void CommonDataSubscriber::setupRGB2Callback(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeScan2d,
		bool subscribeScan3d,
		int queueSize,
		bool approxSync)
{
	ROS_INFO("Setup rgb2 callback");

	std::string rgb1Prefix = "rgb_1";
	std::string rgb2Prefix = "rgb_2";
	ros::NodeHandle rgb1_nh(nh, rgb1Prefix);
	ros::NodeHandle rgb2_nh(nh, rgb2Prefix);
	ros::NodeHandle rgb1_pnh(pnh, rgb1Prefix);
	ros::NodeHandle rgb2_pnh(pnh, rgb2Prefix);
	image_transport::ImageTransport rgb1_it(rgb1_nh);
	image_transport::ImageTransport rgb2_it(rgb2_nh);
	image_transport::TransportHints hintsRgb1("raw", ros::TransportHints(), rgb1_pnh);
	image_transport::TransportHints hintsRgb2("raw", ros::TransportHints(), rgb2_pnh);

	imageSubs_.resize(2);
	imageSubs_[0] = std::make_unique<image_transport::SubscriberFilter>();
	imageSubs_[1] = std::make_unique<image_transport::SubscriberFilter>();
	cameraInfoSubs_.resize(2);
	cameraInfoSubs_[0] = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>();
	cameraInfoSubs_[1] = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>();
	imageSubs_[0]->subscribe(rgb1_it, rgb1_nh.resolveName("image"), queueSize, hintsRgb1);
	imageSubs_[1]->subscribe(rgb2_it, rgb2_nh.resolveName("image"), queueSize, hintsRgb2);
	cameraInfoSubs_[0]->subscribe(rgb1_nh, "camera_info", queueSize);
	cameraInfoSubs_[1]->subscribe(rgb2_nh, "camera_info", queueSize);
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
			SYNC_DECL6(CommonDataSubscriber, rgb2OdomScan2d, approxSync, queueSize, odomSub_, (*imageSubs_[0]), (*imageSubs_[1]), (*cameraInfoSubs_[0]), (*cameraInfoSubs_[1]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			SYNC_DECL6(CommonDataSubscriber, rgb2OdomScan3d, approxSync, queueSize, odomSub_, (*imageSubs_[0]), (*imageSubs_[1]), (*cameraInfoSubs_[0]), (*cameraInfoSubs_[1]), scan3dSub_);
		}
		else
		{
			SYNC_DECL5(CommonDataSubscriber, rgb2Odom, approxSync, queueSize, odomSub_, (*imageSubs_[0]), (*imageSubs_[1]), (*cameraInfoSubs_[0]), (*cameraInfoSubs_[1]));
		}
	}
	else
	{
		if(subscribeScan2d)
		{
			SYNC_DECL5(CommonDataSubscriber, rgb2Scan2d, approxSync, queueSize, (*imageSubs_[0]), (*imageSubs_[1]), (*cameraInfoSubs_[0]), (*cameraInfoSubs_[1]), scanSub_);
		}
		else if(subscribeScan3d)
		{
			SYNC_DECL5(CommonDataSubscriber, rgb2Scan3d, approxSync, queueSize, (*imageSubs_[0]), (*imageSubs_[1]), (*cameraInfoSubs_[0]), (*cameraInfoSubs_[1]), scan3dSub_);
		}
		else
		{
			SYNC_DECL4(CommonDataSubscriber, rgb2, approxSync, queueSize, (*imageSubs_[0]), (*imageSubs_[1]), (*cameraInfoSubs_[0]), (*cameraInfoSubs_[1]));
		}
	}
}

} /* namespace rtabmap_ros */
