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
void CommonDataSubscriber::rgbCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {*cameraInfoMsg}, scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgbScan2dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {*cameraInfoMsg}, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgbScan3dCallback(
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	callbackCalled();
	nav_msgs::OdometryConstPtr odomMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {*cameraInfoMsg}, scanMsg, *scan3dMsg);
}

// RGB + Odom
void CommonDataSubscriber::rgbOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg)
{
	callbackCalled();
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {*cameraInfoMsg}, scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgbOdomScan2dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::LaserScanConstPtr& scanMsg)
{
	callbackCalled();
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {*cameraInfoMsg}, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::rgbOdomScan3dCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const sensor_msgs::ImageConstPtr& imageMsg,
		const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg,
		const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
	callbackCalled();
	sensor_msgs::LaserScan scanMsg; // Null
	commonRGBCallback_(odomMsg, {cv_bridge::toCvShare(imageMsg)}, {*cameraInfoMsg}, scanMsg, *scan3dMsg);
}

void CommonDataSubscriber::setupRGBCallbacks(
		ros::NodeHandle & nh,
		ros::NodeHandle & pnh,
		bool subscribeOdom,
		bool subscribeScan2d,
		bool subscribeScan3d,
		int queueSize,
		bool approxSync)
{
	ROS_INFO("Setup rgb-only callback");

	std::string rgbPrefix = "rgb";
	ros::NodeHandle rgb_nh(nh, rgbPrefix);
	ros::NodeHandle rgb_pnh(pnh, rgbPrefix);
	image_transport::ImageTransport rgb_it(rgb_nh);
	image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);

	imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), queueSize, hintsRgb);
	cameraInfoSub_.subscribe(rgb_nh, "camera_info", queueSize);
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
			SYNC_DECL4(CommonDataSubscriber, rgbOdomScan2d, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_, scanSub_);
		}
		else if(subscribeScan3d)
		{
			SYNC_DECL4(CommonDataSubscriber, rgbOdomScan3d, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_, scan3dSub_);
		}
		else
		{
			SYNC_DECL3(CommonDataSubscriber, rgbOdom, approxSync, queueSize, odomSub_, imageSub_, cameraInfoSub_);
		}
	}
	else
	{
		if(subscribeScan2d)
		{
			SYNC_DECL3(CommonDataSubscriber, rgbScan2d, approxSync, queueSize, imageSub_, cameraInfoSub_, scanSub_);
		}
		else if(subscribeScan3d)
		{
			SYNC_DECL3(CommonDataSubscriber, rgbScan3d, approxSync, queueSize, imageSub_, cameraInfoSub_, scan3dSub_);
		}
		else
		{
			SYNC_DECL2(CommonDataSubscriber, rgb, approxSync, queueSize, imageSub_, cameraInfoSub_);
		}
	}
}

} /* namespace rtabmap_ros */
