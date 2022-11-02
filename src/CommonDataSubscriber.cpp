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

CommonDataSubscriber::CommonDataSubscriber() :
		queueSize_(10),
		approxSync_(true),
		warningThread_(0),
		callbackSetup_(false),
		callbackCalled_(false),
		subscribedToOdom_(false),
		subscribedToScan2d_(false),
		subscribedToScan3d_(false),
		subscribedToRGB_(false),
		subscribedToDepth_(false),
		subscribedToStereo_(false),

		// Scan
		SYNC_INIT(odomScan2d),
		SYNC_INIT(odomScan3d),

		// RGB
		SYNC_INIT(rgb),
		SYNC_INIT(rgbScan2d),
		SYNC_INIT(rgbScan3d),
		SYNC_INIT(rgbOdom),
		SYNC_INIT(rgbOdomScan2d),
		SYNC_INIT(rgbOdomScan3d),

		// RGB2
		SYNC_INIT(rgb2),
		SYNC_INIT(rgb2Scan2d),
		SYNC_INIT(rgb2Scan3d),
		SYNC_INIT(rgb2Odom),
		SYNC_INIT(rgb2OdomScan2d),
		SYNC_INIT(rgb2OdomScan3d),

		// Depth
		SYNC_INIT(depth),
		SYNC_INIT(depthScan2d),
		SYNC_INIT(depthScan3d),
		SYNC_INIT(depthOdom),
		SYNC_INIT(depthOdomScan2d),
		SYNC_INIT(depthOdomScan3d),

		// Stereo
		SYNC_INIT(stereo),
		SYNC_INIT(stereoOdom)
{
}

void CommonDataSubscriber::setCommonOdomCallback(CommonOdomCallback commonOdomCallback)
{
	if (callbackSetup_)
	{
		ROS_ERROR("Cannot set callback after subscribing");
		return;
	}
	commonOdomCallback_ = std::move(commonOdomCallback);
}
void CommonDataSubscriber::setCommonLaserScanCallback(CommonLaserScanCallback commonLaserScanCallback)
{
	if (callbackSetup_)
	{
		ROS_ERROR("Cannot set callback after subscribing");
		return;
	}
	commonLaserScanCallback_ = std::move(commonLaserScanCallback);
}
void CommonDataSubscriber::setCommonRGBCallback(CommonRGBCallback commonRGBCallback)
{
	if (callbackSetup_)
	{
		ROS_ERROR("Cannot set callback after subscribing");
		return;
	}
	commonRGBCallback_ = std::move(commonRGBCallback);
}
void CommonDataSubscriber::setCommonDepthCallback(CommonDepthCallback commonDepthCallback)
{
	if (callbackSetup_)
	{
		ROS_ERROR("Cannot set callback after subscribing");
		return;
	}
	commonDepthCallback_ = std::move(commonDepthCallback);
}
void CommonDataSubscriber::setCommonStereoCallback(CommonStereoCallback commonStereoCallback)
{
	if (callbackSetup_)
	{
		ROS_ERROR("Cannot set callback after subscribing");
		return;
	}
	commonStereoCallback_ = std::move(commonStereoCallback);
}

void CommonDataSubscriber::setupCallbacks(
		ros::NodeHandle & nh,
		const std::string & name)
{
	bool subscribeScan2d = false;
	bool subscribeScan3d = false;
	int rgbCameras_ = 1;
	name_ = name;

	// ROS related parameters (private)
	ros::NodeHandle pnh("~");
	pnh.param(name_ + "_odom",      subscribedToOdom_, subscribedToOdom_);
	pnh.param(name_ + "_scan",      subscribeScan2d, subscribeScan2d);
	pnh.param(name_ + "_scan_cloud", subscribeScan3d, subscribeScan3d);
	pnh.param(name_ + "_rgb",       subscribedToRGB_, subscribedToRGB_);
	pnh.param(name_ + "_depth",     subscribedToDepth_, subscribedToDepth_);
	pnh.param(name_ + "_stereo",    subscribedToStereo_, subscribedToStereo_);

	if(subscribedToDepth_ && subscribedToStereo_)
	{
		ROS_WARN("rtabmap: Parameters subscribe_depth and subscribe_stereo cannot be true at the same time. Parameter subscribe_depth is set to false.");
		subscribedToDepth_ = false;
	}
	if(subscribedToRGB_ && subscribedToStereo_)
	{
		ROS_WARN("rtabmap: Parameters subscribe_stereo and subscribe_rgb cannot be true at the same time. Parameter subscribe_rgb is set to false.");
		subscribedToRGB_ = false;
	}
	if(subscribedToRGB_ && subscribedToDepth_)
	{
		ROS_WARN("rtabmap: Parameters subscribe_rgb and subscribe_depth cannot be true at the same time. Parameter subscribe_rgb is set to false.");
		subscribedToRGB_ = false;
	}
	if(subscribeScan2d && subscribeScan3d)
	{
		ROS_WARN("rtabmap: Parameters subscribe_scan and subscribe_scan_cloud cannot be true at the same time. Parameter subscribe_scan is set to false.");
		subscribeScan2d = false;
	}

	pnh.param(name_ + "_rgb_cameras", rgbCameras_, rgbCameras_);
	if (rgbCameras_ > 2)
	{
		ROS_WARN("Cannot synchronize more than 2 rgb cameras. Parameter rgb_cameras is set to 2.");
		rgbCameras_ = 2;
	}
	pnh.param(name_ + "_queue_size", queueSize_, queueSize_);
	pnh.param(name_ + "_approx_sync", approxSync_, approxSync_);

	ROS_INFO("%s: odom = %s", name.c_str(), subscribedToOdom_?"true":"false");
	ROS_INFO("%s: scan = %s", name.c_str(), subscribeScan2d?"true":"false");
	ROS_INFO("%s: scan_cloud = %s", name.c_str(), subscribeScan3d?"true":"false");
	ROS_INFO("%s: rgb = %s (rgb_cameras %d)", name.c_str(), subscribedToRGB_?"true":"false", rgbCameras_);
	ROS_INFO("%s: depth = %s", name.c_str(), subscribedToDepth_?"true":"false");
	ROS_INFO("%s: stereo = %s", name.c_str(), subscribedToStereo_?"true":"false");
	ROS_INFO("%s: queue_size    = %d", name.c_str(), queueSize_);
	ROS_INFO("%s: approx_sync   = %s", name.c_str(), approxSync_?"true":"false");

	if(subscribedToStereo_)
	{
		setupStereoCallbacks(
			nh,
			pnh,
			subscribedToOdom_,
			queueSize_,
			approxSync_);
	}
	else if(subscribedToDepth_)
	{
		setupDepthCallbacks(
			nh,
			pnh,
			subscribedToOdom_,
			subscribeScan2d,
			subscribeScan3d,
			queueSize_,
			approxSync_);
	}
	else if(subscribedToRGB_)
	{
		if (rgbCameras_ == 1)
		{
			setupRGBCallbacks(
				nh,
				pnh,
				subscribedToOdom_,
				subscribeScan2d,
				subscribeScan3d,
				queueSize_,
				approxSync_);
		}
		else
		{
			setupRGB2Callbacks(
				nh,
				pnh,
				subscribedToOdom_,
				subscribeScan2d,
				subscribeScan3d,
				queueSize_,
				approxSync_);
		}
	}
	else if(subscribeScan2d || subscribeScan3d)
	{
		setupScanCallbacks(
			nh,
			pnh,
			subscribedToOdom_,
			subscribeScan2d,
			queueSize_,
			approxSync_);
	}
	else if(subscribedToOdom_)
	{
		setupOdomCallbacks(
			nh,
			pnh,
			queueSize_,
			approxSync_);
	}

	if(subscribedToOdom_ || subscribedToScan2d_ || subscribedToScan3d_ || subscribedToRGB_ || subscribedToDepth_ || subscribedToStereo_)
	{
		warningThread_ = new boost::thread(boost::bind(&CommonDataSubscriber::warningLoop, this));
		ROS_INFO("%s", subscribedTopicsMsg_.c_str());
	}

	callbackSetup_ = true;
}

CommonDataSubscriber::~CommonDataSubscriber()
{
	if(warningThread_)
	{
		callbackCalled();
		warningThread_->join();
		delete warningThread_;
	}

	// Scan
	SYNC_DEL(odomScan2d);
	SYNC_DEL(odomScan3d);

	// RGB
	SYNC_DEL(rgb);
	SYNC_DEL(rgbScan2d);
	SYNC_DEL(rgbScan3d);
	SYNC_DEL(rgbOdom);
	SYNC_DEL(rgbOdomScan2d);
	SYNC_DEL(rgbOdomScan3d);

	// RGB2
	SYNC_DEL(rgb2);
	SYNC_DEL(rgb2Scan2d);
	SYNC_DEL(rgb2Scan3d);
	SYNC_DEL(rgb2Odom);
	SYNC_DEL(rgb2OdomScan2d);
	SYNC_DEL(rgb2OdomScan3d);

	// Depth
	SYNC_DEL(depth);
	SYNC_DEL(depthScan2d);
	SYNC_DEL(depthScan3d);
	SYNC_DEL(depthOdom);
	SYNC_DEL(depthOdomScan2d);
	SYNC_DEL(depthOdomScan3d);

	// Stereo
	SYNC_DEL(stereo);
	SYNC_DEL(stereoOdom);

	//clear params
	if (!name_.empty())
	{
		ros::NodeHandle pnh("~");
		pnh.deleteParam(name_ + "_odom");
		pnh.deleteParam(name_ + "_scan");
		pnh.deleteParam(name_ + "_scan_cloud");
		pnh.deleteParam(name_ + "_rgb");
		pnh.deleteParam(name_ + "_depth");
		pnh.deleteParam(name_ + "_stereo");
		pnh.deleteParam(name_ + "_queue_size");
		pnh.deleteParam(name_ + "_approx_sync");
	}
}

void CommonDataSubscriber::warningLoop()
{
	ros::Duration r(5.0);
	while(!callbackCalled_)
	{
		r.sleep();
		if(!callbackCalled_)
		{
			ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
					"published (\"$ rostopic hz my_topic\") and the timestamps in their "
					"header are set. If topics are coming from different computers, make sure "
					"the clocks of the computers are synchronized (\"ntpdate\"). %s%s",
					name_.c_str(),
					approxSync_?
							uFormat("If topics are not published at the same rate, you could increase \"queue_size\" parameter (current=%d).", queueSize_).c_str():
							"Parameter \"approx_sync\" is false, which means that input topics should have all the exact timestamp for the callback to be called.",
					subscribedTopicsMsg_.c_str());
		}
	}
}

} /* namespace rtabmap_ros */
