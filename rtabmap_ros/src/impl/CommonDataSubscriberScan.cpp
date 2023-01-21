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

void CommonDataSubscriber::scan2dCallback(
        const sensor_msgs::LaserScanConstPtr& scanMsg)
{
    callbackCalled();
    nav_msgs::OdometryConstPtr odomMsg; // Null
    sensor_msgs::PointCloud2 scan3dMsg; // Null
    commonLaserScanCallback_(odomMsg, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::scan3dCallback(
        const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
    callbackCalled();
    nav_msgs::OdometryConstPtr odomMsg; // Null
    sensor_msgs::LaserScan scanMsg; // Null
    commonLaserScanCallback_(odomMsg, scanMsg, *scan3dMsg);
}
void CommonDataSubscriber::odomScan2dCallback(
        const nav_msgs::OdometryConstPtr & odomMsg,
        const sensor_msgs::LaserScanConstPtr& scanMsg)
{
    callbackCalled();
    sensor_msgs::PointCloud2 scan3dMsg; // Null
    commonLaserScanCallback_(odomMsg, *scanMsg, scan3dMsg);
}
void CommonDataSubscriber::odomScan3dCallback(
        const nav_msgs::OdometryConstPtr & odomMsg,
        const sensor_msgs::PointCloud2ConstPtr& scan3dMsg)
{
    callbackCalled();
    sensor_msgs::LaserScan scanMsg; // Null
    commonLaserScanCallback_(odomMsg, scanMsg, *scan3dMsg);
}

void CommonDataSubscriber::setupScanCallback(
        ros::NodeHandle & nh,
        ros::NodeHandle & pnh,
        bool subscribeOdom,
        bool scan2dTopic,
        int queueSize,
        bool approxSync)
{
    ROS_INFO("Setup scan callback");

    if(subscribeOdom)
    {
        odomSub_.subscribe(nh, "odom", queueSize);
        if(scan2dTopic)
        {
            subscribedToScan2d_ = true;
            scanSub_.subscribe(nh, "scan", queueSize);
            SYNC_DECL2(CommonDataSubscriber, odomScan2d, approxSync, queueSize, odomSub_, scanSub_);
        }
        else
        {
            subscribedToScan3d_ = true;
            scan3dSub_.subscribe(nh, "scan_cloud", queueSize);
            SYNC_DECL2(CommonDataSubscriber, odomScan3d, approxSync, queueSize, odomSub_, scan3dSub_);
        }
    }
    else
    {
        if(scan2dTopic)
        {
            subscribedToScan2d_ = true;
            scan2dSubOnly_ = nh.subscribe("scan", queueSize, &CommonDataSubscriber::scan2dCallback, this);
            subscribedTopicsMsg_ =
                    uFormat("\n%s subscribed to:\n   %s",
                    ros::this_node::getName().c_str(),
                    scan2dSubOnly_.getTopic().c_str());
        }
        else
        {
            subscribedToScan3d_ = true;
            scan3dSubOnly_ = nh.subscribe("scan_cloud", queueSize, &CommonDataSubscriber::scan3dCallback, this);
            subscribedTopicsMsg_ =
                    uFormat("\n%s subscribed to:\n   %s",
                    ros::this_node::getName().c_str(),
                    scan3dSubOnly_.getTopic().c_str());
        }
    }
}

} /* namespace rtabmap_ros */
