#include "rtabmap_ros/OccupancyGridMapWrapper.h"

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Trajectory.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/util3d_transforms.h>

#include "rtabmap_ros/MsgConversion.h"

#include "time_measurer/time_measurer.h"

#include <functional>

namespace rtabmap_ros {

rtabmap::ParametersMap OccupancyGridMapWrapper::readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh)
{
    std::string configPath;
    pnh.param("config_path", configPath, configPath);

    // parameters
    rtabmap::ParametersMap parameters;
    uInsert(parameters, rtabmap::Parameters::getDefaultParameters());

    if(!configPath.empty())
    {
        if(UFile::exists(configPath.c_str()))
        {
            ROS_INFO("%s: Loading parameters from %s", ros::this_node::getName().c_str(), configPath.c_str());
            rtabmap::ParametersMap allParameters;
            rtabmap::Parameters::readINI(configPath.c_str(), allParameters);
            // only update odometry parameters
            for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
            {
                rtabmap::ParametersMap::iterator jter = allParameters.find(iter->first);
                if(jter!=allParameters.end())
                {
                    iter->second = jter->second;
                }
            }
        }
        else
        {
            ROS_ERROR("Config file \"%s\" not found!", configPath.c_str());
        }
    }

    for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
    {
        if (!pnh.hasParam(iter->first))
        {
            continue;
        }

        std::string vStr;
        bool vBool;
        double vDouble;
        int vInt;

        std::string sType = rtabmap::Parameters::getType(iter->first);
        if (sType == "string")
        {
            if (!pnh.getParam(iter->first, vStr))
            {
                ROS_FATAL("Could not get parameter %s. This parameter does not appear to be string type.", iter->first.c_str());
            }
            iter->second = vStr;
            ROS_INFO("Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), vStr.c_str());
        }
        else if (sType == "bool")
        {
            if (!pnh.getParam(iter->first, vBool))
            {
                ROS_FATAL("Could not get parameter %s. This parameter does not appear to be bool type.", iter->first.c_str());
            }
            iter->second = uBool2Str(vBool);
            ROS_INFO("Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.c_str());
        }
        else if (sType == "float" || sType == "double")
        {
            if (!pnh.getParam(iter->first, vDouble))
            {
                ROS_FATAL("Could not get parameter %s. This parameter does not appear to be double type.", iter->first.c_str());
            }
            iter->second = uNumber2Str(vDouble);
            ROS_INFO("Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.c_str());
        }
        else if (sType == "int")
        {
            if (!pnh.getParam(iter->first, vInt))
            {
                ROS_FATAL("Could not get parameter %s. This parameter does not appear to be int type.", iter->first.c_str());
            }
            iter->second = uNumber2Str(vInt);
            ROS_INFO("Setting %s parameter \"%s\"=\"%s\"", ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.c_str());
        }
        else
        {
            UASSERT_MSG(false, uFormat("Unknown parameter type '%s' in parameters list", sType.c_str()).c_str());
        }

        if(iter->first.compare(rtabmap::Parameters::kVisMinInliers()) == 0 && atoi(iter->second.c_str()) < 8)
        {
            ROS_WARN("Parameter min_inliers must be >= 8, setting to 8...");
            iter->second = uNumber2Str(8);
        }
    }

    rtabmap::ParametersMap argParameters = rtabmap::Parameters::parseArguments(argc, argv);
    for(rtabmap::ParametersMap::iterator iter=argParameters.begin(); iter!=argParameters.end(); ++iter)
    {
        rtabmap::ParametersMap::iterator jter = parameters.find(iter->first);
        if(jter!=parameters.end())
        {
            ROS_INFO("Update %s parameter \"%s\"=\"%s\" from arguments", ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.c_str());
            jter->second = iter->second;
        }
    }

    // Backward compatibility
    for(std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=rtabmap::Parameters::getRemovedParameters().begin();
        iter!=rtabmap::Parameters::getRemovedParameters().end();
        ++iter)
    {
        std::string vStr;
        if(pnh.getParam(iter->first, vStr))
        {
            if(iter->second.first && parameters.find(iter->second.second) != parameters.end())
            {
                // can be migrated
                parameters.at(iter->second.second) = vStr;
                ROS_WARN("%s: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
                        ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str(), vStr.c_str());
            }
            else
            {
                if(iter->second.second.empty())
                {
                    ROS_ERROR("%s: Parameter \"%s\" doesn't exist anymore!",
                            ros::this_node::getName().c_str(), iter->first.c_str());
                }
                else
                {
                    ROS_ERROR("%s: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
                            ros::this_node::getName().c_str(), iter->first.c_str(), iter->second.second.c_str());
                }
            }
        }
    }

    return parameters;
}

void OccupancyGridMapWrapper::readRtabmapRosParameters(const ros::NodeHandle& pnh)
{
    pnh.param("map_frame", mapFrame_, std::string(""));
    pnh.param("updated_poses_frame", updatedPosesFrame_, std::string(""));
    pnh.param("load_map_path", loadMapPath_, std::string(""));
    pnh.param("save_map_path", saveMapPath_, std::string(""));
    pnh.param("needs_localization", needsLocalization_, true);
    pnh.param("accumulative_mapping", accumulativeMapping_, true);
    pnh.param("temporary_mapping", temporaryMapping_, false);
    pnh.param("draw_door_center_estimation", drawDoorCenterEstimation_, false);
    pnh.param("draw_door_corners", drawDoorCorners_, false);
    pnh.param("door_tracking_small_radius", doorTrackingSmallRadius_, 0.3);
    pnh.param("door_tracking_large_radius", doorTrackingLargeRadius_, 0.7);
}

OccupancyGridMapWrapper::OccupancyGridMapWrapper(int argc, char** argv) :
        nodeId_(0),
        doorCenterInMapFrame_(std::numeric_limits<int>::min(), std::numeric_limits<int>::min())
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    rtabmap::ParametersMap parameters = readRtabmapParameters(argc, argv, pnh);
    readRtabmapRosParameters(pnh);
    UASSERT(!mapFrame_.empty());
    UASSERT(!updatedPosesFrame_.empty());
    UASSERT(accumulativeMapping_ || temporaryMapping_);

    timedOccupancyGridMap_ =
        std::make_unique<rtabmap::TimedOccupancyGridMap>(parameters);
    globalToOdometry_ = rtabmap::Transform::getIdentity();
    float cellSize = timedOccupancyGridMap_->cellSize();
    doorTracking_.initialize(std::lround(doorTrackingSmallRadius_ / cellSize),
        std::lround(doorTrackingLargeRadius_ / cellSize));

    occupancyGridPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
    coloredOccupancyGridPub_ = nh.advertise<colored_occupancy_grid_msgs::ColoredOccupancyGrid>("colored_grid_map", 1);
    dilatedSemanticPub_ = pnh.advertise<sensor_msgs::Image>("dilated_semantic_image", 1);
    doorCornersPub_ = pnh.advertise<rtabmap_ros_msgs::DoorCorners>("door_corners", 1);
    if (!loadMapPath_.empty())
    {
        nodeId_ = timedOccupancyGridMap_->load(loadMapPath_);
        if (needsLocalization_)
        {
            timedOccupancyGridMap_->updatePoses(
                rtabmap::Trajectories());
        }
    }
    if (accumulativeMapping_)
    {
        commonDataSubscriber_.setCommonRGBCallback(std::bind(&OccupancyGridMapWrapper::commonRGBCallback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5, false));
        commonDataSubscriber_.setCommonLaserScanCallback(std::bind(&OccupancyGridMapWrapper::commonLaserScanCallback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, false));
        commonDataSubscriber_.setupCallback(nh, "subscribe");
    }
    if (temporaryMapping_)
    {
        temporaryCommonDataSubscriber_.setCommonRGBCallback(std::bind(&OccupancyGridMapWrapper::commonRGBCallback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5, true));
        temporaryCommonDataSubscriber_.setCommonLaserScanCallback(std::bind(&OccupancyGridMapWrapper::commonLaserScanCallback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, true));
        temporaryCommonDataSubscriber_.setupCallback(nh, "temporary_subscribe");
    }
    optimizationResultsSub_ = nh.subscribe("optimization_results", 1, &OccupancyGridMapWrapper::updatePoses, this);
    doorCenterEstimationSub_ = pnh.subscribe("start_door_tracking", 1, &OccupancyGridMapWrapper::startDoorTracking, this);
    stopDoorTrackingSub_ = pnh.subscribe("stop_door_tracking", 1, &OccupancyGridMapWrapper::stopDoorTracking, this);
}

OccupancyGridMapWrapper::~OccupancyGridMapWrapper()
{
    if (!saveMapPath_.empty())
    {
        timedOccupancyGridMap_->save(saveMapPath_);
    }
}

void OccupancyGridMapWrapper::updatePoses(
        const optimization_results_msgs::OptimizationResults::ConstPtr& optimizationResults)
{
    UScopeMutex lock(mutex_);
    MEASURE_BLOCK_TIME(updatePoses);
    rtabmap::Trajectories trajectories;
    for (const auto& trajectory_msg : optimizationResults->trajectories)
    {
        rtabmap::Trajectory trajectory;
        for (const auto& global_pose_msg : trajectory_msg.global_poses)
        {
            UASSERT(global_pose_msg.header.frame_id == mapFrame_);
            const ros::Time& stamp = global_pose_msg.header.stamp;
            rtabmap::Time time(stamp.sec, stamp.nsec);
            rtabmap::Transform global_pose = transformFromPoseMsg(global_pose_msg.pose);
            trajectory.addPose(time, global_pose);
        }
        trajectories.addTrajectory(std::move(trajectory));
    }
    if (optimizationResults->current_trajectory_index != -1)
    {
        UASSERT(optimizationResults->current_global_to_odometry.header.frame_id ==
            mapFrame_);
        UASSERT(odomFrame_.empty() ||
            optimizationResults->current_global_to_odometry.child_frame_id ==
                odomFrame_);
        UASSERT(optimizationResults->current_child_frame_id == updatedPosesFrame_);
        globalToOdometry_ = transformFromGeometryMsg(
            optimizationResults->current_global_to_odometry.transform);
    }
    else
    {
        globalToOdometry_ = rtabmap::Transform::getIdentity();
    }
    timedOccupancyGridMap_->updatePoses(trajectories);
}

void OccupancyGridMapWrapper::commonLaserScanCallback(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const sensor_msgs::LaserScan& scanMsg,
        const sensor_msgs::PointCloud2& scan3dMsg,
        bool temporaryMapping)
{
    UScopeMutex lock(mutex_);
    MEASURE_BLOCK_TIME(commonLaserScanCallback);
    mappingPipeline(odomMsg, scan3dMsg, {}, {}, temporaryMapping);
}

void OccupancyGridMapWrapper::commonRGBCallback(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
        const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
        const sensor_msgs::LaserScan& scanMsg,
        const sensor_msgs::PointCloud2& scan3dMsg,
        bool temporaryMapping)
{
    UScopeMutex lock(mutex_);
    MEASURE_BLOCK_TIME(commonRGBCallback);
    mappingPipeline(odomMsg, scan3dMsg, imageMsgs, cameraInfoMsgs, temporaryMapping);
}

void OccupancyGridMapWrapper::mappingPipeline(
        const nav_msgs::OdometryConstPtr& odomMsg,
        const sensor_msgs::PointCloud2& scan3dMsg,
        const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
        const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
        bool temporaryMapping)
{
    UASSERT(odomMsg);
    UASSERT(scan3dMsg.data.size());
    UASSERT(odomFrame_.empty() ||
            (odomFrame_ == odomMsg->header.frame_id &&
                baseLinkFrame_ == odomMsg->child_frame_id));
    if (odomFrame_.empty())
    {
        odomFrame_ = odomMsg->header.frame_id;
        baseLinkFrame_ = odomMsg->child_frame_id;
    }

    rtabmap::Transform odometryPose = transformFromPoseMsg(odomMsg->pose.pose);
    rtabmap::Transform globalPose = globalToOdometry_ * odometryPose;
    rtabmap::Signature signature = createSignature(
        globalPose,
        odomMsg->header.stamp,
        scan3dMsg,
        imageMsgs,
        cameraInfoMsgs);
    addSignatureToOccupancyGrid(signature, temporaryMapping);
    if (doorCenterInMapFrame_.first != std::numeric_limits<int>::min())
    {
        trackDoor();
    }

    ros::Time stamp(signature.getSec(), signature.getNSec());
    publishOccupancyGridMaps(stamp);
    if (imageMsgs.size())
    {
        publishLastDilatedSemantic(stamp, imageMsgs[0]->header.frame_id);
    }
    tryToPublishDoorCorners(stamp);
}

rtabmap::Signature OccupancyGridMapWrapper::createSignature(
        const rtabmap::Transform& pose,
        const ros::Time& time,
        const sensor_msgs::PointCloud2& scan3dMsg,
        const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
        const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs)
{
    rtabmap::LaserScan scan;
    if (scan3dMsg.data.size())
    {
        bool convertionOk = convertScan3dMsg(scan3dMsg, baseLinkFrame_, "", ros::Time(0), scan, tfListener_, 0.0);
        UASSERT(convertionOk);
    }

    std::vector<cv::Mat> rgbs;
    std::vector<rtabmap::CameraModel> cameraModels;
    if (imageMsgs.size())
    {
        bool convertionOk = convertRGBMsgs(imageMsgs, cameraInfoMsgs, baseLinkFrame_, "", ros::Time(0),
            rgbs, cameraModels, tfListener_, 0.0);
        UASSERT(convertionOk);
    }

    rtabmap::SensorData data;
    data.setStamp(time.sec, time.nsec);
    data.setLaserScan(scan);
    data.setRGBImages(rgbs, cameraModels);
    rtabmap::Signature signature(data);
    signature.setPose(pose);
    return signature;
}

void OccupancyGridMapWrapper::addSignatureToOccupancyGrid(
        const rtabmap::Signature& signature, bool temporary /* false */)
{
    rtabmap::Time time(signature.getSec(), signature.getNSec());
    tf::StampedTransform fromUpdatedPoseTF;
    tfListener_.lookupTransform(updatedPosesFrame_, baseLinkFrame_,
        ros::Time(time.sec, time.nsec), fromUpdatedPoseTF);
    std::shared_ptr<const rtabmap::LocalMap> localMap =
        timedOccupancyGridMap_->createLocalMap(
            signature, time, rtabmap_ros::transformFromTF(fromUpdatedPoseTF));
    if (temporary)
    {
        timedOccupancyGridMap_->addTemporaryLocalMap(
            signature.getPose(), localMap);
    }
    else
    {
        timedOccupancyGridMap_->addLocalMap(
            nodeId_, signature.getPose(), localMap);
        nodeId_++;
    }
}

void OccupancyGridMapWrapper::publishOccupancyGridMaps(const ros::Time& stamp)
{
    MEASURE_BLOCK_TIME(publishOccupancyGridMaps);

    bool occupancyGridSubscribed = (occupancyGridPub_.getNumSubscribers() > 0);
    bool coloredOccupancyGridSubscribed = (coloredOccupancyGridPub_.getNumSubscribers() > 0);
    nav_msgs::OccupancyGrid occupancyGridMsg;
    if (occupancyGridSubscribed || coloredOccupancyGridSubscribed)
    {
        occupancyGridMsg = getOccupancyGridMsg(stamp);
    }

    if (occupancyGridSubscribed)
    {
        occupancyGridPub_.publish(occupancyGridMsg);
    }

    if (coloredOccupancyGridSubscribed)
    {
        colored_occupancy_grid_msgs::ColoredOccupancyGrid coloredOccupancyGridMsg;
        coloredOccupancyGridMsg.header = occupancyGridMsg.header;
        coloredOccupancyGridMsg.info = occupancyGridMsg.info;
        coloredOccupancyGridMsg.data = occupancyGridMsg.data;
        fillColorsInColoredOccupancyGridMsg(coloredOccupancyGridMsg);
        maybeDrawDoorOnColoredOccupancyGridMsg(coloredOccupancyGridMsg);
        coloredOccupancyGridPub_.publish(coloredOccupancyGridMsg);
    }
}

nav_msgs::OccupancyGrid OccupancyGridMapWrapper::getOccupancyGridMsg(const ros::Time& stamp)
{
    float xMin, yMin;
    rtabmap::OccupancyGrid occupancyGrid =
        timedOccupancyGridMap_->getOccupancyGrid();
    xMin = occupancyGrid.limits.minX() * timedOccupancyGridMap_->cellSize();
    yMin = occupancyGrid.limits.minY() * timedOccupancyGridMap_->cellSize();
    UASSERT(occupancyGrid.grid.size());

    nav_msgs::OccupancyGrid occupancyGridMsg;
    occupancyGridMsg.header.stamp = stamp;
    occupancyGridMsg.header.frame_id = mapFrame_;
    occupancyGridMsg.info.resolution = timedOccupancyGridMap_->cellSize();
    occupancyGridMsg.info.origin.position.x = 0.0;
    occupancyGridMsg.info.origin.position.y = 0.0;
    occupancyGridMsg.info.origin.position.z = 0.0;
    occupancyGridMsg.info.origin.orientation.x = 0.0;
    occupancyGridMsg.info.origin.orientation.y = 0.0;
    occupancyGridMsg.info.origin.orientation.z = 0.0;
    occupancyGridMsg.info.origin.orientation.w = 1.0;

    occupancyGridMsg.info.width = occupancyGrid.grid.cols();
    occupancyGridMsg.info.height = occupancyGrid.grid.rows();
    occupancyGridMsg.info.origin.position.x = xMin;
    occupancyGridMsg.info.origin.position.y = yMin;
    occupancyGridMsg.data.resize(occupancyGrid.grid.size());

    memcpy(occupancyGridMsg.data.data(), occupancyGrid.grid.data(), occupancyGrid.grid.size());
    return occupancyGridMsg;
}

void OccupancyGridMapWrapper::fillColorsInColoredOccupancyGridMsg(
        colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg)
{
    rtabmap::ColorGrid colorGrid = timedOccupancyGridMap_->getColorGrid();
    for (int h = 0; h < colorGrid.grid.rows(); h++)
    {
        for (int w = 0; w < colorGrid.grid.cols(); w++)
        {
            int color = colorGrid.grid(h, w);
            if (color == rtabmap::Color::missingColor.data())
            {
                color = 0;
            }
            coloredOccupancyGridMsg.b.push_back(color & 0xFF);
            coloredOccupancyGridMsg.g.push_back((color >> 8) & 0xFF);
            coloredOccupancyGridMsg.r.push_back((color >> 16) & 0xFF);
        }
    }
}

void OccupancyGridMapWrapper::maybeDrawDoorOnColoredOccupancyGridMsg(
        colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg)
{
    if (doorCenterInMapFrame_.first != std::numeric_limits<int>::min())
    {
        if (drawDoorCenterEstimation_)
        {
            drawDoorCenterUsedAsEstimationOnColoredOccupancyGrid(
                coloredOccupancyGridMsg, cv::Vec3b(255, 0, 0));
        }
        if (drawDoorCorners_)
        {
            drawDoorCornersOnColoredOccupancyGrid(
                coloredOccupancyGridMsg, cv::Vec3b(0, 0, 255));
        }
    }
}

void OccupancyGridMapWrapper::drawDoorCenterUsedAsEstimationOnColoredOccupancyGrid(
        colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg,
        const cv::Vec3b& color)
{
    if (doorCenterInMapFrame_.first == std::numeric_limits<int>::min())
    {
        return;
    }
    int width =  coloredOccupancyGridMsg.info.width;
    float cellSize = timedOccupancyGridMap_->cellSize();
    float originX, originY;
    std::tie(originX, originY) = timedOccupancyGridMap_->getGridOrigin();
    rtabmap::DoorTracking::Cell doorCenterEstimation;
    doorCenterEstimation.first = doorCenterUsedAsEstimationInMapFrame_.first -
        std::lround(originY / cellSize);
    doorCenterEstimation.second = doorCenterUsedAsEstimationInMapFrame_.second -
        std::lround(originX / cellSize);
    int index = doorCenterEstimation.second + doorCenterEstimation.first * width;
    if (index >= 0 && index < coloredOccupancyGridMsg.b.size())
    {
        coloredOccupancyGridMsg.b[index] = color[0];
        coloredOccupancyGridMsg.g[index] = color[1];
        coloredOccupancyGridMsg.r[index] = color[2];
    }
}

void OccupancyGridMapWrapper::drawDoorCornersOnColoredOccupancyGrid(
        colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg,
        const cv::Vec3b& color)
{
    if (doorCenterInMapFrame_.first == std::numeric_limits<int>::min())
    {
        return;
    }
    if (doorCornersInMapFrame_.first.first == -1)
    {
        return;
    }
    int width =  coloredOccupancyGridMsg.info.width;
    float cellSize = timedOccupancyGridMap_->cellSize();
    float originX, originY;
    std::tie(originX, originY) = timedOccupancyGridMap_->getGridOrigin();
    rtabmap::DoorTracking::Cell firstCorner, secondCorner;
    firstCorner.first = doorCornersInMapFrame_.first.first - std::lround(originY / cellSize);
    firstCorner.second = doorCornersInMapFrame_.first.second - std::lround(originX / cellSize);
    secondCorner.first = doorCornersInMapFrame_.second.first - std::lround(originY / cellSize);
    secondCorner.second = doorCornersInMapFrame_.second.second - std::lround(originX / cellSize);
    int firstIndex = firstCorner.second + firstCorner.first * width;
    int secondIndex = secondCorner.second + secondCorner.first * width;
    UASSERT(firstIndex >= 0 && firstIndex < coloredOccupancyGridMsg.b.size());
    UASSERT(secondIndex >= 0 && secondIndex < coloredOccupancyGridMsg.b.size());
    coloredOccupancyGridMsg.b[firstIndex] = color[0];
    coloredOccupancyGridMsg.g[firstIndex] = color[1];
    coloredOccupancyGridMsg.r[firstIndex] = color[2];
    coloredOccupancyGridMsg.b[secondIndex] = color[0];
    coloredOccupancyGridMsg.g[secondIndex] = color[1];
    coloredOccupancyGridMsg.r[secondIndex] = color[2];
}

void OccupancyGridMapWrapper::publishLastDilatedSemantic(const ros::Time& stamp, const std::string& frame_id)
{
    const cv::Mat& lastDilatedSemantic = timedOccupancyGridMap_->lastDilatedSemantic();
    if (lastDilatedSemantic.empty())
    {
        return;
    }
    std_msgs::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;
    cv_bridge::CvImage cv_bridge(header, sensor_msgs::image_encodings::BGR8, lastDilatedSemantic);
    sensor_msgs::Image lastDilatedSemanticMsg;
    cv_bridge.toImageMsg(lastDilatedSemanticMsg);
    dilatedSemanticPub_.publish(lastDilatedSemanticMsg);
}

void OccupancyGridMapWrapper::tryToPublishDoorCorners(const ros::Time& stamp)
{
    if (doorCenterInMapFrame_.first == std::numeric_limits<int>::min())
    {
        return;
    }
    rtabmap_ros_msgs::DoorCorners doorCornersMsg;
    doorCornersMsg.header.stamp = stamp;
    doorCornersMsg.header.frame_id = mapFrame_;
    if (doorCornersInMapFrame_.first.first == -1)
    {
        doorCornersMsg.success = false;
    }
    else
    {
        doorCornersMsg.success = true;
        float cellSize = timedOccupancyGridMap_->cellSize();
        doorCornersMsg.first_corner.x = doorCornersInMapFrame_.first.second * cellSize + cellSize / 2;
        doorCornersMsg.first_corner.y = doorCornersInMapFrame_.first.first * cellSize + cellSize / 2;
        doorCornersMsg.first_corner.z = 0;
        doorCornersMsg.second_corner.x = doorCornersInMapFrame_.second.second * cellSize + cellSize / 2;
        doorCornersMsg.second_corner.y = doorCornersInMapFrame_.second.first * cellSize + cellSize / 2;
        doorCornersMsg.second_corner.z = 0;
    }
    doorCornersPub_.publish(doorCornersMsg);
}

void OccupancyGridMapWrapper::startDoorTracking(const geometry_msgs::PointConstPtr& doorCenterEstimation)
{
    UScopeMutex lock(mutex_);
    doorCenterInMapFrame_.first = std::lround(doorCenterEstimation->y / timedOccupancyGridMap_->cellSize());
    doorCenterInMapFrame_.second = std::lround(doorCenterEstimation->x / timedOccupancyGridMap_->cellSize());
    doorCornersInMapFrame_.first.first = -1;
    doorCenterUsedAsEstimationInMapFrame_ = doorCenterInMapFrame_;
}

void OccupancyGridMapWrapper::trackDoor()
{
    UASSERT(doorCenterInMapFrame_.first != std::numeric_limits<int>::min());
    float originX, originY;
    rtabmap::OccupancyGrid occupancyGrid =
        timedOccupancyGridMap_->getOccupancyGrid();
    float cellSize = timedOccupancyGridMap_->cellSize();
    originX = occupancyGrid.limits.minX() * cellSize;
    originY = occupancyGrid.limits.minY() * cellSize;
    int width = occupancyGrid.grid.cols();
    int height = occupancyGrid.grid.rows();
    cv::Mat image(height, width, CV_8U, occupancyGrid.grid.data());
    rtabmap::DoorTracking::Cell doorCenterEstimation;
    doorCenterEstimation.first = doorCenterInMapFrame_.first - std::lround(originY / cellSize);
    doorCenterEstimation.second = doorCenterInMapFrame_.second - std::lround(originX / cellSize);
    doorCenterUsedAsEstimationInMapFrame_ = doorCenterInMapFrame_;
    if (doorCenterEstimation.first >= 0 && doorCenterEstimation.second >= 0 &&
        doorCenterEstimation.first < height && doorCenterEstimation.second < width)
    {
        rtabmap::DoorTracking::Cell firstDoorCorner, secondDoorCorner;
        std::tie(firstDoorCorner, secondDoorCorner) = doorTracking_.trackDoor(image, doorCenterEstimation);
        if (firstDoorCorner.first != -1)
        {
            doorCenterEstimation.first = (firstDoorCorner.first + secondDoorCorner.first) / 2;
            doorCenterEstimation.second = (firstDoorCorner.second + secondDoorCorner.second) / 2;
            doorCenterInMapFrame_.first = doorCenterEstimation.first + std::lround(originY / cellSize);
            doorCenterInMapFrame_.second = doorCenterEstimation.second + std::lround(originX / cellSize);
            doorCornersInMapFrame_.first.first = firstDoorCorner.first + std::lround(originY / cellSize);
            doorCornersInMapFrame_.first.second = firstDoorCorner.second + std::lround(originX / cellSize);
            doorCornersInMapFrame_.second.first = secondDoorCorner.first + std::lround(originY / cellSize);
            doorCornersInMapFrame_.second.second = secondDoorCorner.second + std::lround(originX / cellSize);
        }
        else
        {
            doorCornersInMapFrame_.first = rtabmap::DoorTracking::Cell(-1, -1);
            doorCornersInMapFrame_.second = rtabmap::DoorTracking::Cell(-1, -1);
        }
    }
    else
    {
        doorCenterInMapFrame_.first = std::numeric_limits<int>::min();
        doorCenterInMapFrame_.second = std::numeric_limits<int>::min();
    }
}

void OccupancyGridMapWrapper::stopDoorTracking(const std_msgs::EmptyConstPtr& empty)
{
    UScopeMutex lock(mutex_);
    doorCenterInMapFrame_.first = std::numeric_limits<int>::min();
    doorCenterInMapFrame_.second = std::numeric_limits<int>::min();
}

}
