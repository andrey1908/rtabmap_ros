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
#include <random>

namespace rtabmap_ros {

void OccupancyGridMapWrapper::readRosParameters(
    const ros::NodeHandle& pnh, const YAML::Node& params)
{
    mapFrame_ = params["map_frame"].as<std::string>("");
    updatedPosesFrame_ = params["updated_poses_frame"].as<std::string>("");
    pnh.param("load_map_path", loadMapPath_, std::string(""));
    pnh.param("save_map_path", saveMapPath_, std::string(""));
    needsLocalization_ = params["needs_localization"].as<bool>(true);
    accumulativeMapping_ = params["accumulative_mapping"].as<bool>(true);
    temporaryMapping_ = params["temporary_mapping"].as<bool>(false);
}

std::string OccupancyGridMapWrapper::occupancyGridTopicPostfix(
    int index, int numBuilders)
{
    if (numBuilders == 1)
    {
        return "";
    }
    return "_" + std::to_string(index + 1);
}

OccupancyGridMapWrapper::OccupancyGridMapWrapper(int argc, char** argv) :
    nodeId_(0)
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("config_path", configPath_, std::string(""));
    UASSERT(!configPath_.empty());
    YAML::Node config = YAML::LoadFile(configPath_);
    readRosParameters(pnh, config);
    UASSERT(!mapFrame_.empty());
    UASSERT(!updatedPosesFrame_.empty());
    UASSERT(!configPath_.empty());
    UASSERT(accumulativeMapping_ || temporaryMapping_);

    UASSERT(config["TimedOccupancyGridMap"]);
    rtabmap::TimedOccupancyGridMap::Parameters parameters =
        rtabmap::TimedOccupancyGridMap::Parameters::createParameters(
            config["TimedOccupancyGridMap"]);
    timedOccupancyGridMap_ =
        std::make_unique<rtabmap::TimedOccupancyGridMap>(parameters);
    globalToOdometry_ = rtabmap::Transform::getIdentity();

    int numBuilders = timedOccupancyGridMap_->numBuilders();
    for (int i = 0; i < numBuilders; i++)
    {
        std::string postfix = occupancyGridTopicPostfix(i, numBuilders);
        occupancyGridPubs_.push_back(
            pnh.advertise<nav_msgs::OccupancyGrid>("grid_map" + postfix, 1));
        coloredOccupancyGridPubs_.push_back(
            pnh.advertise<colored_occupancy_grid_msgs::ColoredOccupancyGrid>(
                "colored_grid_map" + postfix, 1));
    }
    dilatedSemanticPub_ = pnh.advertise<sensor_msgs::Image>("dilated_semantic_image", 1);
    trackedObjectsPub_ = pnh.advertise<visualization_msgs::MarkerArray>("trackedObjects", 1);
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
        UASSERT(trajectory_msg.child_frame_id.empty() ||
            trajectory_msg.child_frame_id == updatedPosesFrame_);
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
    if (optimizationResults->global_to_odometry.header.frame_id.size())
    {
        UASSERT(optimizationResults->global_to_odometry.header.frame_id ==
            mapFrame_);
        UASSERT(odomFrame_.empty() ||
            optimizationResults->global_to_odometry.child_frame_id ==
                odomFrame_);
        globalToOdometry_ = transformFromGeometryMsg(
            optimizationResults->global_to_odometry.transform);
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

    ros::Time stamp(signature.getSec(), signature.getNSec());
    publishOccupancyGridMaps(stamp);
    if (imageMsgs.size())
    {
        publishLastDilatedSemantic(stamp, imageMsgs[0]->header.frame_id);
    }

    ////////////////
    visualization_msgs::MarkerArray trackedObjectsArray;
    const std::vector<rtabmap::ObjectTracking::TrackedObject>& trackedObjects =
        timedOccupancyGridMap_->trackedObjects();
    visualization_msgs::Marker deleteAllMarkers;
    deleteAllMarkers.action = visualization_msgs::Marker::DELETEALL;
    trackedObjectsArray.markers.push_back(deleteAllMarkers);
    for (const auto& trackedObject : trackedObjects)
    {
        if (trackedObject.trackedTimes < 3)
        {
            continue;
        }

        visualization_msgs::Marker trackedObjectMarker;
        trackedObjectMarker.header.frame_id = mapFrame_;
        trackedObjectMarker.header.stamp = odomMsg->header.stamp;
        trackedObjectMarker.ns = "tracked_objects";
        trackedObjectMarker.id = trackedObject.id;
        trackedObjectMarker.type = visualization_msgs::Marker::ARROW;
        trackedObjectMarker.action = visualization_msgs::Marker::ADD;

        rtabmap::ObjectTracking::Point position = trackedObject.position;
        rtabmap::ObjectTracking::Velocity velocity = trackedObject.velocity;
        Eigen::Vector3f baseVector(1, 0, 0);
        Eigen::Vector3f velocityVector;
        velocityVector.x() = velocity.vx;
        velocityVector.y() = velocity.vy;
        velocityVector.z() = 0.0f;
        Eigen::Quaternionf orientation;
        orientation.setFromTwoVectors(baseVector, velocityVector);
        rtabmap::Transform pose(
            position.x, position.y, 0.0f,
            orientation.x(), orientation.y(), orientation.z(), orientation.w());
        rtabmap_ros::transformToPoseMsg(pose, trackedObjectMarker.pose);

        double v = std::sqrt(velocity.vx * velocity.vx + velocity.vy * velocity.vy);
        trackedObjectMarker.scale.x = 1.0 * v;
        trackedObjectMarker.scale.y = 0.2;
        trackedObjectMarker.scale.z = 0.2;

        std::mt19937 colorGenerator;
        colorGenerator.seed(trackedObject.id);
        rtabmap::Color color(colorGenerator());
        trackedObjectMarker.color.r = color.r() / 255.0f;
        trackedObjectMarker.color.g = color.g() / 255.0f;
        trackedObjectMarker.color.b = color.b() / 255.0f;
        trackedObjectMarker.color.a = 1.0f;
        trackedObjectsArray.markers.push_back(trackedObjectMarker);
    }
    trackedObjectsPub_.publish(trackedObjectsArray);
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

    std::vector<int> subscribedIndices;
    std::vector<int> coloredSubscribedIndices;
    std::map<int, nav_msgs::OccupancyGrid> occupancyGridMsgs;
    for (int i = 0; i < timedOccupancyGridMap_->numBuilders(); i++)
    {
        bool occupancyGridSubscribed = (occupancyGridPubs_[i].getNumSubscribers() > 0);
        bool coloredOccupancyGridSubscribed = (coloredOccupancyGridPubs_[i].getNumSubscribers() > 0);
        if (occupancyGridSubscribed || coloredOccupancyGridSubscribed)
        {
            occupancyGridMsgs[i] = getOccupancyGridMsg(stamp, i);
        }
        if (occupancyGridSubscribed)
        {
            subscribedIndices.push_back(i);
        }
        if (coloredOccupancyGridSubscribed)
        {
            coloredSubscribedIndices.push_back(i);
        }
    }

    for (int index : subscribedIndices)
    {
        occupancyGridPubs_[index].publish(occupancyGridMsgs[index]);
    }

    for (int index : coloredSubscribedIndices)
    {
        colored_occupancy_grid_msgs::ColoredOccupancyGrid coloredOccupancyGridMsg;
        const nav_msgs::OccupancyGrid& occupancyGridMsg = occupancyGridMsgs[index];
        coloredOccupancyGridMsg.header = occupancyGridMsg.header;
        coloredOccupancyGridMsg.info = occupancyGridMsg.info;
        coloredOccupancyGridMsg.data = occupancyGridMsg.data;
        fillColorsInColoredOccupancyGridMsg(coloredOccupancyGridMsg, index);
        coloredOccupancyGridPubs_[index].publish(coloredOccupancyGridMsg);
    }
}

nav_msgs::OccupancyGrid OccupancyGridMapWrapper::getOccupancyGridMsg(
    const ros::Time& stamp, int index)
{
    float xMin, yMin;
    rtabmap::OccupancyGrid occupancyGrid =
        timedOccupancyGridMap_->getOccupancyGrid(index);
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
    colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg,
    int index)
{
    rtabmap::ColorGrid colorGrid = timedOccupancyGridMap_->getColorGrid(index);
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

void OccupancyGridMapWrapper::publishLastDilatedSemantic(
    const ros::Time& stamp, const std::string& frame_id)
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

}
