#include <rtabmap_ros/OccupancyGridMapWrapper.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/Time.h>
#include <rtabmap/core/Trajectory.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/PosesTrimmer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/core/util3d_transforms.h>

#include <rtabmap/proto/RawData.pb.h>

#include <rtabmap_ros/MsgConversion.h>

#include <kas_utils/time_measurer.h>
#include <kas_utils/yaml_utils.h>

#include <functional>
#include <random>
#include <boost/algorithm/string.hpp>

using kas_utils::mergeYaml;

namespace rtabmap_ros {

OccupancyGridMapWrapper::OccupancyGridMapWrapper(int argc, char** argv)
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string configPathsStr;
    pnh.param("config_paths", configPathsStr, std::string(""));
    UASSERT(!configPathsStr.empty());
    boost::split(configPaths_, configPathsStr, boost::is_any_of(","));
    UASSERT(configPaths_.size());

    YAML::Node config;
    for (const std::string& configPath : configPaths_)
    {
        YAML::Node updateConfig = YAML::LoadFile(configPath);
        bool ret = mergeYaml(config, updateConfig);
        UASSERT(ret);
    }

    readRosParameters(pnh, config);
    UASSERT(!updatedPosesFrame_.empty());
    UASSERT(accumulativeMapping_ || temporaryMapping_);

    UASSERT(config["OccupancyGridMap"]);
    rtabmap::OccupancyGridMap::Parameters parameters =
        rtabmap::OccupancyGridMap::Parameters::createParameters(config["OccupancyGridMap"]);
    occupancyGridMap_ = std::make_unique<rtabmap::OccupancyGridMap>(parameters);

    if (!loadMapPath_.empty())
    {
        occupancyGridMap_->load(loadMapPath_);
        if (needsLocalization_)
        {
            occupancyGridMap_->updatePoses(rtabmap::Trajectories(), std::nullopt);
        }
    }

    if (!saveRawDataPath_.empty())
    {
        rawDataWriter_ = std::make_unique<rtabmap::RawDataSerialization>(saveRawDataPath_);
    }

    optimizationResultsSub_ = nh.subscribe(
        "optimization_results", 1, &OccupancyGridMapWrapper::updatePoses, this);
    nodesToRemovePub_ = nh.advertise<slam_communication_msgs::NodesToRemove>(
        "nodes_to_remove", 1);
    if (accumulativeMapping_)
    {
        dataSubscriber_.setDataCallback(std::bind(
            &OccupancyGridMapWrapper::dataCallback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5, false));
        dataSubscriber_.setupCallback(nh, pnh, "accum");
    }
    if (temporaryMapping_)
    {
        temporaryDataSubscriber_.setDataCallback(std::bind(
            &OccupancyGridMapWrapper::dataCallback,
            this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5, true));
        temporaryDataSubscriber_.setupCallback(nh, pnh, "temp");
    }

    int numBuilders = occupancyGridMap_->numBuilders();
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
    trackedObjectsPub_ = pnh.advertise<visualization_msgs::MarkerArray>("tracked_objects", 1);
    sensorIgnoreAreasPub_ = pnh.advertise<visualization_msgs::MarkerArray>("sensor_ignore_areas", 1);
}

void OccupancyGridMapWrapper::readRosParameters(
    const ros::NodeHandle& pnh, const YAML::Node& config)
{
    updatedPosesFrame_ = config["updated_poses_frame"].as<std::string>("");
    pnh.param("load_map_path", loadMapPath_, std::string(""));
    pnh.param("save_map_path", saveMapPath_, std::string(""));
    pnh.param("save_tracking_results_path", saveTrackingResultsPath_, std::string(""));
    pnh.param("save_raw_data_path", saveRawDataPath_, std::string(""));
    needsLocalization_ = config["needs_localization"].as<bool>(true);
    accumulativeMapping_ = config["accumulative_mapping"].as<bool>(true);
    temporaryMapping_ = config["temporary_mapping"].as<bool>(false);
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

OccupancyGridMapWrapper::~OccupancyGridMapWrapper()
{
    if (!saveMapPath_.empty())
    {
        occupancyGridMap_->save(saveMapPath_);
    }
    if (!saveTrackingResultsPath_.empty() &&
        occupancyGridMap_->objectTrackingEnabled())
    {
        const std::list<rtabmap::ObjectTracking::MOT16TrackedObject>& mot16objects =
            occupancyGridMap_->mot16TrackedObjectsCache();
        std::ofstream output(saveTrackingResultsPath_);
        UASSERT(output.is_open());
        for (const auto& mot16object : mot16objects)
        {
            output << mot16object.toMOT16Entry() << std::endl;
        }
        output.close();
    }
    if (rawDataWriter_)
    {
        rawDataWriter_->close();
    }
}

void OccupancyGridMapWrapper::updatePoses(
    const slam_communication_msgs::OptimizationResults::ConstPtr& optimizationResultsMsg)
{
    MEASURE_BLOCK_TIME(updatePoses);
    rtabmap::Trajectories trajectories;
    for (const auto& trajectoryMsg : optimizationResultsMsg->trajectories)
    {
        UASSERT(trajectoryMsg.child_frame_id.empty() ||
            updatedPosesFrame_ == trajectoryMsg.child_frame_id);
        rtabmap::Trajectory trajectory;
        for (const auto& globalPoseMsg : trajectoryMsg.global_poses)
        {
            if (mapFrame_.empty())
            {
                mapFrame_ = globalPoseMsg.header.frame_id;
            }
            UASSERT(mapFrame_ == globalPoseMsg.header.frame_id);
            const ros::Time& stamp = globalPoseMsg.header.stamp;
            rtabmap::Time time(stamp.sec, stamp.nsec);
            rtabmap::Transform global_pose = transformFromPoseMsg(globalPoseMsg.pose);
            trajectory.addPose(time, global_pose);
        }
        trajectories.addTrajectory(std::move(trajectory));
    }

    std::optional<rtabmap::Transform> globalToLocal;
    const auto& mapToOdometryMsg = optimizationResultsMsg->map_to_odometry;
    if (mapToOdometryMsg.header.frame_id.size())
    {
        if (mapFrame_.empty())
        {
            mapFrame_ = mapToOdometryMsg.header.frame_id;
        }
        if (odomFrame_.empty())
        {
            odomFrame_ = mapToOdometryMsg.child_frame_id;
        }
        UASSERT(mapFrame_ == mapToOdometryMsg.header.frame_id);
        UASSERT(odomFrame_ == mapToOdometryMsg.child_frame_id);
        globalToLocal = transformFromGeometryMsg(mapToOdometryMsg.transform);
    }

    rtabmap::Time skipLocalMapsUpto(
        optimizationResultsMsg->skip_odometry_upto.sec,
        optimizationResultsMsg->skip_odometry_upto.nsec);

    if (occupancyGridMap_->posesTrimmerEnabled())
    {
        std::set<rtabmap::Time> posesToTrim =
            occupancyGridMap_->getPosesToTrim(trajectories);
        slam_communication_msgs::NodesToRemove nodesToRemoveMsg;
        nodesToRemoveMsg.nodes_to_remove.reserve(posesToTrim.size());
        for (const rtabmap::Time& poseToTrim : posesToTrim)
        {
            ros::Time time(poseToTrim.sec, poseToTrim.nsec);
            nodesToRemoveMsg.nodes_to_remove.push_back(time);
        }
        nodesToRemovePub_.publish(nodesToRemoveMsg);

        rtabmap::Trajectories trajectoriesTrimmed =
            rtabmap::PosesTrimmer::removePosesFromTrajectories(trajectories, posesToTrim);
        occupancyGridMap_->updatePoses(
            trajectoriesTrimmed, globalToLocal, skipLocalMapsUpto);
    }
    else
    {
        occupancyGridMap_->updatePoses(
            trajectories, globalToLocal, skipLocalMapsUpto);
    }

    if (rawDataWriter_)
    {
        MEASURE_BLOCK_TIME(writeRawData__optimizationResults);
        rtabmap::proto::RawData::UpdatePosesData updatePosesDataProto;
        *updatePosesDataProto.mutable_trajectories() = toProto(trajectories);
        if (globalToLocal)
        {
            *updatePosesDataProto.mutable_global_to_local() = toProto(*globalToLocal);
        }
        *updatePosesDataProto.mutable_skip_local_maps_upto() = toProto(skipLocalMapsUpto);

        rtabmap::proto::RawData proto;
        *proto.mutable_update_poses_data() = std::move(updatePosesDataProto);
        rawDataWriter_->write(proto);
    }
}

void OccupancyGridMapWrapper::dataCallback(
    const nav_msgs::Odometry& globalOdometryMsg,
    const nav_msgs::Odometry& localOdometryMsg,
    const sensor_msgs::PointCloud2& pointCloudMsg,
    const std::vector<sensor_msgs::CameraInfoConstPtr>& cameraInfoMsgs,
    const std::vector<sensor_msgs::ImageConstPtr>& imageMsgs,
    bool temporaryMapping)
{
    MEASURE_TIME_FROM_HERE(dataCallback);

    UASSERT(pointCloudMsg.data.size());

    if (mapFrame_.empty())
    {
        mapFrame_ = globalOdometryMsg.header.frame_id;
    }
    if (odomFrame_.empty())
    {
        odomFrame_ = localOdometryMsg.header.frame_id;
    }
    if (baseLinkFrame_.empty())
    {
        baseLinkFrame_ = localOdometryMsg.child_frame_id;
    }
    UASSERT(mapFrame_ == globalOdometryMsg.header.frame_id);
    UASSERT(odomFrame_ == localOdometryMsg.header.frame_id);
    UASSERT(globalOdometryMsg.child_frame_id ==
        localOdometryMsg.child_frame_id);
    UASSERT(baseLinkFrame_ == localOdometryMsg.child_frame_id);

    ros::Time stamp = localOdometryMsg.header.stamp;
    rtabmap::Time time(stamp.sec, stamp.nsec);
    if (!temporaryMapping)
    {
        bool localMapCanBeAdded = occupancyGridMap_->localMapCanBeAdded(time);
        if (!localMapCanBeAdded)
        {
            return;
        }
    }

    rtabmap::Transform localPose = transformFromPoseMsg(localOdometryMsg.pose.pose);
    rtabmap::Transform globalPose = transformFromPoseMsg(globalOdometryMsg.pose.pose);
    rtabmap::SensorData sensorData = createSensorData(
        pointCloudMsg,
        cameraInfoMsgs,
        imageMsgs);
    addSensorDataToOccupancyGrid(sensorData, time, localPose, globalPose, temporaryMapping);

    publishOccupancyGridMaps(stamp);
    if (imageMsgs.size())
    {
        publishLastDilatedSemantic(stamp, imageMsgs[0]->header.frame_id);
    }
    if (occupancyGridMap_->objectTrackingEnabled())
    {
        publishTrackedObjects(stamp, occupancyGridMap_->trackedObjects());
    }
    const std::vector<rtabmap::LocalMapBuilder::Area>& sensorIgnoreAreas =
        occupancyGridMap_->sensorIgnoreAreas();
    if (sensorIgnoreAreas.size())
    {
        publishSensorIgnoreAreas(stamp, pointCloudMsg.header.frame_id, sensorIgnoreAreas);
    }

    STOP_TIME_MEASUREMENT(dataCallback);
}

rtabmap::SensorData OccupancyGridMapWrapper::createSensorData(
    const sensor_msgs::PointCloud2& scan3dMsg,
    const std::vector<sensor_msgs::CameraInfoConstPtr>& cameraInfoMsgs,
    const std::vector<sensor_msgs::ImageConstPtr>& imageMsgs)
{
    rtabmap::LaserScan scan;
    if (scan3dMsg.data.size())
    {
        bool convertionOk = convertScan3dMsg(scan3dMsg, baseLinkFrame_, "",
            ros::Time(0), scan, tfListener_, 0.1);
        UASSERT(convertionOk);
    }

    std::vector<cv::Mat> rgbs;
    std::vector<rtabmap::CameraModel> cameraModels;
    if (imageMsgs.size())
    {
        bool convertionOk = convertRGBMsgs(imageMsgs, cameraInfoMsgs, baseLinkFrame_, "",
            ros::Time(0), rgbs, cameraModels, tfListener_, 0.1);
        UASSERT(convertionOk);
    }

    rtabmap::SensorData sensorData;
    sensorData.setLaserScan(std::move(scan));
    sensorData.setImages(cameraModels, rgbs);
    return sensorData;
}

void OccupancyGridMapWrapper::addSensorDataToOccupancyGrid(
    const rtabmap::SensorData& sensorData,
    const rtabmap::Time& time,
    const rtabmap::Transform& localPose, const rtabmap::Transform& globalPose,
    bool temporary)
{
    tf::StampedTransform fromUpdatedPoseTF;
    tfListener_.lookupTransform(updatedPosesFrame_, baseLinkFrame_,
        ros::Time(time.sec, time.nsec), fromUpdatedPoseTF);
    rtabmap::Transform fromUpdatedPose = rtabmap_ros::transformFromTF(fromUpdatedPoseTF);
    std::shared_ptr<rtabmap::LocalMap> localMap =
        occupancyGridMap_->createLocalMap(sensorData, time, fromUpdatedPose);
    if (temporary)
    {
        occupancyGridMap_->addTemporaryLocalMap(localPose, globalPose, localMap);
    }
    else
    {
        occupancyGridMap_->addLocalMap(localPose, globalPose, localMap);
    }

    if (rawDataWriter_)
    {
        MEASURE_BLOCK_TIME(writeRawData__inputData);
        rtabmap::proto::RawData::InputData inputDataProto;
        *inputDataProto.mutable_local_pose() = toProto(localPose);
        *inputDataProto.mutable_global_pose() = toProto(globalPose);
        *inputDataProto.mutable_sensor_data() = toProto(sensorData);
        *inputDataProto.mutable_time() = toProto(time);
        *inputDataProto.mutable_from_updated_pose() = toProto(fromUpdatedPose);
        inputDataProto.set_temporary(temporary);

        rtabmap::proto::RawData proto;
        *proto.mutable_input_data() = std::move(inputDataProto);
        rawDataWriter_->write(proto);
    }
}

void OccupancyGridMapWrapper::publishOccupancyGridMaps(const ros::Time& stamp)
{
    MEASURE_BLOCK_TIME(publishOccupancyGridMaps);

    std::vector<int> subscribedIndices;
    std::vector<int> coloredSubscribedIndices;
    std::map<int, nav_msgs::OccupancyGrid> occupancyGridMsgs;
    for (int i = 0; i < occupancyGridMap_->numBuilders(); i++)
    {
        bool occupancyGridSubscribed = (occupancyGridPubs_[i].getNumSubscribers() > 0);
        bool coloredOccupancyGridSubscribed = (coloredOccupancyGridPubs_[i].getNumSubscribers() > 0);
        if (occupancyGridSubscribed || coloredOccupancyGridSubscribed)
        {
            occupancyGridMsgs[i] = getOccupancyGridMsg(stamp, i);
            if (occupancyGridMsgs[i].header.frame_id.size())
            {
                if (occupancyGridSubscribed)
                {
                    subscribedIndices.push_back(i);
                }
                if (coloredOccupancyGridSubscribed)
                {
                    coloredSubscribedIndices.push_back(i);
                }
            }
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
    rtabmap::OccupancyGrid occupancyGrid = occupancyGridMap_->getOccupancyGrid(index);
    if (!occupancyGrid.limits.valid())
    {
        return nav_msgs::OccupancyGrid();
    }

    xMin = occupancyGrid.limits.minX() * occupancyGridMap_->cellSize();
    yMin = occupancyGrid.limits.minY() * occupancyGridMap_->cellSize();
    UASSERT(occupancyGrid.grid.size());

    nav_msgs::OccupancyGrid occupancyGridMsg;
    occupancyGridMsg.header.stamp = stamp;
    occupancyGridMsg.header.frame_id = mapFrame_;
    occupancyGridMsg.info.resolution = occupancyGridMap_->cellSize();
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
    rtabmap::ColorGrid colorGrid = occupancyGridMap_->getColorGrid(index);
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
    const cv::Mat& lastDilatedSemantic = occupancyGridMap_->lastDilatedSemantic();
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

void OccupancyGridMapWrapper::publishTrackedObjects(
    const ros::Time& stamp,
    const std::vector<rtabmap::ObjectTracking::TrackedObject>& trackedObjects)
{
    visualization_msgs::MarkerArray trackedObjectsArray;
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
        trackedObjectMarker.header.stamp = stamp;
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

void OccupancyGridMapWrapper::publishSensorIgnoreAreas(const ros::Time& stamp,
    const std::string& sensorFrame,
    const std::vector<rtabmap::LocalMapBuilder::Area>& sensorIgnoreAreas)
{
    visualization_msgs::MarkerArray sensorIgnoreAreasArray;
    visualization_msgs::Marker deleteAllMarkers;
    deleteAllMarkers.action = visualization_msgs::Marker::DELETEALL;
    sensorIgnoreAreasArray.markers.push_back(deleteAllMarkers);
    int i = 0;
    for (const rtabmap::LocalMapBuilder::Area& area : sensorIgnoreAreas)
    {
        visualization_msgs::Marker areaMarker;
        areaMarker.header.frame_id = sensorFrame;
        areaMarker.header.stamp = stamp;
        areaMarker.ns = "sensor_ignore_areas";
        areaMarker.id = i;
        areaMarker.type = visualization_msgs::Marker::LINE_LIST;
        areaMarker.action = visualization_msgs::Marker::ADD;

        rtabmap::Transform pose(
            area.x, area.y, area.z,
            area.roll, area.pitch, area.yaw);
        rtabmap_ros::transformToPoseMsg(pose, areaMarker.pose);

        areaMarker.points = createCube(area.length, area.width, area.height);

        areaMarker.scale.x = 0.03;
        if (area.transparent)
        {
            areaMarker.color.g = 1.0f;
        }
        else
        {
            areaMarker.color.r = 1.0f;
        }
        areaMarker.color.a = 1.0f;

        sensorIgnoreAreasArray.markers.push_back(areaMarker);
        i++;
    }
    sensorIgnoreAreasPub_.publish(sensorIgnoreAreasArray);
}

std::vector<geometry_msgs::Point> OccupancyGridMapWrapper::createCube(
    float length, float width, float height)
{
    float x = length / 2;
    float y = width / 2;
    float z = height / 2;

    std::vector<geometry_msgs::Point> vertices;
    vertices.reserve(8);
    for (int i = 0; i < 8; i++)
    {
        int xSign = (i % 2) * 2 - 1;
        int ySign = ((i / 2) % 2) * 2 - 1;
        int zSign = ((i / 4) % 2) * 2 - 1;

        geometry_msgs::Point vertex;
        vertex.x = xSign * x;
        vertex.y = ySign * y;
        vertex.z = zSign * z;
        vertices.push_back(vertex);
    }

    // vertices:
    // -x, -y, -z - 0
    // +x, -y, -z - 1
    // -x, +y, -z - 2
    // +x, +y, -z - 3
    // -x, -y, +z - 4
    // +x, -y, +z - 5
    // -x, +y, +z - 6
    // +x, +y, +z - 7

    // cude:
    // 0 - 1
    // 4 - 5
    // 0 - 4
    //
    // 1 - 3
    // 5 - 7
    // 1 - 5
    //
    // 2 - 0
    // 6 - 4
    // 2 - 6
    //
    // 3 - 2
    // 7 - 6
    // 3 - 7

    std::vector<geometry_msgs::Point> cube;
    for (int i = 0; i < 4; i++)
    {
        // some magic
        int j = (1 - i / 2) + (i % 2) * 2;

        cube.push_back(vertices[i]);
        cube.push_back(vertices[j]);

        cube.push_back(vertices[i + 4]);
        cube.push_back(vertices[j + 4]);

        cube.push_back(vertices[i]);
        cube.push_back(vertices[i + 4]);
    }

    return cube;
}

}
