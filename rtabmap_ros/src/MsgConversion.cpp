#include "rtabmap_ros/MsgConversion.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/Color.h>

#include <cstring>
#include <algorithm>

namespace rtabmap_ros {

rtabmap::Time fromRos(const ros::Time& rosTime)
{
    rtabmap::Time time(rosTime.sec, rosTime.nsec);
    return time;
}

ros::Time toRosTime(const rtabmap::Time& time)
{
    ros::Time rosTime(time.sec, time.nsec);
    return rosTime;
}

ros::Duration toRosDuration(const rtabmap::Time& duration)
{
    ros::Duration rosDuration(duration.sec, duration.nsec);
    return rosDuration;
}

rtabmap::Transform fromRos(const geometry_msgs::Pose& msg)
{
    UASSERT(
        msg.orientation.x != 0.0 ||
        msg.orientation.y != 0.0 ||
        msg.orientation.z != 0.0 ||
        msg.orientation.w != 0.0);
    rtabmap::Transform transform(
        msg.position.x,
        msg.position.y,
        msg.position.z,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    return transform;
}

rtabmap::Transform fromRos(const geometry_msgs::Transform& msg)
{
    UASSERT(
        msg.rotation.x != 0.0 ||
        msg.rotation.y != 0.0 ||
        msg.rotation.z != 0.0 ||
        msg.rotation.w != 0.0);
    rtabmap::Transform transform(
        msg.translation.x,
        msg.translation.y,
        msg.translation.z,
        msg.rotation.x,
        msg.rotation.y,
        msg.rotation.z,
        msg.rotation.w);
    return transform;
}

geometry_msgs::Pose toRosPose(const rtabmap::Transform& pose)
{
    UASSERT(!pose.isNull());
    Eigen::Quaterniond q = pose.getQuaterniond();
    geometry_msgs::Pose msg;
    msg.position.x = pose.x();
    msg.position.y = pose.y();
    msg.position.z = pose.z();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    return msg;
}

geometry_msgs::Transform toRosTransform(const rtabmap::Transform& transform)
{
    UASSERT(!transform.isNull());
    Eigen::Quaterniond q = transform.getQuaterniond();
    geometry_msgs::Transform msg;
    msg.translation.x = transform.x();
    msg.translation.y = transform.y();
    msg.translation.z = transform.z();
    msg.rotation.x = q.x();
    msg.rotation.y = q.y();
    msg.rotation.z = q.z();
    msg.rotation.w = q.w();
    return msg;
}

rtabmap::SensorData::CameraParameters fromRos(const sensor_msgs::CameraInfo& msg)
{
    UASSERT(std::all_of(msg.D.begin(), msg.D.end(), [](int d) { return d == 0; }));
    rtabmap::SensorData::CameraParameters parameters;
    parameters.width = msg.width;
    parameters.height = msg.height;
    parameters.fx = msg.K[0];
    parameters.fy = msg.K[4];
    parameters.cx = msg.K[2];
    parameters.cy = msg.K[5];
    return parameters;
}

rtabmap::SensorData::CameraData fromRos(
    const sensor_msgs::CameraInfo& infoMsg,
    const sensor_msgs::Image& msg,
    const tf2_ros::Buffer& tfBuffer,
    const std::string& baseFrame,
    const ros::Duration& timeout)
{
    geometry_msgs::TransformStamped toSensorMsg =
        tfBuffer.lookupTransform(baseFrame, msg.header.frame_id,
            msg.header.stamp, timeout);
    rtabmap::Transform toSensor = fromRos(toSensorMsg.transform);

    rtabmap::SensorData::CameraParameters parameters = fromRos(infoMsg);
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    rtabmap::SensorData::CameraData cameraData{std::move(toSensor), parameters, image};
    return cameraData;
}

rtabmap::SensorData::LidarData fromRos(
    const sensor_msgs::PointCloud2& msg,
    const tf2_ros::Buffer& tfBuffer,
    const std::string& baseFrame,
    const ros::Duration& timeout)
{
    geometry_msgs::TransformStamped toSensorMsg =
        tfBuffer.lookupTransform(baseFrame, msg.header.frame_id,
            msg.header.stamp, timeout);
    rtabmap::Transform toSensor = fromRos(toSensorMsg.transform);

    int offsetX = -1;
    int offsetY = -1;
    int offsetZ = -1;
    for (const sensor_msgs::PointField& field : msg.fields)
    {
        if (field.name == "x")
        {
            UASSERT(field.datatype == sensor_msgs::PointField::FLOAT32);
            UASSERT(field.count == 1);
            offsetX = field.offset;
            continue;
        }
        if (field.name == "y")
        {
            UASSERT(field.datatype == sensor_msgs::PointField::FLOAT32);
            UASSERT(field.count == 1);
            offsetY = field.offset;
            continue;
        }
        if (field.name == "z")
        {
            UASSERT(field.datatype == sensor_msgs::PointField::FLOAT32);
            UASSERT(field.count == 1);
            offsetZ = field.offset;
            continue;
        }
    }
    UASSERT(offsetX >= 0);
    UASSERT(offsetY >= 0);
    UASSERT(offsetZ >= 0);

    UASSERT(msg.data.size() % msg.point_step == 0);
    int pointsNum = msg.data.size() / msg.point_step;
    Eigen::Matrix3Xf points(3, pointsNum);
    const std::uint8_t* dataPtr = msg.data.data();
    for (int i = 0; i < pointsNum; i++)
    {
        points(0, i) = *(float*)(dataPtr + offsetX);
        points(1, i) = *(float*)(dataPtr + offsetY);
        points(2, i) = *(float*)(dataPtr + offsetZ);
        dataPtr += msg.point_step;
    }

    rtabmap::SensorData::LidarData lidarData{std::move(toSensor), std::move(points)};
    return lidarData;
}

nav_msgs::OccupancyGrid toRos(
    const rtabmap::OccupancyGrid& grid,
    const rtabmap::Time& time,
    const std::string& frameId,
    float cellSize)
{
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = toRosTime(time);
    msg.header.frame_id = frameId;
    msg.info.resolution = cellSize;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    if (!grid.limits.valid())
    {
        return msg;
    }

    UASSERT(grid.grid.size());
    msg.info.width = grid.grid.cols();
    msg.info.height = grid.grid.rows();
    msg.info.origin.position.x = grid.limits.min()[1] * cellSize;
    msg.info.origin.position.y = grid.limits.min()[0] * cellSize;
    msg.info.origin.position.z = 0.0;

    msg.data.resize(grid.grid.size());
    std::memcpy(msg.data.data(), grid.grid.data(), grid.grid.size());

    return msg;
}

colored_occupancy_grid_msgs::ColoredOccupancyGrid toRos(
    const rtabmap::OccupancyGrid& grid,
    const rtabmap::ColorGrid& colors,
    const rtabmap::Time& time,
    const std::string& frameId,
    float cellSize)
{
    UASSERT(grid.limits == colors.limits);

    colored_occupancy_grid_msgs::ColoredOccupancyGrid msg;
    msg.header.stamp = toRosTime(time);
    msg.header.frame_id = frameId;
    msg.info.resolution = cellSize;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    if (!grid.limits.valid())
    {
        return msg;
    }

    UASSERT(grid.grid.size());
    UASSERT(colors.grid.size());
    msg.info.width = grid.grid.cols();
    msg.info.height = grid.grid.rows();
    msg.info.origin.position.x = grid.limits.min()[1] * cellSize;
    msg.info.origin.position.y = grid.limits.min()[0] * cellSize;
    msg.info.origin.position.z = 0.0;

    msg.data.resize(grid.grid.size());
    std::memcpy(msg.data.data(), grid.grid.data(), grid.grid.size());

    msg.b.reserve(colors.grid.size());
    msg.g.reserve(colors.grid.size());
    msg.r.reserve(colors.grid.size());
    for (int h = 0; h < colors.grid.rows(); h++)
    {
        for (int w = 0; w < colors.grid.cols(); w++)
        {
            int color = colors.grid(h, w);
            if (color == rtabmap::Color::missingColor.data())
            {
                color = 0;
            }
            msg.b.push_back(color & 0xFF);
            msg.g.push_back((color >> 8) & 0xFF);
            msg.r.push_back((color >> 16) & 0xFF);
        }
    }

    return msg;
}

}
