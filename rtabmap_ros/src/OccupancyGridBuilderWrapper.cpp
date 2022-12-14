#include "rtabmap_ros/OccupancyGridBuilderWrapper.h"

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>

#include "rtabmap_ros/MsgConversion.h"

#include "time_measurer/time_measurer.h"

#include <functional>

namespace rtabmap_ros {

rtabmap::ParametersMap OccupancyGridBuilder::readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh)
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

void OccupancyGridBuilder::readRtabmapRosParameters(const ros::NodeHandle& pnh)
{
	pnh.param("map_frame", mapFrame_, std::string(""));
	pnh.param("load_map_path", loadMapPath_, std::string(""));
	pnh.param("save_map_path", saveMapPath_, std::string(""));
	pnh.param("cache_map", cacheMap_, true);
	pnh.param("needs_localization", needsLocalization_, true);
	pnh.param("max_interpolation_time_error", maxInterpolationTimeError_, 0.06);
	pnh.param("update_max_extrapolation_time", updateMaxExtrapolationTime_, 1.0);
	pnh.param("accumulative_mapping", accumulativeMapping_, true);
	pnh.param("temporary_mapping", temporaryMapping_, false);
	pnh.param("draw_door_center_estimation", drawDoorCenterEstimation_, false);
	pnh.param("draw_door_corners", drawDoorCorners_, false);
	pnh.param("door_tracking_small_radius", doorTrackingSmallRadius_, 0.3);
	pnh.param("door_tracking_large_radius", doorTrackingLargeRadius_, 0.7);
}

OccupancyGridBuilder::OccupancyGridBuilder(int argc, char** argv) :
		nodeId_(0),
		doorCenterInMapFrame_(std::numeric_limits<int>::min(), std::numeric_limits<int>::min())
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	rtabmap::ParametersMap parameters = readRtabmapParameters(argc, argv, pnh);
	readRtabmapRosParameters(pnh);
	UASSERT(!mapFrame_.empty());
	UASSERT(accumulativeMapping_ || temporaryMapping_);

	occupancyGridBuilder_.parseParameters(parameters);
	float cellSize = occupancyGridBuilder_.cellSize();
	doorTracking_.initialize(std::lround(doorTrackingSmallRadius_ / cellSize),
		std::lround(doorTrackingLargeRadius_ / cellSize));

	occupancyGridPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
	coloredOccupancyGridPub_ = nh.advertise<colored_occupancy_grid_msgs::ColoredOccupancyGrid>("colored_grid_map", 1);
	dilatedSemanticPub_ = nh.advertise<sensor_msgs::Image>("dilated_semantic_image", 1);
	doorCornersPub_ = nh.advertise<rtabmap_ros_msgs::DoorCorners>("door_corners", 1);
	if (!loadMapPath_.empty())
	{
		load();
		if (needsLocalization_)
		{
			occupancyGridBuilder_.updatePoses({}, {});
		}
	}
	if (accumulativeMapping_)
	{
		commonDataSubscriber_.setCommonRGBCallback(std::bind(&OccupancyGridBuilder::commonRGBCallback,
			this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
			std::placeholders::_4, std::placeholders::_5, false));
		commonDataSubscriber_.setCommonLaserScanCallback(std::bind(&OccupancyGridBuilder::commonLaserScanCallback,
			this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, false));
		commonDataSubscriber_.setupCallback(nh, "subscribe");
	}
	if (temporaryMapping_)
	{
		temporaryCommonDataSubscriber_.setCommonRGBCallback(std::bind(&OccupancyGridBuilder::commonRGBCallback,
			this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
			std::placeholders::_4, std::placeholders::_5, true));
		temporaryCommonDataSubscriber_.setCommonLaserScanCallback(std::bind(&OccupancyGridBuilder::commonLaserScanCallback,
			this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, true));
		temporaryCommonDataSubscriber_.setupCallback(nh, "temporary_subscribe");
	}
	optimizationResultsSub_ = nh.subscribe("optimization_results", 1, &OccupancyGridBuilder::updatePoses, this);
	doorCenterEstimationSub_ = nh.subscribe("start_door_tracking", 1, &OccupancyGridBuilder::startDoorTracking, this);
	stopDoorTrackingSub_ = nh.subscribe("stop_door_tracking", 1, &OccupancyGridBuilder::stopDoorTracking, this);
}

OccupancyGridBuilder::~OccupancyGridBuilder()
{
	if (!saveMapPath_.empty())
	{
		save();
	}
}

void OccupancyGridBuilder::save()
{
	MEASURE_BLOCK_TIME(save);
	std::fstream fs(saveMapPath_, std::fstream::out | std::fstream::binary | std::fstream::trunc);
	UASSERT(fs.is_open());

	int version = latestMapVersion;
	fs.write((const char*)(&version), sizeof(version));

	float cellSize = occupancyGridBuilder_.cellSize();
	fs.write((const char*)(&cellSize), sizeof(cellSize));

	const auto& nodes = occupancyGridBuilder_.nodes();
	for (const auto& entry : nodes)
	{
		int nodeId = entry.first;
		const std::optional<rtabmap::Transform>& pose = entry.second.localPose;
		auto timeIt = times_.find(nodeId);
		const auto& localMap = entry.second.localMap;
		int numGround = 0;
		int numEmpty = localMap.numEmpty;
		int numObstacles = localMap.numObstacles;
		const Eigen::Matrix3Xf& points = localMap.points;
		const std::vector<rtabmap::LocalMapBuilder::Color>& colors = localMap.colors;
		float sensorBlindRange2dSqr = localMap.sensorBlindRange2dSqr;
		const rtabmap::Transform& toSensor = localMap.toSensor;
		const Eigen::Matrix4f& eigenToSensor = toSensor.toEigen4f();
		UASSERT(timeIt != times_.end());

		fs.write((const char*)(&nodeId), sizeof(nodeId));
		if (pose.has_value())
		{
			const Eigen::Matrix4f& eigenPose = pose->toEigen4f();
			fs.write((const char*)(eigenPose.data()), eigenPose.size() * sizeof(eigenPose(0, 0)));
		}
		else
		{
			const Eigen::Matrix4f& nullEigenMatrix = Eigen::Matrix4f::Zero();
			fs.write((const char*)(nullEigenMatrix.data()), nullEigenMatrix.size() * sizeof(nullEigenMatrix(0, 0)));
		}
		fs.write((const char*)(&timeIt->second.sec), sizeof(timeIt->second.sec));
		fs.write((const char*)(&timeIt->second.nsec), sizeof(timeIt->second.nsec));
		fs.write((const char*)(&numGround), sizeof(numGround));
		fs.write((const char*)(&numEmpty), sizeof(numEmpty));
		fs.write((const char*)(&numObstacles), sizeof(numObstacles));
		fs.write((const char*)(points.data()), points.size() * sizeof(points(0, 0)));
		fs.write((const char*)(colors.data()), colors.size() * sizeof(colors[0]));
		fs.write((const char*)(&sensorBlindRange2dSqr), sizeof(sensorBlindRange2dSqr));
		fs.write((const char*)(eigenToSensor.data()), eigenToSensor.size() * sizeof(eigenToSensor(0, 0)));
	}
	fs.close();
}

void OccupancyGridBuilder::load()
{
	MEASURE_BLOCK_TIME(load);
	std::fstream fs(loadMapPath_, std::fstream::in | std::fstream::binary | std::fstream::app);
	UASSERT(fs.is_open());

	UASSERT(fs.peek() != EOF);
	int version;
	fs.read((char*)(&version), sizeof(version));
	UASSERT(version == latestMapVersion || version == mapVersionWithoutSensorBlindRange);

	UASSERT(fs.peek() != EOF);
	float cellSize;
	fs.read((char*)(&cellSize), sizeof(cellSize));
	UASSERT(cellSize == occupancyGridBuilder_.cellSize());

	int maxNodeId = 0;
	while (fs.peek() != EOF)
	{
		int nodeId;
		Eigen::Matrix4f eigenPose;
		ros::Time time;
		int numGround;
		int numEmpty;
		int numObstacles;
		Eigen::Matrix3Xf points;
		std::vector<rtabmap::LocalMapBuilder::Color> colors;
		float sensorBlindRange2dSqr;
		Eigen::Matrix4f eigenToSensor;

		fs.read((char*)(&nodeId), sizeof(nodeId));
		fs.read((char*)(eigenPose.data()), eigenPose.size() * sizeof(eigenPose(0, 0)));
		fs.read((char*)(&time.sec), sizeof(time.sec));
		fs.read((char*)(&time.nsec), sizeof(time.nsec));
		fs.read((char*)(&numGround), sizeof(numGround));
		fs.read((char*)(&numEmpty), sizeof(numEmpty));
		fs.read((char*)(&numObstacles), sizeof(numObstacles));
		points.resize(3, numGround + numEmpty + numObstacles);
		fs.read((char*)(points.data()), points.size() * sizeof(points(0, 0)));
		colors.resize(numGround + numEmpty + numObstacles);
		fs.read((char*)(colors.data()), colors.size() * sizeof(colors[0]));
		if (version == latestMapVersion)
		{
			fs.read((char*)(&sensorBlindRange2dSqr), sizeof(sensorBlindRange2dSqr));
			fs.read((char*)(eigenToSensor.data()), eigenToSensor.size() * sizeof(eigenToSensor(0, 0)));
		}
		else if (version == mapVersionWithoutSensorBlindRange)
		{
			sensorBlindRange2dSqr = 0.0f;
		}

		if (nodeId > maxNodeId)
		{
			maxNodeId = nodeId;
		}
		times_[nodeId] = time;

		rtabmap::OccupancyGridBuilder::LocalMap localMap;
		localMap.numEmpty = numEmpty;
		localMap.numObstacles = numObstacles;
		localMap.points = std::move(points);
		localMap.colors = std::move(colors);
		localMap.sensorBlindRange2dSqr = sensorBlindRange2dSqr;
		localMap.toSensor = rtabmap::Transform::fromEigen4f(eigenToSensor);

		if (eigenPose != Eigen::Matrix4f::Zero())
		{
			rtabmap::Transform pose = rtabmap::Transform::fromEigen4f(eigenPose);
			occupancyGridBuilder_.addLocalMap(nodeId, pose, std::move(localMap));
		}
		else
		{
			occupancyGridBuilder_.addLocalMap(nodeId, std::move(localMap));
		}
	}
	nodeId_ = maxNodeId + 1;
	fs.close();
}

void OccupancyGridBuilder::updatePoses(
		const optimization_results_msgs::OptimizationResults::ConstPtr& optimizationResults)
{
	MEASURE_BLOCK_TIME(updatePoses);
	UScopeMutex lock(mutex_);
	lastOptimizedPoseTime_ = ros::Time();
	optimizedPosesTimes_.clear();
	trajectoryBuffers_.clear();
	odometryCorrection_.reset();

	ros::Time latestTrajectoryStartTime;
	for (const auto& trajectory : optimizationResults->trajectories)
	{
		trajectoryBuffers_.emplace_back(ros::Duration(1000000));
		tf2_ros::Buffer& trajectoryBuffer = *trajectoryBuffers_.rbegin();
		ros::Time trajectoryStartTime = trajectory.global_poses.begin()->header.stamp;
		for (const geometry_msgs::TransformStamped& pose : trajectory.global_poses)
		{
			UASSERT(mapFrame_ == pose.header.frame_id);
			trajectoryBuffer.setTransform(pose, "default");
			lastOptimizedPoseTime_ = std::max(lastOptimizedPoseTime_, pose.header.stamp);
			optimizedPosesTimes_.insert(pose.header.stamp);
			trajectoryStartTime = std::min(trajectoryStartTime, pose.header.stamp);
		}
		const std::string& trajectoryChildFrameId = trajectory.global_poses.begin()->child_frame_id;
		if (!baseLinkFrame_.empty() && trajectoryChildFrameId != baseLinkFrame_)
		{
			tf::StampedTransform toBaseLinkTF;
			tfListener_.lookupTransform(trajectoryChildFrameId, baseLinkFrame_, ros::Time(0), toBaseLinkTF);
			geometry_msgs::TransformStamped toBaseLink;
			transformStampedTFToMsg(toBaseLinkTF, toBaseLink);
			trajectoryBuffer.setTransform(toBaseLink, "default", true);
		}
		latestTrajectoryStartTime = std::max(latestTrajectoryStartTime, trajectoryStartTime);
	}

	if (!optimizationResults->map_to_odom.header.frame_id.empty())
	{
		UASSERT(mapFrame_ == optimizationResults->map_to_odom.header.frame_id &&
			(odomFrame_.empty() || odomFrame_ == optimizationResults->map_to_odom.child_frame_id));
		odometryCorrection_ = transformFromGeometryMsg(optimizationResults->map_to_odom.transform);
	}

	std::map<int, rtabmap::Transform> updatedPoses;
	for (const auto& idTime: times_)
	{
		int nodeId = idTime.first;
		ros::Time time = idTime.second;
		std::optional<rtabmap::Transform> pose;
		auto poseAfterLastUpdateIt = posesAfterLastUpdate_.find(nodeId);
		if (poseAfterLastUpdateIt != posesAfterLastUpdate_.end())
		{
			pose = getOptimizedPose(time, &(poseAfterLastUpdateIt->second),
				ros::Duration(updateMaxExtrapolationTime_));
		}
		else
		{
			pose = getOptimizedPose(time);
		}
		if (pose.has_value())
		{
			updatedPoses[nodeId] = *pose;
		}
	}
	UINFO("Dropped %d (%d -> %d) frames (reduced to %lf%)", (int)(times_.size() - updatedPoses.size()),
		(int)(times_.size()), (int)(updatedPoses.size()), 100.0 * updatedPoses.size() / times_.size());

	std::list<rtabmap::Transform> updatedTemporaryPoses;
	if (lastOptimizedPoseTime_ == ros::Time(0))
	{
		temporaryTimesPoses_.clear();
		occupancyGridBuilder_.resetTemporaryMap();
	}
	else
	{
		for (const auto& temporaryTimePose : temporaryTimesPoses_)
		{
			std::optional<rtabmap::Transform> pose =
				getOptimizedPose(temporaryTimePose.first, &(temporaryTimePose.second),
					ros::Duration(updateMaxExtrapolationTime_));
			UASSERT(pose.has_value());
			updatedTemporaryPoses.push_back(*pose);
		}
	}

	int lastNodeIdForCachedMap = -1;
	if (cacheMap_ && updatedPoses.size())
	{
		auto updatedPoseIt = updatedPoses.begin();
		while (times_.at(updatedPoseIt->first) < latestTrajectoryStartTime)
		{
			++updatedPoseIt;
			if (updatedPoseIt == updatedPoses.end())
			{
				break;
			}
		}
		if (updatedPoseIt != updatedPoses.begin())
		{
			lastNodeIdForCachedMap = std::prev(updatedPoseIt)->first;
		}
	}
	occupancyGridBuilder_.updatePoses(updatedPoses, updatedTemporaryPoses, lastNodeIdForCachedMap);
	posesAfterLastUpdate_.clear();
}

std::optional<rtabmap::Transform> OccupancyGridBuilder::getOptimizedPose(ros::Time time,
		const rtabmap::Transform* odometryPose /* nullptr */, ros::Duration maxExtrapolationTime /* 0 */,
		bool defaultIdentityOdometryCorrection /* false */)
{
	bool canInterpolate = (time <= lastOptimizedPoseTime_);
	if (canInterpolate && maxInterpolationTimeError_ != 0.0)
	{
		auto upperTimeIt = optimizedPosesTimes_.lower_bound(time);
		UASSERT(upperTimeIt != optimizedPosesTimes_.end());
		double interpolationTimeError = std::abs((*upperTimeIt - time).toSec());
		if (upperTimeIt != optimizedPosesTimes_.begin())
		{
			auto lowerTimeIt = std::prev(upperTimeIt);
			interpolationTimeError =
				std::min(interpolationTimeError, std::abs((*lowerTimeIt - time).toSec()));
		}
		if (interpolationTimeError > maxInterpolationTimeError_)
		{
			canInterpolate = false;
		}
	}
	ros::Duration extrapolationTime;
	if (time > lastOptimizedPoseTime_)
	{
		extrapolationTime = time - lastOptimizedPoseTime_;
	}
	else
	{
		extrapolationTime = lastOptimizedPoseTime_ - time;
	}
	if (canInterpolate && !baseLinkFrame_.empty())
	{
		for (const auto& trajectoryBuffer : trajectoryBuffers_)
		{
			try
			{
				geometry_msgs::TransformStamped pose = trajectoryBuffer.lookupTransform(
					mapFrame_, baseLinkFrame_, time);
				return std::optional<rtabmap::Transform>(transformFromGeometryMsg(pose.transform));
			}
			catch (...)
			{
				continue;
			}
		}
	}
	else if ((odometryCorrection_ || defaultIdentityOdometryCorrection) && odometryPose &&
		(maxExtrapolationTime == ros::Duration(0) || extrapolationTime <= maxExtrapolationTime))
	{
		if (!odometryCorrection_.has_value())
		{
			return std::optional<rtabmap::Transform>(*odometryPose);
		}
		rtabmap::Transform pose = *odometryCorrection_ * (*odometryPose);
		return std::optional<rtabmap::Transform>(pose);
	}
	return std::optional<rtabmap::Transform>();
}

void OccupancyGridBuilder::commonLaserScanCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const sensor_msgs::LaserScan& scanMsg,
		const sensor_msgs::PointCloud2& scan3dMsg,
		bool temporaryMapping)
{
	UScopeMutex lock(mutex_);
	MEASURE_BLOCK_TIME(commonLaserScanCallback);
	nav_msgs::Odometry* odomPtr = new nav_msgs::Odometry();
	odomPtr->header.frame_id = "odom";
	odomPtr->header.stamp = scan3dMsg.header.stamp;
	odomPtr->child_frame_id = "base_link";
	odomPtr->pose.pose.orientation.w = 1;
	nav_msgs::OdometryConstPtr odomShrd(odomPtr);
	mappingPipeline(odomShrd, scan3dMsg, {}, {}, temporaryMapping);
}

void OccupancyGridBuilder::commonRGBCallback(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
		const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
		const sensor_msgs::LaserScan& scanMsg,
		const sensor_msgs::PointCloud2& scan3dMsg,
		bool temporaryMapping)
{
	UScopeMutex lock(mutex_);
	MEASURE_BLOCK_TIME(commonRGBCallback);
	nav_msgs::Odometry* odomPtr = new nav_msgs::Odometry();
	odomPtr->header.frame_id = "odom";
	odomPtr->header.stamp = scan3dMsg.header.stamp;
	odomPtr->child_frame_id = "base_link";
	odomPtr->pose.pose.orientation.w = 1;
	nav_msgs::OdometryConstPtr odomShrd(odomPtr);
	mappingPipeline(odomShrd, scan3dMsg, imageMsgs, cameraInfoMsgs, temporaryMapping);
}

void OccupancyGridBuilder::mappingPipeline(
		const nav_msgs::OdometryConstPtr& odomMsg,
		const sensor_msgs::PointCloud2& scan3dMsg,
		const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
		const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
		bool temporaryMapping)
{
	UASSERT(odomMsg);
	UASSERT(scan3dMsg.data.size());
	UASSERT(odomFrame_.empty() ||
			(odomFrame_ == odomMsg->header.frame_id && baseLinkFrame_ == odomMsg->child_frame_id));
	if (odomFrame_.empty())
	{
		odomFrame_ = odomMsg->header.frame_id;
		baseLinkFrame_ = odomMsg->child_frame_id;
	}

	rtabmap::Transform odometryPose = transformFromPoseMsg(odomMsg->pose.pose);
	std::optional<rtabmap::Transform> optimizedPose = getOptimizedPose(
		odomMsg->header.stamp, &odometryPose, ros::Duration(0), true);
	UASSERT(optimizedPose.has_value());
	rtabmap::Signature signature;
	signature = createSignature(
		*optimizedPose,
		odomMsg->header.stamp,
		scan3dMsg,
		imageMsgs,
		cameraInfoMsgs);
	addSignatureToOccupancyGrid(signature, odometryPose, temporaryMapping);
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

rtabmap::Signature OccupancyGridBuilder::createSignature(
		const rtabmap::Transform& pose,
		const ros::Time& time,
		const sensor_msgs::PointCloud2& scan3dMsg,
		const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
		const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs)
{
	rtabmap::LaserScan scan;
	if (scan3dMsg.data.size())
	{
		MEASURE_BLOCK_TIME(convertScan3dMsg);
		bool convertionOk = convertScan3dMsg(scan3dMsg, baseLinkFrame_, "", ros::Time(0), scan, tfListener_, 0.0);
		UASSERT(convertionOk);
	}

	std::vector<cv::Mat> rgbs;
	std::vector<rtabmap::CameraModel> cameraModels;
	if (imageMsgs.size())
	{
		MEASURE_BLOCK_TIME(convertRGBMsgs);
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

void OccupancyGridBuilder::addSignatureToOccupancyGrid(const rtabmap::Signature& signature,
		const rtabmap::Transform& odometryPose, bool temporary /* false */)
{
	rtabmap::OccupancyGridBuilder::LocalMap localMap = occupancyGridBuilder_.createLocalMap(signature);
	if (temporary)
	{
		occupancyGridBuilder_.addTemporaryLocalMap(signature.getPose(), std::move(localMap));
		temporaryTimesPoses_.emplace_back(ros::Time(signature.getSec(), signature.getNSec()),
			odometryPose);
		if (temporaryTimesPoses_.size() > occupancyGridBuilder_.maxTemporaryLocalMaps())
		{
			temporaryTimesPoses_.pop_front();
		}
	}
	else
	{
		occupancyGridBuilder_.addLocalMap(nodeId_, signature.getPose(), std::move(localMap));
		times_[nodeId_] = ros::Time(signature.getSec(), signature.getNSec());
		posesAfterLastUpdate_[nodeId_] = odometryPose;
		nodeId_++;
	}
}

void OccupancyGridBuilder::publishOccupancyGridMaps(const ros::Time& stamp)
{
	nav_msgs::OccupancyGrid occupancyGridMsg = getOccupancyGridMsg(stamp);
	occupancyGridPub_.publish(occupancyGridMsg);

	colored_occupancy_grid_msgs::ColoredOccupancyGrid coloredOccupancyGridMsg;
	coloredOccupancyGridMsg.header = occupancyGridMsg.header;
	coloredOccupancyGridMsg.info = occupancyGridMsg.info;
	coloredOccupancyGridMsg.data = occupancyGridMsg.data;
	fillColorsInColoredOccupancyGridMsg(coloredOccupancyGridMsg);
	maybeDrawDoorOnColoredOccupancyGridMsg(coloredOccupancyGridMsg);
	coloredOccupancyGridPub_.publish(coloredOccupancyGridMsg);
}

nav_msgs::OccupancyGrid OccupancyGridBuilder::getOccupancyGridMsg(const ros::Time& stamp)
{
	float xMin, yMin;
	rtabmap::OccupancyGridBuilder::OccupancyGrid occupancyGrid =
		occupancyGridBuilder_.getOccupancyGrid(xMin, yMin);
	UASSERT(occupancyGrid.size());

	nav_msgs::OccupancyGrid occupancyGridMsg;
	occupancyGridMsg.header.stamp = stamp;
	occupancyGridMsg.header.frame_id = mapFrame_;
	occupancyGridMsg.info.resolution = occupancyGridBuilder_.cellSize();
	occupancyGridMsg.info.origin.position.x = 0.0;
	occupancyGridMsg.info.origin.position.y = 0.0;
	occupancyGridMsg.info.origin.position.z = 0.0;
	occupancyGridMsg.info.origin.orientation.x = 0.0;
	occupancyGridMsg.info.origin.orientation.y = 0.0;
	occupancyGridMsg.info.origin.orientation.z = 0.0;
	occupancyGridMsg.info.origin.orientation.w = 1.0;

	occupancyGridMsg.info.width = occupancyGrid.cols();
	occupancyGridMsg.info.height = occupancyGrid.rows();
	occupancyGridMsg.info.origin.position.x = xMin;
	occupancyGridMsg.info.origin.position.y = yMin;
	occupancyGridMsg.data.resize(occupancyGrid.size());

	memcpy(occupancyGridMsg.data.data(), occupancyGrid.data(), occupancyGrid.size());
	return occupancyGridMsg;
}

void OccupancyGridBuilder::fillColorsInColoredOccupancyGridMsg(
		colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg)
{
	float xMin, yMin;
	rtabmap::OccupancyGridBuilder::ColorGrid colorGrid = occupancyGridBuilder_.getColorGrid(xMin, yMin);
	for (int h = 0; h < colorGrid.rows(); h++)
	{
		for (int w = 0; w < colorGrid.cols(); w++)
		{
			int color = colorGrid(h, w);
			if (color == rtabmap::LocalMapBuilder::Color::missingColor.data())
			{
				color = 0;
			}
			coloredOccupancyGridMsg.b.push_back(color & 0xFF);
			coloredOccupancyGridMsg.g.push_back((color >> 8) & 0xFF);
			coloredOccupancyGridMsg.r.push_back((color >> 16) & 0xFF);
		}
	}
}

void OccupancyGridBuilder::maybeDrawDoorOnColoredOccupancyGridMsg(
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

void OccupancyGridBuilder::drawDoorCenterUsedAsEstimationOnColoredOccupancyGrid(
		colored_occupancy_grid_msgs::ColoredOccupancyGrid& coloredOccupancyGridMsg,
		const cv::Vec3b& color)
{
	if (doorCenterInMapFrame_.first == std::numeric_limits<int>::min())
	{
		return;
	}
	int width =  coloredOccupancyGridMsg.info.width;
	float cellSize = occupancyGridBuilder_.cellSize();
	float originX, originY;
	std::tie(originX, originY) = occupancyGridBuilder_.getGridOrigin();
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

void OccupancyGridBuilder::drawDoorCornersOnColoredOccupancyGrid(
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
	float cellSize = occupancyGridBuilder_.cellSize();
	float originX, originY;
	std::tie(originX, originY) = occupancyGridBuilder_.getGridOrigin();
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

void OccupancyGridBuilder::publishLastDilatedSemantic(const ros::Time& stamp, const std::string& frame_id)
{
	const cv::Mat& lastDilatedSemantic = occupancyGridBuilder_.lastDilatedSemantic();
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

void OccupancyGridBuilder::tryToPublishDoorCorners(const ros::Time& stamp)
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
		float cellSize = occupancyGridBuilder_.cellSize();
		doorCornersMsg.first_corner.x = doorCornersInMapFrame_.first.second * cellSize + cellSize / 2;
		doorCornersMsg.first_corner.y = doorCornersInMapFrame_.first.first * cellSize + cellSize / 2;
		doorCornersMsg.first_corner.z = 0;
		doorCornersMsg.second_corner.x = doorCornersInMapFrame_.second.second * cellSize + cellSize / 2;
		doorCornersMsg.second_corner.y = doorCornersInMapFrame_.second.first * cellSize + cellSize / 2;
		doorCornersMsg.second_corner.z = 0;
	}
	doorCornersPub_.publish(doorCornersMsg);
}

void OccupancyGridBuilder::startDoorTracking(const geometry_msgs::PointConstPtr& doorCenterEstimation)
{
	UScopeMutex lock(mutex_);
	doorCenterInMapFrame_.first = std::lround(doorCenterEstimation->y / occupancyGridBuilder_.cellSize());
	doorCenterInMapFrame_.second = std::lround(doorCenterEstimation->x / occupancyGridBuilder_.cellSize());
	doorCornersInMapFrame_.first.first = -1;
	doorCenterUsedAsEstimationInMapFrame_ = doorCenterInMapFrame_;
}

void OccupancyGridBuilder::trackDoor()
{
	UASSERT(doorCenterInMapFrame_.first != std::numeric_limits<int>::min());
	float originX, originY;
	rtabmap::OccupancyGridBuilder::OccupancyGrid occupancyGrid =
		occupancyGridBuilder_.getOccupancyGrid(originX, originY);
	int width = occupancyGrid.cols();
	int height = occupancyGrid.rows();
	cv::Mat image(height, width, CV_8U, occupancyGrid.data());
	float cellSize = occupancyGridBuilder_.cellSize();
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

void OccupancyGridBuilder::stopDoorTracking(const std_msgs::EmptyConstPtr& empty)
{
	UScopeMutex lock(mutex_);
	doorCenterInMapFrame_.first = std::numeric_limits<int>::min();
	doorCenterInMapFrame_.second = std::numeric_limits<int>::min();
}

}
