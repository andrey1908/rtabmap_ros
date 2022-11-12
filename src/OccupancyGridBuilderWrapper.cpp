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
	pnh.param("map_path", mapPath_, std::string(""));
	pnh.param("load_map", loadMap_, false);
	pnh.param("save_map", saveMap_, false);
	pnh.param("cache_map", cacheMap_, true);
	pnh.param("needs_localization", needsLocalization_, true);
	pnh.param("accumulative_mapping", accumulativeMapping_, true);
	pnh.param("temporary_mapping", temporaryMapping_, false);
}

OccupancyGridBuilder::OccupancyGridBuilder(int argc, char** argv) :
			nodeId_(0)
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	rtabmap::ParametersMap parameters = readRtabmapParameters(argc, argv, pnh);
	readRtabmapRosParameters(pnh);
	UASSERT(accumulativeMapping_ || temporaryMapping_);

	occupancyGridBuilder_.parseParameters(parameters);
	occupancyGridPub_ = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);
	coloredOccupancyGridPub_ = nh.advertise<colored_occupancy_grid::ColoredOccupancyGrid>("colored_grid_map", 1);
	dilatedSemanticPub_ = nh.advertise<sensor_msgs::Image>("dilated_semantic_image", 1);
	if (mapPath_.empty())
	{
		loadMap_ = false;
		saveMap_ = false;
	}
	if (loadMap_)
	{
		load();
		if (needsLocalization_)
		{
			occupancyGridBuilder_.updatePoses({});
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
}

OccupancyGridBuilder::~OccupancyGridBuilder()
{
	if (saveMap_)
	{
		save();
	}
}

void OccupancyGridBuilder::load()
{
	MEASURE_BLOCK_TIME(load);
	std::fstream fs(mapPath_, std::fstream::in | std::fstream::binary | std::fstream::app);
	UASSERT(fs.is_open());

	UASSERT(fs.peek() != EOF);
	int version;
	fs.read((char*)(&version), sizeof(version));
	UASSERT(version == 1);

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
		std::vector<int> colors;

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

		if (nodeId > maxNodeId)
		{
			maxNodeId = nodeId;
		}
		rtabmap::Transform pose;
		if (eigenPose != Eigen::Matrix4f::Zero())
		{
			pose = rtabmap::Transform::fromEigen4f(eigenPose);
		}
		times_[nodeId] = time;

		rtabmap::OccupancyGridBuilder::LocalMap localMap;
		localMap.numGround = numGround;
		localMap.numEmpty = numEmpty;
		localMap.numObstacles = numObstacles;
		localMap.points = std::move(points);
		localMap.colors = std::move(colors);
		if (eigenPose != Eigen::Matrix4f::Zero())
		{
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

void OccupancyGridBuilder::save()
{
	MEASURE_BLOCK_TIME(save);
	std::fstream fs(mapPath_, std::fstream::out | std::fstream::binary | std::fstream::trunc);
	UASSERT(fs.is_open());

	int version = 1;
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
		int numGround = localMap.numGround;
		int numEmpty = localMap.numEmpty;
		int numObstacles = localMap.numObstacles;
		const Eigen::Matrix3Xf& points = localMap.points;
		const std::vector<int>& colors = localMap.colors;

		UASSERT(timeIt != times_.end());

		fs.write((const char*)(&nodeId), sizeof(nodeId));
		if (pose.has_value())
		{
			const Eigen::Matrix4f& eigenPose = pose->toEigen4f();
			fs.write((const char*)(eigenPose.data()), eigenPose.size() * sizeof(eigenPose(0, 0)));
		}
		else
		{
			const Eigen::Matrix4f& nullEigenPose = Eigen::Matrix4f::Zero();
			fs.write((const char*)(nullEigenPose.data()), nullEigenPose.size() * sizeof(nullEigenPose(0, 0)));
		}
		fs.write((const char*)(&timeIt->second.sec), sizeof(timeIt->second.sec));
		fs.write((const char*)(&timeIt->second.nsec), sizeof(timeIt->second.nsec));
		fs.write((const char*)(&numGround), sizeof(numGround));
		fs.write((const char*)(&numEmpty), sizeof(numEmpty));
		fs.write((const char*)(&numObstacles), sizeof(numObstacles));
		fs.write((const char*)(points.data()), points.size() * sizeof(points(0, 0)));
		fs.write((const char*)(colors.data()), colors.size() * sizeof(colors[0]));
	}
	fs.close();
}

void OccupancyGridBuilder::updatePoses(const optimization_results_msgs::OptimizationResults::ConstPtr& optimizationResults)
{
	UScopeMutex lock(mutex_);
	lastOptimizationResultsTime_ = optimizationResults->header.stamp;
	trajectoryBuffers_.clear();
	odometryCorrection_.reset();

	int trajectory_counter = 0;
	ros::Time latest_trajectory_start_time;
	for (const auto& trajectory : optimizationResults->trajectories)
	{
		trajectoryBuffers_.emplace_back(ros::Duration(1000000));
		tf2_ros::Buffer& trajectoryBuffer = *trajectoryBuffers_.rbegin();
		bool addedStaticTransformToBaseLink = false;
		ros::Time current_trajectory_start_time = trajectory.global_poses.begin()->header.stamp;
		for (const geometry_msgs::TransformStamped& pose : trajectory.global_poses)
		{
			UASSERT(mapFrame_.empty() || mapFrame_ == pose.header.frame_id);
			if (mapFrame_.empty())
			{
				mapFrame_ = pose.header.frame_id;
			}
			trajectoryBuffer.setTransform(pose, "default");
			if (pose.child_frame_id != baseLinkFrame_ &&
				!baseLinkFrame_.empty() &&
				addedStaticTransformToBaseLink == false)
			{
				tf::StampedTransform toBaseLinkTF;
				tfListener_.lookupTransform(pose.child_frame_id, baseLinkFrame_, ros::Time(0), toBaseLinkTF);
				geometry_msgs::TransformStamped toBaseLink;
				transformStampedTFToMsg(toBaseLinkTF, toBaseLink);
				trajectoryBuffer.setTransform(toBaseLink, "default", true);
				addedStaticTransformToBaseLink = true;
			}
			current_trajectory_start_time = std::min(current_trajectory_start_time, pose.header.stamp);
		}
		latest_trajectory_start_time = std::max(latest_trajectory_start_time, current_trajectory_start_time);
		trajectory_counter++;
	}

	UASSERT((mapFrame_.empty() || optimizationResults->map_to_odom.header.frame_id.empty() ||
			mapFrame_ == optimizationResults->map_to_odom.header.frame_id) &&
			(odomFrame_.empty() || optimizationResults->map_to_odom.child_frame_id.empty() ||
			odomFrame_ == optimizationResults->map_to_odom.child_frame_id));
	if (!optimizationResults->map_to_odom.header.frame_id.empty())
	{
		if (mapFrame_.empty())
		{
			mapFrame_ = optimizationResults->map_to_odom.header.frame_id;
		}
		if (odometryCorrection_)
		{
			*odometryCorrection_ = optimizationResults->map_to_odom;
		}
		else
		{
			odometryCorrection_ = std::make_unique<geometry_msgs::TransformStamped>(optimizationResults->map_to_odom);
		}
	}
	else
	{
		odometryCorrection_.reset();
	}

	std::map<int, rtabmap::Transform> updatedPoses;
	for (const auto& idTime: times_)
	{
		int nodeId = idTime.first;
		ros::Time time = idTime.second;
		std::optional<rtabmap::Transform> pose = getPose(time);
		if (pose.has_value())
		{
			updatedPoses[nodeId] = *pose;
		}
	}

	std::list<rtabmap::Transform> updatedTemporaryPoses;
	if (trajectoryBuffers_.size() == 0)
	{
		occupancyGridBuilder_.resetTemporaryMap();
		temporaryOdometryPoses_.clear();
		temporaryTimes_.clear();
	}
	else
	{
		auto temporaryOdometryPoseIt = temporaryOdometryPoses_.begin();
		auto temporaryTimeIt = temporaryTimes_.begin();
		for (; temporaryOdometryPoseIt != temporaryOdometryPoses_.end(); ++temporaryOdometryPoseIt, ++temporaryTimeIt)
		{
			std::optional<rtabmap::Transform> pose = getPose(*temporaryTimeIt, &(*temporaryOdometryPoseIt), ros::Duration(1, 0));
			UASSERT(pose.has_value());
			updatedTemporaryPoses.push_back(*pose);
		}
	}

	occupancyGridBuilder_.updatePoses(updatedPoses, updatedTemporaryPoses);

	std::vector<int> nodeIdsToCache;
	nodeIdsToCache.reserve(updatedPoses.size());
	for (const auto& updatedPose : updatedPoses)
	{
		if (times_.at(updatedPose.first) < latest_trajectory_start_time)
		{
			nodeIdsToCache.push_back(updatedPose.first);
		}
	}
	if (cacheMap_ && nodeIdsToCache.size())
	{
		occupancyGridBuilder_.cacheMap(nodeIdsToCache);
	}
}

std::optional<rtabmap::Transform> OccupancyGridBuilder::getPose(ros::Time time,
		const rtabmap::Transform* odometryPose /* nullptr */, ros::Duration maxDistance /* (0, 0) */,
		bool defaultIdentityOdometryCorrection /* false */)
{
	if (time <= lastOptimizationResultsTime_ && !mapFrame_.empty() && !baseLinkFrame_.empty())
	{
		for (const auto& trajectoryBuffer : trajectoryBuffers_)
		{
			try
			{
				geometry_msgs::TransformStamped pose = trajectoryBuffer.lookupTransform(mapFrame_, baseLinkFrame_, time);
				return std::optional<rtabmap::Transform>(transformFromGeometryMsg(pose.transform));
			}
			catch (...)
			{
				continue;
			}
		}
	}
	else if ((odometryCorrection_ || defaultIdentityOdometryCorrection) && odometryPose &&
		(maxDistance == ros::Duration(-1, 0) || time - lastOptimizationResultsTime_ <= maxDistance))
	{
		if (odometryCorrection_ == nullptr)
		{
			return std::optional<rtabmap::Transform>(*odometryPose);
		}
		rtabmap::Transform pose = transformFromGeometryMsg(odometryCorrection_->transform) * (*odometryPose);
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
	if (temporaryMapping)
	{
		UASSERT(temporaryCommonDataSubscriber_.isSubscribedToOdom());
		UASSERT(temporaryCommonDataSubscriber_.isSubscribedToScan3d());
	}
	else
	{
		UASSERT(commonDataSubscriber_.isSubscribedToOdom());
		UASSERT(commonDataSubscriber_.isSubscribedToScan3d());
	}
	UASSERT(odomFrame_.empty() ||
			(odomFrame_ == odomMsg->header.frame_id && baseLinkFrame_ == odomMsg->child_frame_id));
	if (odomFrame_.empty())
	{
		odomFrame_ = odomMsg->header.frame_id;
		baseLinkFrame_ = odomMsg->child_frame_id;
	}
	rtabmap::Transform odometryPose = transformFromPoseMsg(odomMsg->pose.pose);
	std::optional<rtabmap::Transform> correctedPose = getPose(odomMsg->header.stamp, &odometryPose, ros::Duration(-1, 0), true);
	UASSERT(correctedPose.has_value() || odomMsg->header.stamp <= lastOptimizationResultsTime_);
	if (!correctedPose.has_value())
	{
		return;
	}
	rtabmap::Signature signature = createSignature(
		*correctedPose,
		odomMsg->header.stamp,
		scan3dMsg,
		{}, {});
	addSignatureToOccupancyGrid(signature, odometryPose, temporaryMapping);
	publishOccupancyGridMaps(ros::Time(signature.getSec(), signature.getNSec()), !mapFrame_.empty() ? mapFrame_ : odomFrame_);
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
	if (temporaryMapping)
	{
		UASSERT(temporaryCommonDataSubscriber_.isSubscribedToOdom());
		UASSERT(temporaryCommonDataSubscriber_.isSubscribedToScan3d());
	}
	else
	{
		UASSERT(commonDataSubscriber_.isSubscribedToOdom());
		UASSERT(commonDataSubscriber_.isSubscribedToScan3d());
	}
	UASSERT(odomFrame_.empty() ||
			(odomFrame_ == odomMsg->header.frame_id && baseLinkFrame_ == odomMsg->child_frame_id));
	if (odomFrame_.empty())
	{
		odomFrame_ = odomMsg->header.frame_id;
		baseLinkFrame_ = odomMsg->child_frame_id;
	}
	rtabmap::Transform odometryPose = transformFromPoseMsg(odomMsg->pose.pose);
	std::optional<rtabmap::Transform> correctedPose = getPose(odomMsg->header.stamp, &odometryPose, ros::Duration(-1, 0), true);
	UASSERT(correctedPose.has_value() || odomMsg->header.stamp <= lastOptimizationResultsTime_);
	if (!correctedPose.has_value())
	{
		return;
	}
	rtabmap::Signature signature;
	signature = createSignature(
		*correctedPose,
		odomMsg->header.stamp,
		scan3dMsg,
		imageMsgs,
		cameraInfoMsgs);
	addSignatureToOccupancyGrid(signature, odometryPose, temporaryMapping);
	publishOccupancyGridMaps(ros::Time(signature.getSec(), signature.getNSec()), !mapFrame_.empty() ? mapFrame_ : odomFrame_);
	publishLastDilatedSemantic(ros::Time(signature.getSec(), signature.getNSec()), imageMsgs[0]->header.frame_id);
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
	data.setId(nodeId_);
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
		temporaryOdometryPoses_.push_back(odometryPose);
		temporaryTimes_.push_back(ros::Time(signature.getSec(), signature.getNSec()));
		if (temporaryTimes_.size() > occupancyGridBuilder_.maxTemporaryLocalMaps())
		{
			temporaryOdometryPoses_.pop_front();
			temporaryTimes_.pop_front();
		}
	}
	else
	{
		occupancyGridBuilder_.addLocalMap(nodeId_, signature.getPose(), std::move(localMap));
		times_[nodeId_] = ros::Time(signature.getSec(), signature.getNSec());
		nodeId_++;
	}
}

void OccupancyGridBuilder::publishOccupancyGridMaps(ros::Time stamp, const std::string& frame_id)
{
	nav_msgs::OccupancyGrid occupancyGridMsg = getOccupancyGridMsg();
	occupancyGridMsg.header.stamp = stamp;
	occupancyGridMsg.header.frame_id = frame_id;
	occupancyGridPub_.publish(occupancyGridMsg);

	colored_occupancy_grid::ColoredOccupancyGrid coloredOccupancyGridMsg;
	coloredOccupancyGridMsg.header = occupancyGridMsg.header;
	coloredOccupancyGridMsg.info = occupancyGridMsg.info;
	coloredOccupancyGridMsg.data = occupancyGridMsg.data;
	float xMin, yMin;
	rtabmap::OccupancyGridBuilder::ColorGrid colorGrid = occupancyGridBuilder_.getColorGrid(xMin, yMin);
	for (int h = 0; h < colorGrid.rows(); h++)
	{
		for (int w = 0; w < colorGrid.cols(); w++)
		{
			int color = colorGrid(h, w);
			if (color == -1)
			{
				color = 0;
			}
			coloredOccupancyGridMsg.b.push_back(color & 0xFF);
			coloredOccupancyGridMsg.g.push_back((color >> 8) & 0xFF);
			coloredOccupancyGridMsg.r.push_back((color >> 16) & 0xFF);
		}
	}
	coloredOccupancyGridPub_.publish(coloredOccupancyGridMsg);
}

nav_msgs::OccupancyGrid OccupancyGridBuilder::getOccupancyGridMsg()
{
	float xMin, yMin;
	rtabmap::OccupancyGridBuilder::OccupancyGrid occupancyGrid =
		occupancyGridBuilder_.getOccupancyGrid(xMin, yMin);
	UASSERT(occupancyGrid.size());

	nav_msgs::OccupancyGrid occupancyGridMsg;
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

void OccupancyGridBuilder::publishLastDilatedSemantic(ros::Time stamp, const std::string& frame_id)
{
	const cv::Mat lastDilatedSemantic = occupancyGridBuilder_.lastDilatedSemantic();
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
