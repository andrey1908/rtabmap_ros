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

namespace rtabmap_ros {

rtabmap::ParametersMap OccupancyGridBuilder::readRtabmapParameters(int argc, char** argv, const ros::NodeHandle& pnh)
{
	std::string configPath;
	pnh.param("config_path", configPath, configPath);

	//parameters
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
	pnh.param("cache_loaded_map", cacheLoadedMap_, true);
	pnh.param("needs_localization", needsLocalization_, true);
	pnh.param("temporary_mapping", temporaryMapping_, false);
}

OccupancyGridBuilder::OccupancyGridBuilder(int argc, char** argv) :
			CommonDataSubscriber(false),
			nodeId_(0),
			lastOptimizedPoseTime_(0)
{
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	rtabmap::ParametersMap parameters = readRtabmapParameters(argc, argv, pnh);
	readRtabmapRosParameters(pnh);

	occupancyGrid_.parseParameters(parameters);
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
		if (cacheLoadedMap_)
		{
			occupancyGrid_.cacheCurrentMap();
		}
		if (needsLocalization_)
		{
			poses_.clear();
			occupancyGrid_.updatePoses(poses_);
		}
	}
	setupCallbacks(nh, pnh, "DataSubscriber");
	optimizedPosesSub_ = nh.subscribe("optimized_poses", 1, &OccupancyGridBuilder::updatePoses, this);
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
	UASSERT(cellSize == occupancyGrid_.getCellSize());

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
			poses_[nodeId] = pose;
		}
		times_[nodeId] = time;

		rtabmap::OccupancyGrid::LocalMap localMap;
		localMap.numGround = numGround;
		localMap.numEmpty = numEmpty;
		localMap.numObstacles = numObstacles;
		localMap.points = std::move(points);
		localMap.colors = std::move(colors);
		if (eigenPose != Eigen::Matrix4f::Zero())
		{
			occupancyGrid_.addLocalMap(nodeId, pose, std::move(localMap));
		}
		else
		{
			occupancyGrid_.addLocalMap(nodeId, std::move(localMap));
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

	float cellSize = occupancyGrid_.getCellSize();
	fs.write((const char*)(&cellSize), sizeof(cellSize));

	const auto& localMaps = occupancyGrid_.localMaps();
	for (const auto& nodeIdLocalMap : localMaps)
	{
		int nodeId = nodeIdLocalMap.first;
		auto poseIt = poses_.find(nodeId);
		auto timeIt = times_.find(nodeId);
		const auto& localMap = nodeIdLocalMap.second;
		int numGround = localMap.numGround;
		int numEmpty = localMap.numEmpty;
		int numObstacles = localMap.numObstacles;
		const Eigen::Matrix3Xf& points = localMap.points;
		const std::vector<int>& colors = localMap.colors;

		UASSERT(timeIt != times_.end());

		fs.write((const char*)(&nodeId), sizeof(nodeId));
		if (poseIt == poses_.end())
		{
			const Eigen::Matrix4f& nullEigenPose = Eigen::Matrix4f::Zero();
			fs.write((const char*)(nullEigenPose.data()), nullEigenPose.size() * sizeof(nullEigenPose(0, 0)));
		}
		else
		{
			const Eigen::Matrix4f& eigenPose = poseIt->second.toEigen4f();
			fs.write((const char*)(eigenPose.data()), eigenPose.size() * sizeof(eigenPose(0, 0)));
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

void OccupancyGridBuilder::updatePoses(const nav_msgs::Path::ConstPtr& optimizedPoses)
{
	UScopeMutex lock(mutex_);
	if (mapFrame_.empty())
	{
		return;
	}

	lastOptimizedPoseTime_ = optimizedPoses->header.stamp;
	optimizedPosesBuffers_.clear();
	odometryCorrectonTransform_.reset();

	const std::string& mapFrame = optimizedPoses->header.frame_id;
	UASSERT(mapFrame == mapFrame_);

	std::set<std::string> addedPosesFrames;
	for (const auto& pose: optimizedPoses->poses)
	{
		geometry_msgs::TransformStamped tf;
		tf.header.stamp = pose.header.stamp;
		tf.transform.translation.x = pose.pose.position.x;
		tf.transform.translation.y = pose.pose.position.y;
		tf.transform.translation.z = pose.pose.position.z;
		tf.transform.rotation.x = pose.pose.orientation.x;
		tf.transform.rotation.y = pose.pose.orientation.y;
		tf.transform.rotation.z = pose.pose.orientation.z;
		tf.transform.rotation.w = pose.pose.orientation.w;

		if (pose.header.frame_id.empty())
		{
			if (odometryCorrectonTransform_ == nullptr)
			{
				odometryCorrectonTransform_ = std::make_unique<geometry_msgs::TransformStamped>();
				odometryCorrectonTransform_->header.stamp = ros::Time();
			}
			if (tf.header.stamp > odometryCorrectonTransform_->header.stamp)
			{
				*odometryCorrectonTransform_ = tf;
			}
		}
		else
		{
			tf.header.frame_id = mapFrame;
			tf.child_frame_id = pose.header.frame_id;

			if (optimizedPosesBuffers_.count(pose.header.seq) == 0)
			{
				optimizedPosesBuffers_.emplace(pose.header.seq, ros::Duration(1000000));
			}
			optimizedPosesBuffers_.at(pose.header.seq).setTransform(tf, "default");

			if (tf.child_frame_id != baseLinkFrame_ &&
				addedPosesFrames.count(tf.child_frame_id) == 0)
			{
				tf::StampedTransform tfFromPoseToBaseLinkTF;
				tfListener_.lookupTransform(pose.header.frame_id, baseLinkFrame_, ros::Time(0), tfFromPoseToBaseLinkTF);
				geometry_msgs::TransformStamped tfFromPoseToBaseLink;
				transformStampedTFToMsg(tfFromPoseToBaseLinkTF, tfFromPoseToBaseLink);
				optimizedPosesBuffers_.at(pose.header.seq).setTransform(tfFromPoseToBaseLink, "default", true);
				addedPosesFrames.insert(pose.header.frame_id);
			}
		}
	}

	for (const auto& idTime: times_)
	{
		int nodeId = idTime.first;
		ros::Time time = idTime.second;
		geometry_msgs::Pose pose;
		std::unique_ptr<geometry_msgs::Pose> oldPose;
		if (poses_.count(nodeId))
		{
			oldPose = std::make_unique<geometry_msgs::Pose>();
			transformToPoseMsg(poses_.at(nodeId), *oldPose);
		}
		bool found = getPose(pose, time, ros::Duration(1, 0), oldPose.get());
		if (found)
		{
			poses_[nodeId] = transformFromPoseMsg(pose);
		}
		else
		{
			poses_.erase(nodeId);
		}
	}

	auto temporaryPoseIt = temporaryPoses_.begin();
	auto temporaryTimeIt = temporaryTimes_.begin();
	for (; temporaryPoseIt != temporaryPoses_.end(); ++temporaryPoseIt, ++temporaryTimeIt)
	{
		geometry_msgs::Pose pose;
		geometry_msgs::Pose oldPose;
		transformToPoseMsg(*temporaryPoseIt, oldPose);
		bool found = getPose(pose, *temporaryTimeIt, ros::Duration(1, 0), &oldPose);
		if (found == false)
		{
			occupancyGrid_.clearTemporaryLocalMaps();
			temporaryPoses_.clear();
			temporaryTimes_.clear();
			break;
		}
		*temporaryPoseIt = transformFromPoseMsg(pose);
	}

	occupancyGrid_.updatePoses(poses_, temporaryPoses_);
}

bool OccupancyGridBuilder::getPose(geometry_msgs::Pose& pose, ros::Time time,
		const ros::Duration maxAllowedDistance, const geometry_msgs::Pose* oldPose)
{
	for (const auto& optimizedPosesBuffer : optimizedPosesBuffers_)
	{
		try
		{
			geometry_msgs::Transform tf = optimizedPosesBuffer.second.lookupTransform(mapFrame_, baseLinkFrame_, time).transform;
			pose.position.x = tf.translation.x;
			pose.position.y = tf.translation.y;
			pose.position.z = tf.translation.z;
			pose.orientation.x = tf.rotation.x;
			pose.orientation.y = tf.rotation.y;
			pose.orientation.z = tf.rotation.z;
			pose.orientation.w = tf.rotation.w;
			return true;
		}
		catch (...)
		{
			continue;
		}
	}

	if (odometryCorrectonTransform_ == nullptr)
	{
		return false;
	}
	if (oldPose == nullptr)
	{
		return false;
	}
	ros::Duration distance = time - odometryCorrectonTransform_->header.stamp;
	if (distance < ros::Duration() || distance > maxAllowedDistance)
	{
		return false;
	}
	tf2::doTransform(*oldPose, pose, *odometryCorrectonTransform_);
	return true;
}

nav_msgs::OdometryConstPtr OccupancyGridBuilder::correctOdometry(nav_msgs::OdometryConstPtr odomMsg)
{
	if (odomMsg->header.stamp <= lastOptimizedPoseTime_)
	{
		nav_msgs::OdometryPtr correctedOdomMsg(new nav_msgs::Odometry(*odomMsg));
		bool found = getPose(correctedOdomMsg->pose.pose, odomMsg->header.stamp,
			ros::Duration(1, 0), &odomMsg->pose.pose);
		if (found == false) {
			occupancyGrid_.clearTemporaryLocalMaps();
			temporaryPoses_.clear();
			temporaryTimes_.clear();
			return nullptr;
		}
		return correctedOdomMsg;
	}
	return odomMsg;
}

void OccupancyGridBuilder::commonDepthCallback(
				const nav_msgs::OdometryConstPtr& odomMsg,
				const rtabmap_ros::UserDataConstPtr& userDataMsg,
				const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
				const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
				const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs,
				const sensor_msgs::LaserScan& scanMsg,
				const sensor_msgs::PointCloud2& scan3dMsg,
				const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
				const std::vector<rtabmap_ros::GlobalDescriptor>& globalDescriptorMsgs /* std::vector<rtabmap_ros::GlobalDescriptor>() */,
				const std::vector<std::vector<rtabmap_ros::KeyPoint>>& localKeyPoints /* std::vector<std::vector<rtabmap_ros::KeyPoint>>() */,
				const std::vector<std::vector<rtabmap_ros::Point3f>>& localPoints3d /* std::vector<std::vector<rtabmap_ros::Point3f>>() */,
				const std::vector<cv::Mat>& localDescriptors /* std::vector<cv::Mat>() */)
{
	UScopeMutex lock(mutex_);
	if (mapFrame_.empty())
	{
		mapFrame_ = odomMsg->header.frame_id;
		baseLinkFrame_ = odomMsg->child_frame_id;
	}
	nav_msgs::OdometryConstPtr correctedOdomMsg = correctOdometry(odomMsg);
	if (correctedOdomMsg == nullptr)
	{
		return;
	}

	MEASURE_BLOCK_TIME(commonDepthCallback);
	UDEBUG("\n\nReceived new data");
	UASSERT(isSubscribedToOdom());
	UASSERT(isSubscribedToRGB());

	rtabmap::Signature signature;
	if (isSubscribedToScan3d())
	{
		signature = createSignature(correctedOdomMsg, scan3dMsg, imageMsgs, depthMsgs, cameraInfoMsgs);
	}
	else
	{
		UASSERT(isSubscribedToDepth());
		signature = createSignature(correctedOdomMsg, sensor_msgs::PointCloud2(), imageMsgs, depthMsgs, cameraInfoMsgs);
	}
	addSignatureToOccupancyGrid(signature, temporaryMapping_);
	publishOccupancyGridMaps(ros::Time(signature.getSec(), signature.getNSec()), odomMsg->header.frame_id);
	publishLastDilatedSemantic(ros::Time(signature.getSec(), signature.getNSec()), imageMsgs[0]->header.frame_id);
}

void OccupancyGridBuilder::commonLaserScanCallback(
			const nav_msgs::OdometryConstPtr& odomMsg,
			const rtabmap_ros::UserDataConstPtr& userDataMsg,
			const sensor_msgs::LaserScan& scanMsg,
			const sensor_msgs::PointCloud2& scan3dMsg,
			const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg,
			const rtabmap_ros::GlobalDescriptor& globalDescriptor /* rtabmap_ros::GlobalDescriptor() */)
{
	UScopeMutex lock(mutex_);
	if (mapFrame_.empty())
	{
		mapFrame_ = odomMsg->header.frame_id;
		baseLinkFrame_ = odomMsg->child_frame_id;
	}
	nav_msgs::OdometryConstPtr correctedOdomMsg = correctOdometry(odomMsg);
	if (correctedOdomMsg == nullptr)
	{
		return;
	}

	MEASURE_BLOCK_TIME(commonLaserScanCallback);
	UDEBUG("\n\nReceived new data");
	UASSERT(isSubscribedToOdom());
	UASSERT(isSubscribedToScan3d());

	rtabmap::Signature signature = createSignature(correctedOdomMsg, scan3dMsg,
		std::vector<cv_bridge::CvImageConstPtr>(),
		std::vector<cv_bridge::CvImageConstPtr>(),
		std::vector<sensor_msgs::CameraInfo>());
	addSignatureToOccupancyGrid(signature, temporaryMapping_);
	publishOccupancyGridMaps(ros::Time(signature.getSec(), signature.getNSec()), odomMsg->header.frame_id);
}

rtabmap::Signature OccupancyGridBuilder::createSignature(const nav_msgs::OdometryConstPtr& odomMsg,
														 const sensor_msgs::PointCloud2& scan3dMsg,
														 const std::vector<cv_bridge::CvImageConstPtr>& imageMsgs,
														 const std::vector<cv_bridge::CvImageConstPtr>& depthMsgs,
														 const std::vector<sensor_msgs::CameraInfo>& cameraInfoMsgs)
{
	rtabmap::LaserScan scan;
	if (scan3dMsg.data.size())
	{
		bool convertionOk = convertScan3dMsg(scan3dMsg, odomMsg->child_frame_id, "", ros::Time(0), scan, tfListener_, 0.0);
		UASSERT(convertionOk);
	}

	cv::Mat rgb;
	cv::Mat depth;
	std::vector<rtabmap::CameraModel> cameraModels;
	if (cameraInfoMsgs.size())
	{
		bool convertionOk = convertRGBDMsgs(imageMsgs, depthMsgs, cameraInfoMsgs, odomMsg->child_frame_id, "", ros::Time(0),
			rgb, depth, cameraModels, tfListener_, 0.0);
		UASSERT(convertionOk);
	}

	rtabmap::SensorData data;
	data.setStamp(odomMsg->header.stamp.sec, odomMsg->header.stamp.nsec);
	data.setId(nodeId_);
	data.setLaserScan(scan);
	data.setRGBDImage(rgb, depth, cameraModels);
	rtabmap::Signature signature(data);
	signature.setPose(transformFromPoseMsg(odomMsg->pose.pose));
	return signature;
}

void OccupancyGridBuilder::addSignatureToOccupancyGrid(const rtabmap::Signature& signature, bool temporary /* false */)
{
	rtabmap::OccupancyGrid::LocalMap localMap = occupancyGrid_.createLocalMap(signature);
	if (temporary)
	{
		occupancyGrid_.addTemporaryLocalMap(signature.getPose(), std::move(localMap));
		temporaryPoses_.push_back(signature.getPose());
		temporaryTimes_.push_back(ros::Time(signature.getSec(), signature.getNSec()));
		if (temporaryTimes_.size() > occupancyGrid_.getMaxTemporaryLocalMaps())
		{
			temporaryPoses_.pop_front();
			temporaryTimes_.pop_front();
		}
	}
	else
	{
		occupancyGrid_.addLocalMap(nodeId_, signature.getPose(), std::move(localMap));
		poses_[nodeId_] = signature.getPose();
		times_[nodeId_] = ros::Time(signature.getSec(), signature.getNSec());
		nodeId_++;
	}
}

void OccupancyGridBuilder::publishOccupancyGridMaps(ros::Time stamp, const std::string& frame_id)
{
	nav_msgs::OccupancyGrid map = getOccupancyGridMap();
	map.header.stamp = stamp;
	map.header.frame_id = frame_id;
	occupancyGridPub_.publish(map);

	colored_occupancy_grid::ColoredOccupancyGrid coloredMap;
	coloredMap.header = map.header;
	coloredMap.info = map.info;
	coloredMap.data = map.data;
	float xMin, yMin;
	cv::Mat colors = occupancyGrid_.getColors(xMin, yMin);
	for (int h = 0; h < colors.rows; h++)
	{
		for (int w = 0; w < colors.cols; w++)
		{
			cv::Vec3b color = colors.at<cv::Vec3b>(h, w);
			coloredMap.b.push_back(color[0]);
			coloredMap.g.push_back(color[1]);
			coloredMap.r.push_back(color[2]);
		}
	}
	coloredOccupancyGridPub_.publish(coloredMap);
}

nav_msgs::OccupancyGrid OccupancyGridBuilder::getOccupancyGridMap()
{
	float gridCellSize = occupancyGrid_.getCellSize();
	float xMin, yMin;
	cv::Mat pixels = occupancyGrid_.getMap(xMin, yMin);
	UASSERT(!pixels.empty());

	nav_msgs::OccupancyGrid map;
	map.info.resolution = gridCellSize;
	map.info.origin.position.x = 0.0;
	map.info.origin.position.y = 0.0;
	map.info.origin.position.z = 0.0;
	map.info.origin.orientation.x = 0.0;
	map.info.origin.orientation.y = 0.0;
	map.info.origin.orientation.z = 0.0;
	map.info.origin.orientation.w = 1.0;

	map.info.width = pixels.cols;
	map.info.height = pixels.rows;
	map.info.origin.position.x = xMin;
	map.info.origin.position.y = yMin;
	map.data.resize(map.info.width * map.info.height);

	memcpy(map.data.data(), pixels.data, map.info.width * map.info.height);
	return map;
}

void OccupancyGridBuilder::publishLastDilatedSemantic(ros::Time stamp, const std::string& frame_id)
{
	const cv::Mat lastDilatedSemantic = occupancyGrid_.lastDilatedSemantic();
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
