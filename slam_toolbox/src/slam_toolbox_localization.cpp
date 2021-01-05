/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include "slam_toolbox/slam_toolbox_localization.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
LocalizationSlamToolbox::LocalizationSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  odomOnly_ = false;

  processor_type_ = PROCESS_LOCALIZATION;
  localization_pose_sub_ = nh.subscribe("/initialpose", 1,
    &LocalizationSlamToolbox::localizePoseCallback, this);

    localization_use_odom_sub_ = nh.subscribe("/slam_toolbox/odom_only", 1,
      &LocalizationSlamToolbox::odomOnlyCallback, this);

  score_scan_match_pub_ = nh.advertise<std_msgs::Float64>("scan_match_score", 1000);
  best_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_toolbox_best_pose", 1000);

  std::string filename;
  geometry_msgs::Pose2D pose;
  bool dock = false;
  if (shouldStartWithPoseGraph(filename, pose, dock))
  {
    slam_toolbox_msgs::DeserializePoseGraph::Request req;
    slam_toolbox_msgs::DeserializePoseGraph::Response resp;
    req.initial_pose = pose;
    req.filename = filename;
    req.match_type =
      slam_toolbox_msgs::DeserializePoseGraph::Request::LOCALIZE_AT_POSE;
    if (dock)
    {
      ROS_ERROR("LocalizationSlamToolbox: Starting localization "
        "at first node (dock) is correctly not supported.");
    }

    deserializePoseGraphCallback(req, resp);
  }

  // in localization mode, we cannot allow for interactive mode
  enable_interactive_mode_ = false;

  // in localization mode, disable map saver
  map_saver_.reset();

  nh.param("map_frame", frame_id_, std::string("map"));
  return;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::serializePoseGraphCallback(
  slam_toolbox_msgs::SerializePoseGraph::Request& req,
  slam_toolbox_msgs::SerializePoseGraph::Response& resp)
/*****************************************************************************/
{
  ROS_FATAL("LocalizationSlamToolbox: Cannot call serialize map "
    "in localization mode!");
  return false;
}

/*****************************************************************************/
bool LocalizationSlamToolbox::deserializePoseGraphCallback(
  slam_toolbox_msgs::DeserializePoseGraph::Request& req,
  slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
  if (req.match_type != procType::LOCALIZE_AT_POSE)
  {
    ROS_ERROR("Requested a non-localization deserialization "
      "in localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

/*****************************************************************************/
void LocalizationSlamToolbox::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  // no odom info
  Pose2 pose;
  if(!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  if (shouldProcessScan(scan, pose))
  {
    addScan(laser, scan, pose);
  }

  return;
}

/*****************************************************************************/
LocalizedRangeScan* LocalizationSlamToolbox::addScan(
  LaserRangeFinder* laser,
  const sensor_msgs::LaserScan::ConstPtr& scan,
  Pose2& karto_pose)
/*****************************************************************************/
{
  boost::mutex::scoped_lock l(pose_mutex_);

  if (PROCESS_LOCALIZATION && process_near_pose_)
  {
    processor_type_ = PROCESS_NEAR_REGION;
  }

  LocalizedRangeScan* range_scan = getLocalizedRangeScan(
    laser, scan, karto_pose);

  // Add the localized range scan to the smapper
  boost::mutex::scoped_lock lock(smapper_mutex_);
  bool processed = false, update_reprocessing_transform = false;
  if (processor_type_ == PROCESS_NEAR_REGION)
  {
    if (!process_near_pose_)
    {
      ROS_ERROR("Process near region called without a "
        "valid region request. Ignoring scan.");
      return nullptr;
    }

    // set our position to the requested pose and process
    range_scan->SetOdometricPose(*process_near_pose_);
    range_scan->SetCorrectedPose(range_scan->GetOdometricPose());
    process_near_pose_.reset(nullptr);
    processed = smapper_->getMapper()->ProcessAgainstNodesNearBy(range_scan);

    // reset to localization mode
    processor_type_ = PROCESS_LOCALIZATION;
    update_reprocessing_transform = true;
    std::cout << "PROCESSOR TYPE IS NEAR REGION" << std::endl;
  }
  else if (processor_type_ == PROCESS_LOCALIZATION)
  {
    double* pScore;
    double score = 0.0;
    pScore = &score;
    processed = smapper_->getMapper()->ProcessLocalization(range_scan, pScore,odomOnly_);

    // Get the current estimated pose and its covariance and send it as a PoseWithCovariance msg
    LocalizationSlamToolbox::PublishEstimatedPose();

    std_msgs::Float64 score_msg;
    score_msg.data = (float)*pScore;
    if (score_msg.data != 0.0){
        score_scan_match_pub_.publish(score_msg);
    }

    update_reprocessing_transform = false;
  }
  else
  {
    ROS_FATAL("LocalizationSlamToolbox: "
      "No valid processor type set! Exiting.");
    exit(-1);
  }

  // if successfully processed, create odom to map transformation
  if(!processed)
  {
    delete range_scan;
    range_scan = nullptr;
  } else {
    // compute our new transform
    setTransformFromPoses(range_scan->GetCorrectedPose(), karto_pose,
      scan->header.stamp, update_reprocessing_transform);
  }

  return range_scan;
}

/*****************************************************************************/
void LocalizationSlamToolbox::localizePoseCallback(const
  geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
/*****************************************************************************/
{
  if (processor_type_ != PROCESS_LOCALIZATION)
  {
    ROS_ERROR("LocalizePoseCallback: Cannot process localization command "
      "if not in localization mode.");
    return;
  }

  boost::mutex::scoped_lock l(pose_mutex_);
  if (process_near_pose_)
  {
    process_near_pose_.reset(new Pose2(msg->pose.pose.position.x,
      msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation)));
  }
  else
  {
    process_near_pose_ = std::make_unique<Pose2>(msg->pose.pose.position.x,
      msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
  }

  first_measurement_ = true;

  ROS_INFO("LocalizePoseCallback: Localizing to: (%0.2f %0.2f), theta=%0.2f",
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    tf2::getYaw(msg->pose.pose.orientation));
  return;
}

/*****************************************************************************/
void LocalizationSlamToolbox::odomOnlyCallback(const
  std_msgs::Bool& msg)
/*****************************************************************************/
{
  ROS_INFO("Received msg %i",msg.data);
  odomOnly_ = msg.data;
}

/*****************************************************************************/
void LocalizationSlamToolbox::PublishEstimatedPose()
/*****************************************************************************/
{
  // Temporary variables
  Pose2 current_pose;
  Matrix3 current_cov;
  geometry_msgs::PoseWithCovarianceStamped best_pose_msg;
  geometry_msgs::Quaternion quat_msg;
  geometry_msgs::Point pt_msg; 
  float cov_msg[36];
  tf2::Quaternion quat;

  // Fill up header
  best_pose_msg.header.stamp = ros::Time::now();
  best_pose_msg.header.frame_id = frame_id_;

  // Get last best estimated pose and its covariance from scan matching
  current_pose = smapper_->getMapper()->getLastBestPose();
  current_cov = smapper_->getMapper()->getLastCovariance();

  // Convert from Karto::Pose2 to geometry_msgs::Pose
  //// Position
  pt_msg.x = float(current_pose.GetX());
  pt_msg.y = float(current_pose.GetY());
  best_pose_msg.pose.pose.position = pt_msg;
  //// Orientation
  quat.setRPY(0,0,current_pose.GetHeading());
  quat_msg = tf2::toMsg(quat);
  best_pose_msg.pose.pose.orientation = quat_msg;

  // Convert from Matrix3 to geometry_msgs::Quaternion
  best_pose_msg.pose.covariance[0] = current_cov(0,0);
  best_pose_msg.pose.covariance[7] = current_cov(1,1);
  best_pose_msg.pose.covariance[35] = current_cov(2,2);

  // Publish
  best_pose_pub_.publish(best_pose_msg);
}


} // end namespace
