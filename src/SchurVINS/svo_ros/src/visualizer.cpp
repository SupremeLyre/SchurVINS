// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
// Modification Note:
// This file may have been modified by the authors of SchurVINS.
// (All authors of SchurVINS are with PICO department of ByteDance Corporation)
#include <svo_ros/visualizer.h>

#include <algorithm>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <svo_msgs/msg/feature.hpp>

#include <rpg_common/pose.h>
#include <svo/common/frame.h>
#include <svo/common/point.h>
#include <svo/common/seed.h>
#include <svo/direct/feature_detection_utils.h>
#include <svo/frame_handler_base.h>
#include <svo/img_align/sparse_img_align.h>
#include <svo/initialization.h>
#include <svo/map.h>
#include <svo/reprojector.h>
#include <svo/tracker/feature_tracking_utils.h>
#include <vikit/output_helper.h>
#include <vikit/params_helper.h>
#include <vikit/timer.h>

namespace
{
rclcpp::Time timeFromNs(const int64_t timestamp_nanoseconds)
{
  return rclcpp::Time(timestamp_nanoseconds, RCL_ROS_TIME);
}

geometry_msgs::msg::Point pointMsg(const Eigen::Vector3d& p)
{
  geometry_msgs::msg::Point msg;
  msg.x = p.x();
  msg.y = p.y();
  msg.z = p.z();
  return msg;
}

void fillPoseMsg(const svo::Transformation& T, geometry_msgs::msg::Pose* pose)
{
  const Eigen::Quaterniond q = T.getRotation().toImplementation();
  const Eigen::Vector3d p = T.getPosition();
  pose->position.x = p.x();
  pose->position.y = p.y();
  pose->position.z = p.z();
  pose->orientation.x = q.x();
  pose->orientation.y = q.y();
  pose->orientation.z = q.z();
  pose->orientation.w = q.w();
}

bool hasSubscribers(const rclcpp::PublisherBase::SharedPtr& pub)
{
  return pub && pub->get_subscription_count() > 0;
}

void publishPointCloud(
    const svo::Visualizer::PointCloud& pc,
    const svo::Visualizer::PointCloudPublisher& pub)
{
  if (!hasSubscribers(pub))
  {
    return;
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pc, msg);
  pub->publish(msg);
}
}  // namespace

namespace svo
{
std::string Visualizer::kWorldFrame = std::string("world");

Visualizer::Visualizer(const std::string& trace_dir,
                       const rclcpp::Node::SharedPtr& node,
                       const size_t n_cameras)
  : node_(node)
  , trace_dir_(trace_dir)
  , img_pub_level_(vk::param<int>(node_, "publish_img_pyr_level", 0))
  , img_pub_nth_(vk::param<int>(node_, "publish_every_nth_img", 1))
  , dense_pub_nth_(vk::param<int>(node_, "publish_every_nth_dense_input", 1))
  , viz_caption_str_(vk::param<bool>(node_, "publish_image_caption_str", false))
  , pc_(new PointCloud)
  , br_(std::make_unique<tf2_ros::TransformBroadcaster>(node_))
  , publish_world_in_cam_frame_(
        vk::param<bool>(node_, "publish_world_in_cam_frame", true))
  , publish_map_every_frame_(
        vk::param<bool>(node_, "publish_map_every_frame", false))
  , publish_points_display_time_(
        rclcpp::Duration::from_seconds(
          vk::param<double>(node_, "publish_point_display_time", 0.0)))
  , publish_seeds_(vk::param<bool>(node_, "publish_seeds", true))
  , publish_seeds_uncertainty_(
        vk::param<bool>(node_, "publish_seeds_uncertainty", false))
  , publish_active_keyframes_(
        vk::param<bool>(node_, "publish_active_kfs", false))
  , trace_pointcloud_(vk::param<bool>(node_, "trace_pointcloud", false))
  , vis_scale_(vk::param<double>(node_, "publish_marker_scale", 1.2))
{
  traj_path_ = trace_dir_ + "/imu_traj.txt";
  save_traj_.open(traj_path_);

  pub_frames_ = node_->create_publisher<Marker>("~/keyframes", 100);
  pub_points_ = node_->create_publisher<Marker>("~/points", 10000);
  pub_imu_pose_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "~/pose_imu", 10);
  pub_imu_path_ = node_->create_publisher<nav_msgs::msg::Path>("~/path_imu", 10);
  pub_vins_path_ = node_->create_publisher<nav_msgs::msg::Path>("~/path_vins", 10);
  pub_info_ = node_->create_publisher<svo_msgs::msg::Info>("~/info", 10);
  pub_markers_ = node_->create_publisher<Marker>("~/markers", 100);
  pub_pc_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/pointcloud", 1);

  pub_dense_.resize(n_cameras);
  pub_images_.resize(n_cameras);
  pub_cam_poses_.resize(n_cameras);
  for (size_t i = 0; i < n_cameras; ++i)
  {
    pub_dense_.at(i) =
        node_->create_publisher<svo_msgs::msg::DenseInputWithFeatures>(
            "~/dense_input/cam" + std::to_string(i), 2);
    pub_images_.at(i) =
        node_->create_publisher<sensor_msgs::msg::Image>(
            "~/image/cam" + std::to_string(i), 10);
    pub_cam_poses_.at(i) =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/pose_cam/cam" + std::to_string(i), 10);
  }

#ifdef SVO_LOOP_CLOSING
  pose_graph_map_.clear();
  pose_graph_map_.header.frame_id = kWorldFrame;
  pub_loop_closure_ = node_->create_publisher<Marker>("~/loop_closures", 10);
  pub_pose_graph_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/pose_graph", 10);
  pub_pose_graph_map_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/pose_graph_pointcloud", 10);
#endif

#ifdef SVO_GLOBAL_MAP
  pub_global_map_kfs_opt_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/global_map_kfs", 10);
  pub_global_map_query_kfs_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/global_map_query_kfs", 10);
  pub_global_map_pts_opt_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/global_map_pts", 10);
  pub_global_map_vis_ =
      node_->create_publisher<Marker>("~/global_map_all_vis", 10);
  pub_global_map_keypoints_vis_ =
      node_->create_publisher<Marker>("~/global_map_keypoints_vis", 10);
  pub_visible_fixed_landmarks_ =
      node_->create_publisher<Marker>("~/visible_fixed_landmarks", 10);
  pub_global_map_matched_points_ =
      node_->create_publisher<Marker>("~/global_map_matched", 10);
  pub_global_map_reobserved_points_ =
      node_->create_publisher<Marker>("~/global_map_reobserved", 10);
  pub_global_map_reobserved_points_frontend_ =
      node_->create_publisher<Marker>("~/global_map_reobserved_frontend", 10);
  pub_global_map_point_ids_ =
      node_->create_publisher<MarkerArray>("~/global_map_point_ids", 10);
#endif
}

void Visualizer::publishSvoInfo(const svo::FrameHandlerBase* const svo,
                                const int64_t timestamp_nanoseconds)
{
  CHECK_NOTNULL(svo);
  ++trace_id_;

  if (!hasSubscribers(pub_info_))
  {
    return;
  }

  svo_msgs::msg::Info msg_info;
  msg_info.header.frame_id = "cam";
  msg_info.header.stamp = timeFromNs(timestamp_nanoseconds);
  msg_info.processing_time = svo->lastProcessingTime();
  msg_info.stage = static_cast<int>(svo->stage());
  msg_info.tracking_quality = static_cast<int>(svo->trackingQuality());
  msg_info.num_matches = svo->lastNumObservations();
  pub_info_->publish(msg_info);
}

void Visualizer::publishImuPose(const Transformation& T_world_imu,
                                const Eigen::Matrix<double, 6, 6> covariance,
                                const int64_t timestamp_nanoseconds)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg_pose;
  msg_pose.header.stamp = timeFromNs(timestamp_nanoseconds);
  msg_pose.header.frame_id = kWorldFrame;
  fillPoseMsg(T_world_imu, &msg_pose.pose.pose);
  for (size_t i = 0; i < 36; ++i)
  {
    msg_pose.pose.covariance[i] = covariance(i % 6, i / 6);
  }
  pub_imu_pose_->publish(msg_pose);

  imu_path_.header = msg_pose.header;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = imu_path_.header;
  pose.pose = msg_pose.pose.pose;
  imu_path_.poses.push_back(pose);
  pub_imu_path_->publish(imu_path_);

  vins_path_.header = msg_pose.header;
  vins_path_.poses.push_back(pose);
  pub_vins_path_->publish(vins_path_);

  if (save_traj_.is_open())
  {
    const Eigen::Quaterniond q =
        T_world_imu.getRotation().toImplementation();
    const Eigen::Vector3d p = T_world_imu.getPosition();
    save_traj_ << std::fixed << std::setprecision(10)
               << 1e-9 * timestamp_nanoseconds << " "
               << p[0] << " " << p[1] << " " << p[2] << " "
               << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
               << std::endl;
  }
}

void Visualizer::publishCameraPoses(const FrameBundlePtr& frame_bundle,
                                    const int64_t timestamp_nanoseconds)
{
  const rclcpp::Time stamp = timeFromNs(timestamp_nanoseconds);
  vk::output_helper::publishTfTransform(
      frame_bundle->at(0)->T_cam_world(), stamp, "cam_pos", kWorldFrame, *br_);

  for (size_t i = 0; i < frame_bundle->size(); ++i)
  {
    if (!hasSubscribers(pub_cam_poses_.at(i)))
    {
      continue;
    }
    geometry_msgs::msg::PoseStamped msg_pose;
    msg_pose.header.stamp = stamp;
    msg_pose.header.frame_id = "cam" + std::to_string(i);
    fillPoseMsg(frame_bundle->at(i)->T_world_cam(), &msg_pose.pose);
    pub_cam_poses_.at(i)->publish(msg_pose);
  }
}

void Visualizer::publishBundleFeatureTracks(const FrameBundlePtr frames_ref,
                                            const FrameBundlePtr frames_cur,
                                            int64_t timestamp)
{
  if (trace_id_ % img_pub_nth_ != 0 || !frames_ref)
  {
    return;
  }

  for (size_t i = 0; i < frames_ref->size(); ++i)
  {
    std::vector<std::pair<size_t, size_t>> matches_ref_cur;
    feature_tracking_utils::getFeatureMatches(
        *frames_ref->at(i), *frames_cur->at(i), &matches_ref_cur);
    publishFeatureTracks(frames_ref->at(i)->px_vec_, frames_cur->at(i)->px_vec_,
                         matches_ref_cur, frames_cur->at(i)->img_pyr_,
                         img_pub_level_, timestamp, i);
  }
}

void Visualizer::publishFeatureTracks(
    const Keypoints& px_ref, const Keypoints& px_cur,
    const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
    const ImgPyr& img_pyr, const Level& level, const uint64_t timestamp,
    const size_t frame_index)
{
  if (!hasSubscribers(pub_images_.at(frame_index)))
  {
    return;
  }

  const int scale = (1 << level);
  cv::Mat img_rgb(img_pyr[level].size(), CV_8UC3);
  cv::cvtColor(img_pyr[level], img_rgb, cv::COLOR_GRAY2RGB);
  for (size_t i = 0; i < matches_ref_cur.size(); ++i)
  {
    const size_t i_ref = matches_ref_cur[i].first;
    const size_t i_cur = matches_ref_cur[i].second;
    cv::line(img_rgb,
             cv::Point2f(px_cur(0, i_cur) / scale, px_cur(1, i_cur) / scale),
             cv::Point2f(px_ref(0, i_ref) / scale, px_ref(1, i_ref) / scale),
             cv::Scalar(0, 255, 0), 2);
  }
  writeCaptionStr(img_rgb);
  cv_bridge::CvImage img_msg;
  img_msg.header.frame_id = "cam";
  img_msg.header.stamp = timeFromNs(static_cast<int64_t>(timestamp));
  img_msg.image = img_rgb;
  img_msg.encoding = sensor_msgs::image_encodings::BGR8;
  pub_images_.at(frame_index)->publish(*img_msg.toImageMsg());
}

void Visualizer::publishImages(const std::vector<cv::Mat>& images,
                               const int64_t timestamp_nanoseconds)
{
  if (trace_id_ % img_pub_nth_ != 0)
  {
    return;
  }

  for (size_t i = 0; i < images.size(); ++i)
  {
    if (!hasSubscribers(pub_images_.at(i)))
    {
      continue;
    }

    ImgPyr img_pyr;
    if (images[i].type() == CV_8UC1)
    {
      frame_utils::createImgPyramid(images[i], img_pub_level_ + 1, img_pyr);
    }
    else if (images[i].type() == CV_8UC3)
    {
      cv::Mat gray_image;
      cv::cvtColor(images[i], gray_image, cv::COLOR_BGR2GRAY);
      frame_utils::createImgPyramid(gray_image, img_pub_level_ + 1, img_pyr);
    }
    else
    {
      LOG(FATAL) << "Unknown image type " << images[i].type() << "!";
    }

    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = timeFromNs(timestamp_nanoseconds);
    img_msg.header.frame_id = "cam" + std::to_string(i);
    img_msg.image = img_pyr.at(img_pub_level_);
    img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    pub_images_.at(i)->publish(*img_msg.toImageMsg());
  }
}

void Visualizer::publishImagesWithFeatures(const FrameBundlePtr& frame_bundle,
                                           const int64_t timestamp,
                                           const bool draw_boundary)
{
  if (trace_id_ % img_pub_nth_ != 0)
  {
    return;
  }

  for (size_t i = 0; i < frame_bundle->size(); ++i)
  {
    if (!hasSubscribers(pub_images_.at(i)))
    {
      continue;
    }
    FramePtr frame = frame_bundle->at(i);
    cv::Mat img_rgb;
    feature_detection_utils::drawFeatures(*frame, img_pub_level_, true,
                                          &img_rgb);
    if (draw_boundary)
    {
      cv::rectangle(img_rgb, cv::Point2f(0.0, 0.0),
                    cv::Point2f(img_rgb.cols, img_rgb.rows),
                    cv::Scalar(0, 255, 0), 6);
    }
    writeCaptionStr(img_rgb);
    cv_bridge::CvImage img_msg;
    img_msg.header.frame_id = "cam";
    img_msg.header.stamp = timeFromNs(timestamp);
    img_msg.image = img_rgb;
    img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    pub_images_.at(i)->publish(*img_msg.toImageMsg());
  }
}

void Visualizer::visualizeHexacopter(const Transformation& T_frame_world,
                                     const uint64_t timestamp)
{
  const rclcpp::Time stamp = timeFromNs(static_cast<int64_t>(timestamp));
  vk::output_helper::publishTfTransform(T_frame_world, stamp, "cam_pos",
                                        kWorldFrame, *br_);
  if (hasSubscribers(pub_frames_))
  {
    vk::output_helper::publishCameraMarker(
        pub_frames_, "cam_pos", "cams", stamp, 1, 0, 0.8,
        Eigen::Vector3d(0., 0., 1.));
  }
}

void Visualizer::visualizeQuadrocopter(const Transformation& T_frame_world,
                                       const uint64_t timestamp)
{
  const rclcpp::Time stamp = timeFromNs(static_cast<int64_t>(timestamp));
  vk::output_helper::publishTfTransform(T_frame_world, stamp, "cam_pos",
                                        kWorldFrame, *br_);
  if (hasSubscribers(pub_frames_))
  {
    vk::output_helper::publishQuadrocopterMarkers(
        pub_frames_, "cam_pos", "cams", stamp, 1, 0, 0.8,
        Eigen::Vector3d(0., 0., 1.));
  }
}

void Visualizer::visualizeMarkers(const FrameBundlePtr& frame_bundle,
                                  const std::vector<FramePtr>& close_kfs,
                                  const Map::Ptr& map)
{
  FramePtr frame = frame_bundle->at(0);
  const uint64_t timestamp =
      static_cast<uint64_t>(frame_bundle->getMinTimestampNanoseconds());
  visualizeHexacopter(frame->T_f_w_, timestamp);
  if (hasSubscribers(pub_frames_))
  {
    const Transformation T_world_cam(frame->T_f_w_.inverse());
    vk::output_helper::publishFrameMarker(
        pub_frames_, T_world_cam.getRotationMatrix(), T_world_cam.getPosition(),
        "frame", node_->now(), 0, 0, 0.2 * vis_scale_);
  }
  publishTrajectoryPoint(frame->pos(), timestamp, static_cast<int>(trace_id_));
  if (frame->isKeyframe() || publish_map_every_frame_)
  {
    std::vector<FramePtr> frames_to_visualize = close_kfs;
    frames_to_visualize.push_back(frame);
    publishMapRegion(frames_to_visualize);
  }

  if (publish_seeds_)
  {
    publishSeeds(map);
  }
  if (publish_seeds_uncertainty_)
  {
    publishSeedsUncertainty(map);
  }
  if (publish_active_keyframes_)
  {
    std::vector<FramePtr> kfs_sorted;
    map->getSortedKeyframes(kfs_sorted);
    publishActiveKeyframes(kfs_sorted);
  }
}

void Visualizer::publishTrajectoryPoint(const Eigen::Vector3d& pos_in_vision,
                                        const uint64_t timestamp, const int id)
{
  if (hasSubscribers(pub_points_))
  {
    vk::output_helper::publishPointMarker(
        pub_points_, pos_in_vision, "trajectory",
        timeFromNs(static_cast<int64_t>(timestamp)), id, 0,
        0.5 * trajectory_marker_scale_ * vis_scale_,
        Eigen::Vector3d(0., 0., 0.5));
  }
}

void Visualizer::publishSeeds(const Map::Ptr& map)
{
  if (!hasSubscribers(pub_points_))
  {
    return;
  }

  Marker m;
  m.header.frame_id = kWorldFrame;
  m.header.stamp = node_->now();
  m.ns = "seeds";
  m.id = 0;
  m.type = Marker::POINTS;
  m.action = Marker::ADD;
  m.scale.x = seed_marker_scale_ * vis_scale_;
  m.scale.y = seed_marker_scale_ * vis_scale_;
  m.scale.z = seed_marker_scale_ * vis_scale_;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.pose.orientation.w = 1.0;
  m.points.reserve(1000);
  for (auto kf : map->keyframes_)
  {
    const FramePtr& frame = kf.second;
    const Transformation T_w_f = frame->T_world_cam();
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (isCornerEdgeletSeed(frame->type_vec_[i]))
      {
        CHECK(!frame->seed_ref_vec_[i].keyframe) << "Data inconsistent";
        m.points.push_back(pointMsg(T_w_f * frame->getSeedPosInFrame(i)));
      }
    }
  }
  pub_points_->publish(m);
}

void Visualizer::publishSeedsAsPointcloud(const Frame& frame,
                                          bool only_converged_seeds,
                                          bool reset_pc_before_publishing)
{
  if (!hasSubscribers(pub_pc_))
  {
    return;
  }

  if (reset_pc_before_publishing)
  {
    pc_->clear();
  }

  pc_->header.frame_id = kWorldFrame;
  pcl_conversions::toPCL(node_->now(), pc_->header.stamp);
  pc_->reserve(frame.num_features_);
  for (size_t i = 0; i < frame.num_features_; ++i)
  {
    if ((only_converged_seeds &&
         isConvergedCornerEdgeletSeed(frame.type_vec_.at(i))) ||
        !only_converged_seeds)
    {
      const Eigen::Vector3d xyz = frame.getSeedPosInFrame(i);
      PointType p;
      p.x = xyz.x();
      p.y = xyz.y();
      p.z = xyz.z();
      p.intensity =
          frame.img().at<uint8_t>(frame.px_vec_(1, i), frame.px_vec_(0, i));
      pc_->push_back(p);
    }
  }
  publishPointCloud(*pc_, pub_pc_);
}

void Visualizer::publishSeedsUncertainty(const Map::Ptr& map)
{
  if (!hasSubscribers(pub_points_))
  {
    return;
  }

  Marker msg_variance;
  msg_variance.header.frame_id = kWorldFrame;
  msg_variance.header.stamp = node_->now();
  msg_variance.ns = "seeds_variance";
  msg_variance.id = 0;
  msg_variance.type = Marker::LINE_LIST;
  msg_variance.action = Marker::ADD;
  msg_variance.scale.x = seed_uncertainty_marker_scale_ * vis_scale_;
  msg_variance.scale.y = seed_uncertainty_marker_scale_ * vis_scale_;
  msg_variance.scale.z = seed_uncertainty_marker_scale_ * vis_scale_;
  msg_variance.color.a = 1.0;
  msg_variance.color.r = 1.0;
  msg_variance.color.g = 0.0;
  msg_variance.color.b = 0.0;
  msg_variance.points.reserve(1000);
  for (auto kf : map->keyframes_)
  {
    const FramePtr& frame = kf.second;
    const Transformation T_w_f = frame->T_world_cam();
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (isCornerEdgeletSeed(frame->type_vec_[i]))
      {
        CHECK(!frame->seed_ref_vec_[i].keyframe) << "Data inconsistent";
        const FloatType z_inv_max =
            seed::getInvMaxDepth(frame->invmu_sigma2_a_b_vec_.col(i));
        const FloatType z_inv_min =
            seed::getInvMinDepth(frame->invmu_sigma2_a_b_vec_.col(i));
        const Eigen::Vector3d p1 =
            T_w_f * (frame->f_vec_.col(i) * (1.0 / z_inv_min));
        const Eigen::Vector3d p2 =
            T_w_f * (frame->f_vec_.col(i) * (1.0 / z_inv_max));
        msg_variance.points.push_back(pointMsg(p1));
        msg_variance.points.push_back(pointMsg(p2));
      }
    }
  }
  pub_points_->publish(msg_variance);
}

void Visualizer::publishMapRegion(const std::vector<FramePtr>& frames)
{
  const uint64_t ts = vk::Timer::getCurrentTime();
  const bool publish_keyframes = hasSubscribers(pub_frames_);
  const bool publish_points = hasSubscribers(pub_points_);
  if (hasSubscribers(pub_pc_))
  {
    PointCloud pc;
    pc.header.frame_id = kWorldFrame;
    pcl_conversions::toPCL(node_->now(), pc.header.stamp);
    pc.reserve(frames.size() * 150);
    for (const FramePtr& frame : frames)
    {
      for (size_t i = 0; i < frame->num_features_; ++i)
      {
        if (frame->landmark_vec_[i] == nullptr)
        {
          continue;
        }
        Point& point = *frame->landmark_vec_[i];
        if (point.last_published_ts_ == ts)
        {
          continue;
        }
        point.last_published_ts_ = ts;
        PointType p;
        p.x = point.pos_.x();
        p.y = point.pos_.y();
        p.z = point.pos_.z();
        p.intensity = isEdgelet(frame->type_vec_[i]) ? 60 : 0;
        pc.push_back(p);
      }
    }
    publishPointCloud(pc, pub_pc_);
  }

  if (publish_keyframes || publish_points)
  {
    uint64_t marker_ts = ts;
    for (const FramePtr& frame : frames)
    {
      publishKeyframeWithPoints(frame, ++marker_ts, point_marker_scale_);
    }
  }
}

void Visualizer::publishKeyframeWithPoints(const FramePtr& frame,
                                           const uint64_t timestamp,
                                           const double marker_scale)
{
  if (hasSubscribers(pub_frames_))
  {
    const Transformation T_world_cam(frame->T_f_w_.inverse());
    vk::output_helper::publishFrameMarker(
        pub_frames_, T_world_cam.getRotationMatrix(), T_world_cam.getPosition(),
        "kfs", node_->now(), frame->id_ * 10, 0, marker_scale * 2.0);
  }

  if (!hasSubscribers(pub_points_))
  {
    return;
  }

  Position xyz_world;
  int id = 0;
  for (size_t i = 0; i < frame->num_features_; ++i)
  {
    if (frame->landmark_vec_[i] != nullptr)
    {
      PointPtr& point = frame->landmark_vec_[i];
      if (point->last_published_ts_ == timestamp)
      {
        continue;
      }
      point->last_published_ts_ = timestamp;
      xyz_world = point->pos();
      id = point->id();
    }
    else if (frame->seed_ref_vec_[i].keyframe != nullptr)
    {
      const SeedRef& ref = frame->seed_ref_vec_[i];
      xyz_world = ref.keyframe->T_world_cam() *
                  ref.keyframe->getSeedPosInFrame(ref.seed_id);
      id = -ref.keyframe->id() * 1000 + ref.seed_id;
    }
    else
    {
      continue;
    }

    const Eigen::Vector3d color =
        isEdgelet(frame->type_vec_[i]) ? Eigen::Vector3d(0, 0.6, 0) :
                                         Eigen::Vector3d(1, 0, 1);
    vk::output_helper::publishPointMarker(
        pub_points_, xyz_world, "pts", node_->now(), id, 0,
        marker_scale * vis_scale_, color, publish_points_display_time_);

    if (trace_pointcloud_)
    {
      if (!ofs_pointcloud_.is_open())
      {
        ofs_pointcloud_.open(trace_dir_ + "/pointcloud.txt");
      }
      if (ofs_pointcloud_.is_open())
      {
        ofs_pointcloud_ << xyz_world.x() << " " << xyz_world.y() << " "
                        << xyz_world.z() << std::endl;
      }
    }
  }
}

void Visualizer::publishActiveKeyframes(const std::vector<FramePtr>& active_kfs)
{
  if (active_kfs.size() < 2)
  {
    return;
  }
  const std::string ns("active_kfs");
  for (size_t i = 0; i < active_kfs.size() - 1; i++)
  {
    vk::output_helper::publishLineMarker(
        pub_frames_, active_kfs[i]->pos(), active_kfs[i + 1]->pos(), ns,
        node_->now(), static_cast<int>(i), 0,
        0.8 * trajectory_marker_scale_ * vis_scale_,
        Eigen::Vector3d(.0, .0, 0.5));
  }
}

void Visualizer::exportToDense(const FrameBundlePtr& frame_bundle)
{
  for (size_t cam_index = 0; cam_index < frame_bundle->size(); ++cam_index)
  {
    if (dense_pub_nth_ == 0 || trace_id_ % dense_pub_nth_ != 0 ||
        !hasSubscribers(pub_dense_.at(cam_index)))
    {
      continue;
    }

    const FramePtr& frame = frame_bundle->at(cam_index);
    svo_msgs::msg::DenseInputWithFeatures msg;
    msg.header.stamp = timeFromNs(frame->getTimestampNSec());
    msg.header.frame_id = kWorldFrame;
    msg.frame_id = frame->id_;

    cv_bridge::CvImage img_msg;
    img_msg.header.stamp = msg.header.stamp;
    img_msg.header.frame_id = "camera";
    if (!frame->original_color_image_.empty())
    {
      img_msg.image = frame->original_color_image_;
      img_msg.encoding = sensor_msgs::image_encodings::BGR8;
    }
    else
    {
      img_msg.image = frame->img();
      img_msg.encoding = sensor_msgs::image_encodings::MONO8;
    }
    msg.image = *img_msg.toImageMsg();

    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();
    Position xyz_world;
    for (size_t i = 0; i < frame->num_features_; ++i)
    {
      if (frame->landmark_vec_[i] != nullptr)
      {
        xyz_world = frame->landmark_vec_[i]->pos();
      }
      else if (frame->seed_ref_vec_[i].keyframe != nullptr)
      {
        const SeedRef& ref = frame->seed_ref_vec_[i];
        xyz_world = ref.keyframe->T_world_cam() *
                    ref.keyframe->getSeedPosInFrame(ref.seed_id);
      }
      else
      {
        continue;
      }

      svo_msgs::msg::Feature feature;
      feature.x = xyz_world(0);
      feature.y = xyz_world(1);
      feature.z = xyz_world(2);
      msg.features.push_back(feature);

      const Position pos_in_frame = frame->T_f_w_ * xyz_world;
      min_z = std::min(pos_in_frame[2], min_z);
      max_z = std::max(pos_in_frame[2], max_z);
    }
    msg.min_depth = static_cast<float>(min_z);
    msg.max_depth = static_cast<float>(max_z);
    fillPoseMsg(frame->T_f_w_.inverse(), &msg.pose);
    pub_dense_.at(cam_index)->publish(msg);
  }
}

void Visualizer::visualizeCoordinateFrames(const Transformation& T_world_cam)
{
  if (!hasSubscribers(pub_markers_))
  {
    return;
  }
  vk::output_helper::publishFrameMarker(
      pub_markers_, T_world_cam.getRotationMatrix(), T_world_cam.getPosition(),
      "cam", node_->now(), 0, 0, 0.2);
  vk::output_helper::publishFrameMarker(
      pub_markers_, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
      kWorldFrame, node_->now(), 0, 0, 0.2);
}

void Visualizer::visualizeMarkersWithUncertainty(
    const FramePtr& frame, const std::vector<FramePtr>& close_kfs,
    const MapPtr& map, const float sigma_threshold)
{
  (void)frame;
  (void)close_kfs;
  (void)sigma_threshold;
  publishSeedsUncertainty(map);
}

void Visualizer::publishSeedsBinary(const MapPtr& map, const float sigma_threshold)
{
  (void)sigma_threshold;
  publishSeeds(map);
}

void Visualizer::publishVelocity(
    const Eigen::Vector3d& velocity_imu, const uint64_t timestamp)
{
  (void)velocity_imu;
  (void)timestamp;
}

void Visualizer::writeCaptionStr(cv::Mat img_rgb)
{
  if (viz_caption_str_)
  {
    cv::putText(img_rgb, img_caption_, cv::Point(20, 20),
                cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 250),
                1, cv::LINE_AA);
  }
}

#ifdef SVO_LOOP_CLOSING
void Visualizer::publishLoopClosureInfo(const LoopVizInfoVec& loop_viz_info,
                                        const std::string& ns,
                                        const Eigen::Vector3f& color,
                                        const double scale)
{
  (void)loop_viz_info;
  (void)ns;
  (void)color;
  (void)scale;
}

bool Visualizer::publishPoseGraph(const std::vector<KeyFramePtr>& kf_list,
                                  const bool redo_pointcloud,
                                  const size_t ignored_past_frames)
{
  (void)kf_list;
  (void)redo_pointcloud;
  (void)ignored_past_frames;
  return false;
}
#endif

#ifdef SVO_GLOBAL_MAP
void Visualizer::visualizeGlobalMap(const GlobalMap& map, const std::string ns,
                                    const Eigen::Vector3f& color,
                                    const double scale)
{
  (void)map;
  (void)ns;
  (void)color;
  (void)scale;
}

void Visualizer::visualizeFixedLandmarks(const FramePtr& frame)
{
  (void)frame;
}
#endif

}  // namespace svo
