// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).

// Modification Note:
// This file may have been modified by the authors of SchurVINS.
// (All authors of SchurVINS are with PICO department of ByteDance Corporation)

#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <utility>

#include <Eigen/Core>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <svo_msgs/msg/dense_input_with_features.hpp>
#include <svo_msgs/msg/info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <svo/common/types.h>
#include <svo/global.h>

#ifdef SVO_LOOP_CLOSING
#include <svo/online_loopclosing/keyframe.h>
#include <svo/online_loopclosing/loop_closing.h>
#endif

#ifdef SVO_GLOBAL_MAP
#include <svo/global_map.h>
#endif

namespace svo
{
// forward declarations
class FrameHandlerBase;

/// Publish visualisation messages to ROS.
class Visualizer
{
public:
  typedef std::shared_ptr<Visualizer> Ptr;
  typedef pcl::PointXYZI PointType;
  typedef pcl::PointCloud<PointType> PointCloud;

  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using MarkerPublisher = rclcpp::Publisher<Marker>::SharedPtr;
  using MarkerArrayPublisher = rclcpp::Publisher<MarkerArray>::SharedPtr;
  using PosePublisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
  using PoseCovPublisher =
      rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr;
  using ImagePublisher = rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr;
  using PointCloudPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

  static std::string kWorldFrame;

  static constexpr double seed_marker_scale_ = 0.03;
  static constexpr double seed_uncertainty_marker_scale_ = 0.03;
  static constexpr double trajectory_marker_scale_ = 0.03;
  static constexpr double point_marker_scale_ = 0.05;

  rclcpp::Node::SharedPtr node_;
  size_t trace_id_ = 0;
  std::string trace_dir_;
  size_t img_pub_level_;
  size_t img_pub_nth_;
  size_t dense_pub_nth_;
  bool viz_caption_str_;

  std::string traj_path_;
  std::ofstream save_traj_;

  MarkerPublisher pub_frames_;
  MarkerPublisher pub_points_;
  PoseCovPublisher pub_imu_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_imu_path_;
  nav_msgs::msg::Path imu_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_vins_path_;
  nav_msgs::msg::Path vins_path_;
  rclcpp::Publisher<svo_msgs::msg::Info>::SharedPtr pub_info_;
  MarkerPublisher pub_markers_;
  PointCloudPublisher pub_pc_;
  PointCloud::Ptr pc_;
  std::vector<PosePublisher> pub_cam_poses_;
  std::vector<rclcpp::Publisher<svo_msgs::msg::DenseInputWithFeatures>::SharedPtr> pub_dense_;
  std::vector<ImagePublisher> pub_images_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  bool publish_world_in_cam_frame_;
  bool publish_map_every_frame_;
  rclcpp::Duration publish_points_display_time_;
  bool publish_seeds_;
  bool publish_seeds_uncertainty_;
  bool publish_active_keyframes_;
  bool trace_pointcloud_;
  double vis_scale_;
  std::ofstream ofs_pointcloud_;

#ifdef SVO_LOOP_CLOSING
  PointCloud pose_graph_map_;
  MarkerPublisher pub_loop_closure_;
  PointCloudPublisher pub_pose_graph_;
  PointCloudPublisher pub_pose_graph_map_;
#endif

#ifdef SVO_GLOBAL_MAP
  PointCloudPublisher pub_global_map_kfs_opt_;
  PointCloudPublisher pub_global_map_query_kfs_;
  PointCloudPublisher pub_global_map_pts_opt_;
  MarkerPublisher pub_global_map_vis_;
  MarkerPublisher pub_global_map_keypoints_vis_;
  MarkerPublisher pub_global_map_matched_points_;
  MarkerPublisher pub_global_map_reobserved_points_;
  MarkerPublisher pub_global_map_reobserved_points_frontend_;
  MarkerArrayPublisher pub_global_map_point_ids_;
#endif
  MarkerPublisher pub_visible_fixed_landmarks_;

  std::string img_caption_;

  Visualizer(const std::string& trace_dir, const rclcpp::Node::SharedPtr& node,
             const size_t num_cameras);

  ~Visualizer() = default;

  void publishSvoInfo(const svo::FrameHandlerBase* const svo,
                      const int64_t timestamp_nanoseconds);

  void publishImages(const std::vector<cv::Mat>& images,
                     const int64_t timestamp_nanoseconds);

  void publishImagesWithFeatures(const FrameBundlePtr& frame_bundle,
                                 const int64_t timestamp,
                                 const bool draw_boundary);

  void publishImuPose(const Transformation& T_world_imu,
                      const Eigen::Matrix<double, 6, 6> Covariance,
                      const int64_t timestamp_nanoseconds);

  void publishCameraPoses(const FrameBundlePtr& frame_bundle,
                          const int64_t timestamp_nanoseconds);

  void publishBundleFeatureTracks(const FrameBundlePtr frames_ref,
                                  const FrameBundlePtr frames_cur,
                                  int64_t timestamp);

  void publishFeatureTracks(
      const Keypoints& px_ref, const Keypoints& px_cur,
      const std::vector<std::pair<size_t, size_t>>& matches_ref_cur,
      const ImgPyr& img_pyr, const Level& level, const uint64_t timestamp,
      const size_t frame_index);

  void visualizeHexacopter(const Transformation& T_frame_world,
                           const uint64_t timestamp);

  void visualizeQuadrocopter(const Transformation& T_frame_world,
                             const uint64_t timestamp);

  void visualizeMarkers(const FrameBundlePtr& frame_bundle,
                        const std::vector<FramePtr>& close_kfs,
                        const MapPtr& map);

  void publishTrajectoryPoint(const Eigen::Vector3d& pos_in_vision,
                              const uint64_t timestamp, const int id);

  void visualizeMarkersWithUncertainty(const FramePtr& frame,
                                       const std::vector<FramePtr>& close_kfs,
                                       const MapPtr& map,
                                       const float sigma_threshold);

  void publishSeedsBinary(const MapPtr& map, const float sigma_threshold);

  void publishSeeds(const MapPtr& map);

  void publishSeedsAsPointcloud(const Frame& frame, bool only_converged_seeds,
                                bool reset_pc_before_publishing = true);

  void publishVelocity(const Eigen::Vector3d& velocity_imu,
                       const uint64_t timestamp);

  void publishMapRegion(const std::vector<FramePtr>& frames);

  void publishKeyframeWithPoints(const FramePtr& frame,
                                 const uint64_t timestamp,
                                 const double marker_scale = 0.05);

  void publishActiveKeyframes(const std::vector<FramePtr>& active_kfs);

  void exportToDense(const FrameBundlePtr& frame_bundle);

  void publishSeedsUncertainty(const MapPtr& map);

  void visualizeCoordinateFrames(const Transformation& T_world_cam);

#ifdef SVO_LOOP_CLOSING
  void publishLoopClosureInfo(
      const LoopVizInfoVec& loop_viz_info_vec,
      const std::string& ns, const Eigen::Vector3f& color,
      const double scale=1.0);

  bool publishPoseGraph(const std::vector<KeyFramePtr>& kf_list,
                        const bool redo_pointcloud,
                        const size_t ignored_past_frames);
#endif

#ifdef SVO_GLOBAL_MAP
  void visualizeGlobalMap(const GlobalMap& gmap,
                          const std::string ns,
                          const Eigen::Vector3f& color,
                          const double scale);
  void visualizeFixedLandmarks(const FramePtr& frame);
#endif
  void writeCaptionStr(cv::Mat img);
};

}  // end namespace svo
