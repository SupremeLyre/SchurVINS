#pragma once

#include <memory>
#include <thread>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <svo/common/camera_fwd.h>
#include <svo/common/transformation.h>
#include <svo/common/types.h>

namespace svo {

// forward declarations
class FrameHandlerBase;
class Visualizer;
class ImuHandler;
class BackendInterface;
class CeresBackendInterface;
class CeresBackendPublisher;

enum class PipelineType {
  kMono,
  kStereo,
  kArray
};

/// SVO Interface
class SvoInterface
{
public:

  using Image = sensor_msgs::msg::Image;
  using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
  using ImuConstPtr = sensor_msgs::msg::Imu::ConstSharedPtr;
  using StringConstPtr = std_msgs::msg::String::ConstSharedPtr;
  using ExactPolicy = message_filters::sync_policies::ExactTime<Image, Image>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;

  // ROS subscription and publishing.
  rclcpp::Node::SharedPtr node_;
  PipelineType pipeline_type_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_remote_key_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mono_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam0_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_cam1_;
  std::shared_ptr<ExactSync> stereo_sync_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr remote_key_callback_group_;
  std::string remote_input_;
  std::unique_ptr<std::thread> imu_thread_;
  std::unique_ptr<std::thread> image_thread_;

  // SVO modules.
  std::shared_ptr<FrameHandlerBase> svo_;
  std::shared_ptr<Visualizer> visualizer_;
  std::shared_ptr<ImuHandler> imu_handler_;
  std::shared_ptr<BackendInterface> backend_interface_;
  std::shared_ptr<CeresBackendInterface> ceres_backend_interface_;
  std::shared_ptr<CeresBackendPublisher> ceres_backend_publisher_;

  CameraBundlePtr ncam_;

  // Parameters
  bool set_initial_attitude_from_gravity_ = true;

  // System state.
  bool quit_ = false;
  bool idle_ = false;
  bool automatic_reinitialization_ = false;

  SvoInterface(const PipelineType& pipeline_type,
               const rclcpp::Node::SharedPtr& node);

  virtual ~SvoInterface();

  // Processing
  void processImageBundle(
      const std::vector<cv::Mat>& images,
      int64_t timestamp_nanoseconds);

  bool setImuPrior(const int64_t timestamp_nanoseconds);

  void publishResults(
      const std::vector<cv::Mat>& images,
      const int64_t timestamp_nanoseconds);

  // Subscription and callbacks
  void monoCallback(const ImageConstPtr& msg);
  void stereoCallback(
      const ImageConstPtr& msg0,
      const ImageConstPtr& msg1);
  void imuCallback(const ImuConstPtr& imu_msg);
  void inputKeyCallback(const StringConstPtr& key_input);


  // These functions are called before and after monoCallback or stereoCallback.
  // a derived class can implement some additional logic here.
  virtual void imageCallbackPreprocessing(int64_t timestamp_nanoseconds) {}
  virtual void imageCallbackPostprocessing() {}

  void subscribeImu();
  void subscribeImage();
  void subscribeRemoteKey();

  void imuLoop();
  void monoLoop();
  void stereoLoop();
};

} // namespace svo
