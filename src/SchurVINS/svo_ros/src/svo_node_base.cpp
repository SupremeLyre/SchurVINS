// Modification Note:
// This file may have been modified by the authors of SchurVINS.
// (All authors of SchurVINS are with PICO department of ByteDance Corporation)
#include "svo_ros/svo_node_base.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <svo/common/logging.h>
#include <vikit/params_helper.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace svo_ros {

void SvoNodeBase::initThirdParty(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::SetStderrLogging(google::GLOG_ERROR);

  rclcpp::init(argc, argv);
}

SvoNodeBase::SvoNodeBase()
{
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>("svo", options);

  type_ = vk::param<bool>(node_, "pipeline_is_stereo", false) ?
      svo::PipelineType::kStereo : svo::PipelineType::kMono;
  svo_interface_ = std::make_unique<svo::SvoInterface>(type_, node_);

  if (svo_interface_->imu_handler_)
  {
    svo_interface_->subscribeImu();
  }
  svo_interface_->subscribeImage();
  svo_interface_->subscribeRemoteKey();
}

void SvoNodeBase::run()
{
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4);
  executor.add_node(node_);
  executor.spin();
  SVO_INFO_STREAM("SVO quit");
  if (svo_interface_)
  {
    svo_interface_->quit_ = true;
  }
  SVO_INFO_STREAM("SVO terminated.\n");
  rclcpp::shutdown();
}

}  // namespace svo_ros
