// Modification Note: 
// This file may have been modified by the authors of SchurVINS.
// (All authors of SchurVINS are with PICO department of ByteDance Corporation)
#include <svo_ros/svo_interface.h>

#include <svo_ros/svo_factory.h>
#include <svo_ros/visualizer.h>
#include <svo/common/frame.h>
#include <svo/map.h>
#include <svo/imu_handler.h>
#include <svo/common/camera.h>
#include <svo/common/conversions.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/frame_handler_array.h>
#include <svo/initialization.h>
#include <svo/direct/depth_filter.h>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <vikit/params_helper.h>
#include <vikit/timer.h>
#include <svo_ros/ceres_backend_factory.h>

#include <functional>


#ifdef SVO_USE_GTSAM_BACKEND
#include <svo_ros/backend_factory.h>
#include <svo/backend/backend_interface.h>
#include <svo/backend/backend_optimizer.h>
#endif

#ifdef SVO_LOOP_CLOSING
#include <svo/online_loopclosing/loop_closing.h>
#endif

#ifdef SVO_GLOBAL_MAP
#include <svo/global_map.h>
#endif

namespace svo {

namespace {
int64_t stampToNanoseconds(const builtin_interfaces::msg::Time& stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

double stampToSeconds(const builtin_interfaces::msg::Time& stamp)
{
  return rclcpp::Time(stamp).seconds();
}
}  // namespace

SvoInterface::SvoInterface(
    const PipelineType& pipeline_type,
    const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , pipeline_type_(pipeline_type)
  , set_initial_attitude_from_gravity_(
      vk::param<bool>(node_, "set_initial_attitude_from_gravity", true))
  , automatic_reinitialization_(
      vk::param<bool>(node_, "automatic_reinitialization", false))
{
  imu_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  remote_key_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  switch (pipeline_type)
  {
    case PipelineType::kMono:
      svo_ = factory::makeMono(node_);
      break;
    case PipelineType::kStereo:
      svo_ = factory::makeStereo(node_);
      break;
    case PipelineType::kArray:
      svo_ = factory::makeArray(node_);
      break;
    default:
      LOG(FATAL) << "Unknown pipeline";
      break;
  }
  ncam_ = svo_->getNCamera();

  visualizer_.reset(
        new Visualizer(svo_->options_.trace_dir, node_, ncam_->getNumCameras()));

  if(vk::param<bool>(node_, "use_imu", false))
  {
    imu_handler_ = factory::getImuHandler(node_);
    svo_->setImuHandler(imu_handler_);
    // svo_->imu_handler_ = imu_handler_;
  }

  if(vk::param<bool>(node_, "use_ceres_backend", false))
  {
    ceres_backend_interface_ = ceres_backend_factory::makeBackend(node_,ncam_);
    if(imu_handler_){
      svo_->setBundleAdjuster(ceres_backend_interface_);
      ceres_backend_interface_->setImu(imu_handler_);
      ceres_backend_interface_->makePublisher(node_, ceres_backend_publisher_);
    }
    else
    {
      SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
    }
  }
#ifdef SVO_USE_GTSAM_BACKEND
  if(vk::param<bool>(node_, "use_backend", false))
  {
    backend_interface_ = svo::backend_factory::makeBackend(node_);
    ceres_backend_publisher_.reset(new CeresBackendPublisher(svo_->options_.trace_dir, node_));
    svo_->setBundleAdjuster(backend_interface_);
    backend_interface_->imu_handler_ = imu_handler_;
  }
#endif
  if(vk::param<bool>(node_, "runlc", false))
  {
#ifdef SVO_LOOP_CLOSING
    LoopClosingPtr loop_closing_ptr =
        factory::getLoopClosingModule(node_, svo_->getNCamera());
    svo_->lc_ = std::move(loop_closing_ptr);
    CHECK(svo_->depth_filter_->options_.extra_map_points)
        << "The depth filter seems to be initialized without extra map points.";
#else
    LOG(FATAL) << "You have to enable loop closing in svo_cmake.";
#endif
  }

  if(vk::param<bool>(node_, "use_global_map", false))
  {
#ifdef SVO_GLOBAL_MAP
    svo_->global_map_ = factory::getGlobalMap(node_, svo_->getNCamera());
    if (imu_handler_)
    {
      svo_->global_map_->initializeIMUParams(imu_handler_->imu_calib_,
                                             imu_handler_->imu_init_);
    }
#else
    LOG(FATAL) << "You have to enable global map in cmake";
#endif
  }

  svo_->start();
}

SvoInterface::~SvoInterface()
{
  if (imu_thread_)
    imu_thread_->join();
  if (image_thread_)
    image_thread_->join();
  VLOG(1) << "Destructed SVO.";
}

void SvoInterface::processImageBundle(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  if (!svo_->isBackendValid())
  {
    if (vk::param<bool>(node_, "use_ceres_backend", false, true))
    {
      ceres_backend_interface_ =
          ceres_backend_factory::makeBackend(node_, ncam_);
      if (imu_handler_)
      {
        svo_->setBundleAdjuster(ceres_backend_interface_);
        ceres_backend_interface_->setImu(imu_handler_);
        ceres_backend_interface_->makePublisher(node_, ceres_backend_publisher_);
      }
      else
      {
        SVO_ERROR_STREAM("Cannot use ceres backend without using imu");
      }
    }
  }
  svo_->addImageBundle(images, timestamp_nanoseconds);
}

void SvoInterface::publishResults(
    const std::vector<cv::Mat>& images,
    const int64_t timestamp_nanoseconds)
{
  CHECK_NOTNULL(svo_.get());
  CHECK_NOTNULL(visualizer_.get());

  visualizer_->img_caption_.clear();
  if (svo_->isBackendValid())
  {
    std::string static_str = ceres_backend_interface_->getStationaryStatusStr();
    visualizer_->img_caption_ = static_str;
  }

  visualizer_->publishSvoInfo(svo_.get(), timestamp_nanoseconds);
  switch (svo_->stage())
  {
    case Stage::kTracking: {
      Eigen::Matrix<double, 6, 6> covariance;
      covariance.setZero();
      visualizer_->publishImuPose(
            svo_->getLastFrames()->get_T_W_B(), covariance, timestamp_nanoseconds);
      visualizer_->publishCameraPoses(svo_->getLastFrames(), timestamp_nanoseconds);
      visualizer_->visualizeMarkers(
            svo_->getLastFrames(), svo_->closeKeyframes(), svo_->map());
      visualizer_->exportToDense(svo_->getLastFrames());
      bool draw_boundary = false;
      if (svo_->isBackendValid())
      {
        draw_boundary = svo_->getBundleAdjuster()->isFixedToGlobalMap();
      }
      visualizer_->publishImagesWithFeatures(
            svo_->getLastFrames(), timestamp_nanoseconds,
            draw_boundary);
#ifdef SVO_LOOP_CLOSING
      // detections
      if (svo_->lc_)
      {
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->cur_loop_check_viz_info_,
              std::string("loop_query"),
              Eigen::Vector3f(0.0f, 0.0f, 1.0f), 0.5);
        visualizer_->publishLoopClosureInfo(
              svo_->lc_->loop_detect_viz_info_, std::string("loop_detection"),
              Eigen::Vector3f(1.0f, 0.0f, 0.0f), 1.0);
        if (svo_->isBackendValid())
        {
          visualizer_->publishLoopClosureInfo(
                svo_->lc_->loop_correction_viz_info_,
                std::string("loop_correction"),
                Eigen::Vector3f(0.0f, 1.0f, 0.0f), 3.0);
        }
        if (svo_->getLastFrames()->at(0)->isKeyframe())
        {
          bool pc_recalculated = visualizer_->publishPoseGraph(
                svo_->lc_->kf_list_,
                svo_->lc_->need_to_update_pose_graph_viz_,
                static_cast<size_t>(svo_->lc_->options_.ignored_past_frames));
          if(pc_recalculated)
          {
            svo_->lc_->need_to_update_pose_graph_viz_ = false;
          }
        }
      }
#endif
#ifdef SVO_GLOBAL_MAP
      if (svo_->global_map_)
      {
        visualizer_->visualizeGlobalMap(*(svo_->global_map_),
                                        std::string("global_vis"),
                                        Eigen::Vector3f(0.0f, 0.0f, 1.0f),
                                        0.3);
        visualizer_->visualizeFixedLandmarks(svo_->getLastFrames()->at(0));
      }
#endif
      break;
    }
    case Stage::kInitializing: {
      visualizer_->publishBundleFeatureTracks(
            svo_->initializer_->frames_ref_, svo_->getLastFrames(),
            timestamp_nanoseconds);
      break;
    }
    case Stage::kPaused:
    case Stage::kRelocalization:
      visualizer_->publishImages(images, timestamp_nanoseconds);
      break;
    default:
      LOG(FATAL) << "Unknown stage";
      break;
  }

#ifdef SVO_USE_GTSAM_BACKEND
  if(svo_->stage() == Stage::kTracking && backend_interface_)
  {
    if(svo_->getLastFrames()->isKeyframe())
    {
      std::lock_guard<std::mutex> estimate_lock(backend_interface_->optimizer_->estimate_mut_);
      const gtsam::Values& state = backend_interface_->optimizer_->estimate_;
      ceres_backend_publisher_->visualizeFrames(state);
      if(backend_interface_->options_.add_imu_factors)
        ceres_backend_publisher_->visualizeVelocity(state);
      ceres_backend_publisher_->visualizePoints(state);
    }
  }
#endif
}

bool SvoInterface::setImuPrior(const int64_t timestamp_nanoseconds)
{
  if(svo_->getBundleAdjuster())
  {
    //if we use backend, this will take care of setting priors
    if(!svo_->hasStarted())
    {
      //when starting up, make sure we already have IMU measurements
      if(imu_handler_->getMeasurementsCopy().size() < 10u)
      {
        return false;
      }
    }
    return true;
  }

  if(imu_handler_ && !svo_->hasStarted() && set_initial_attitude_from_gravity_)
  {
    // set initial orientation
    Quaternion R_imu_world;
    if(imu_handler_->getInitialAttitude(
         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
         R_imu_world))
    {
      VLOG(3) << "Set initial orientation from accelerometer measurements.";
      svo_->setRotationPrior(R_imu_world);
    }
    else
    {
      return false;
    }
  }
  else if(imu_handler_ && svo_->getLastFrames())
  {
    // set incremental rotation prior
    Quaternion R_lastimu_newimu;
    if(imu_handler_->getRelativeRotationPrior(
         svo_->getLastFrames()->getMinTimestampNanoseconds() *
         common::conversions::kNanoSecondsToSeconds,
         timestamp_nanoseconds * common::conversions::kNanoSecondsToSeconds,
         false, R_lastimu_newimu))
    {
      VLOG(3) << "Set incremental rotation prior from IMU.";
      svo_->setRotationIncrementPrior(R_lastimu_newimu);
    }
  }
  return true;
}

void SvoInterface::monoCallback(const ImageConstPtr& msg)
{
  if(idle_)
    return;

  cv::Mat image;
  try
  {
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<cv::Mat> images;
  images.push_back(image.clone());

  const int64_t timestamp_ns = stampToNanoseconds(msg->header.stamp);
  if(!setImuPrior(timestamp_ns))
  {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(timestamp_ns);

  processImageBundle(images, timestamp_ns);


  publishResults(images, timestamp_ns);

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_)
    svo_->start();

  imageCallbackPostprocessing();
}

void SvoInterface::stereoCallback(
    const ImageConstPtr& msg0,
    const ImageConstPtr& msg1)
{
  if(idle_)
    return;

  cv::Mat img0, img1;
  try {
    img0 = cv_bridge::toCvShare(msg0, "mono8")->image;
    img1 = cv_bridge::toCvShare(msg1, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const int64_t timestamp_ns = stampToNanoseconds(msg0->header.stamp);
  if(!setImuPrior(timestamp_ns))
  {
    VLOG(3) << "Could not align gravity! Attempting again in next iteration.";
    return;
  }

  imageCallbackPreprocessing(timestamp_ns);

  processImageBundle({img0, img1}, timestamp_ns);
  publishResults({img0, img1}, timestamp_ns);

  if(svo_->stage() == Stage::kPaused && automatic_reinitialization_)
    svo_->start();

  imageCallbackPostprocessing();
}

void SvoInterface::imuCallback(const ImuConstPtr& msg)
{
  const Eigen::Vector3d omega_imu(
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  const Eigen::Vector3d lin_acc_imu(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  const ImuMeasurement m(stampToSeconds(msg->header.stamp), omega_imu, lin_acc_imu);
  if(imu_handler_)
    imu_handler_->addImuMeasurement(m);
  else
    SVO_ERROR_STREAM("SvoNode has no ImuHandler");
}

void SvoInterface::inputKeyCallback(const StringConstPtr& key_input)
{
  std::string remote_input = key_input->data;
  char input = remote_input.c_str()[0];
  switch(input)
  {
    case 'q':
      quit_ = true;
      SVO_INFO_STREAM("SVO user input: QUIT");
      rclcpp::shutdown();
      break;
    case 'r':
      svo_->reset();
      idle_ = true;
      SVO_INFO_STREAM("SVO user input: RESET");
      break;
    case 's':
      svo_->start();
      idle_ = false;
      SVO_INFO_STREAM("SVO user input: START");
      break;
     case 'c':
      svo_->setCompensation(true);
      SVO_INFO_STREAM("Enabled affine compensation.");
      break;
     case 'C':
      svo_->setCompensation(false);
      SVO_INFO_STREAM("Disabled affine compensation.");
      break;
    default: ;
  }
}

void SvoInterface::subscribeImu()
{
  std::string imu_topic = vk::param<std::string>(node_, "imu_topic", "imu");
  rclcpp::SubscriptionOptions options;
  options.callback_group = imu_callback_group_;
  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&svo::SvoInterface::imuCallback, this, std::placeholders::_1),
      options);
  SVO_INFO_STREAM("SvoNode: Subscribed IMU topic " << imu_topic);
}

void SvoInterface::subscribeImage()
{
  if(pipeline_type_ == PipelineType::kMono)
  {
    monoLoop();
  }
  else if(pipeline_type_ == PipelineType::kStereo)
  {
    stereoLoop();
  }
}

void SvoInterface::subscribeRemoteKey()
{
  std::string remote_key_topic =
      vk::param<std::string>(node_, "remote_key_topic", "svo/remote_key");
  rclcpp::SubscriptionOptions options;
  options.callback_group = remote_key_callback_group_;
  sub_remote_key_ =
      node_->create_subscription<std_msgs::msg::String>(
          remote_key_topic, 5,
          std::bind(&svo::SvoInterface::inputKeyCallback, this, std::placeholders::_1),
          options);
}

void SvoInterface::imuLoop()
{
  subscribeImu();
}

void SvoInterface::monoLoop()
{
  std::string image_topic =
      vk::param<std::string>(node_, "cam0_topic", "camera/image_raw");
  rclcpp::SubscriptionOptions options;
  options.callback_group = image_callback_group_;
  sub_mono_ = node_->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&svo::SvoInterface::monoCallback, this, std::placeholders::_1),
      options);
  SVO_INFO_STREAM("SvoNode: Subscribed image topic " << image_topic);
}

void SvoInterface::stereoLoop()
{
  // subscribe to cam msgs
  std::string cam0_topic(vk::param<std::string>(node_, "cam0_topic", "/cam0/image_raw"));
  std::string cam1_topic(vk::param<std::string>(node_, "cam1_topic", "/cam1/image_raw"));
  rclcpp::SubscriptionOptions options;
  options.callback_group = image_callback_group_;
  sub_cam0_.subscribe(node_, cam0_topic, rmw_qos_profile_sensor_data, options);
  sub_cam1_.subscribe(node_, cam1_topic, rmw_qos_profile_sensor_data, options);
  stereo_sync_ = std::make_shared<ExactSync>(ExactPolicy(1000), sub_cam0_, sub_cam1_);
  stereo_sync_->registerCallback(
      std::bind(&svo::SvoInterface::stereoCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  SVO_INFO_STREAM("SvoNode: Subscribed stereo topics " << cam0_topic << " and " << cam1_topic);
}

} // namespace svo
