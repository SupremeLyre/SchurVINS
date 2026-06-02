/*
 * output_helper.cpp
 *
 *  Created on: Jan 20, 2013
 *      Author: chrigi
 */

#include <complex>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <glog/logging.h>
#include <vikit/output_helper.h>

namespace vk {
namespace output_helper {

void publishTfTransform(
    const Transformation& T,
    const rclcpp::Time& stamp,
    const string& frame_id,
    const string& child_frame_id,
    tf2_ros::TransformBroadcaster& br)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  const Eigen::Quaterniond& q = T.getRotation().toImplementation();
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = frame_id;
  transform_msg.child_frame_id = child_frame_id;
  transform_msg.transform.translation.x = T.getPosition().x();
  transform_msg.transform.translation.y = T.getPosition().y();
  transform_msg.transform.translation.z = T.getPosition().z();
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();
  br.sendTransform(transform_msg);
}

void publishPointMarker(
    const MarkerPublisher& pub,
    const Vector3d& pos,
    const string& ns,
    const rclcpp::Time& timestamp,
    int id,
    int action,
    double marker_scale,
    const Vector3d& color,
    rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = timestamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::CUBE;
  msg.action = action; // 0 = add/modify
  msg.scale.x = marker_scale;
  msg.scale.y = marker_scale;
  msg.scale.z = marker_scale;
  msg.color.a = 1.0;
  msg.color.r = color[0];
  msg.color.g = color[1];
  msg.color.b = color[2];
  msg.lifetime = lifetime;
  msg.pose.position.x = pos[0];
  msg.pose.position.y = pos[1];
  msg.pose.position.z = pos[2];
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  pub->publish(msg);
}

void
publishLineMarker(const MarkerPublisher& pub,
                  const Vector3d& start,
                  const Vector3d& end,
                  const string& ns,
                  const rclcpp::Time& timestamp,
                  int id,
                  int action,
                  double marker_scale,
                  const Vector3d& color,
                  rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = timestamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
  msg.action = action; // 0 = add/modify
  msg.scale.x = marker_scale;
  msg.color.a = 1.0;
  msg.color.r = color[0];
  msg.color.g = color[1];
  msg.color.b = color[2];
  msg.points.resize(2);
  msg.lifetime = lifetime;
  msg.points[0].x = start[0];
  msg.points[0].y = start[1];
  msg.points[0].z = start[2];
  msg.points[1].x = end[0];
  msg.points[1].y = end[1];
  msg.points[1].z = end[2];
  pub->publish(msg);
}


void
publishArrowMarker(const MarkerPublisher& pub,
                   const Vector3d& pos,
                   const Vector3d& dir,
                   double scale,
                   const string& ns,
                   const rclcpp::Time& timestamp,
                   int id,
                   int action,
                   double marker_scale,
                   const Vector3d& color)
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = "world";
  msg.header.stamp = timestamp;
  msg.ns = ns;
  msg.id = id;
  msg.type = visualization_msgs::msg::Marker::ARROW;
  msg.action = action; // 0 = add/modify
  msg.scale.x = marker_scale;
  msg.scale.y = marker_scale*0.35;
  msg.scale.z = 0.0;
  msg.color.a = 1.0;
  msg.color.r = color[0];
  msg.color.g = color[1];
  msg.color.b = color[2];
  msg.points.resize(2);
  msg.points[0].x = pos[0];
  msg.points[0].y = pos[1];
  msg.points[0].z = pos[2];
  msg.points[1].x = pos[0] + scale*dir[0];
  msg.points[1].y = pos[1] + scale*dir[1];
  msg.points[1].z = pos[2] + scale*dir[2];
  pub->publish(msg);
}

void
publishHexacopterMarker(const MarkerPublisher& pub,
                        const string& frame_id,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        const Vector3d& color)
{
  /*
   * Function by Markus Achtelik from libsfly_viz.
   * Thank you.
   */
  const double sqrt2_2 = sqrt(2) / 2;

  visualization_msgs::msg::Marker marker;

  // the marker will be displayed in frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = action;
  marker.id = id;
  marker.frame_locked = true;

  // make rotors
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = 0.2*marker_scale;
  marker.scale.y = 0.2*marker_scale;
  marker.scale.z = 0.01*marker_scale;
  marker.color.r = 0.4;
  marker.color.g = 0.4;
  marker.color.b = 0.4;
  marker.color.a = 0.8;
  marker.pose.position.z = 0;

  // front left/right
  marker.pose.position.x = 0.19*marker_scale;
  marker.pose.position.y = 0.11*marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = 0.19*marker_scale;
  marker.pose.position.y = -0.11*marker_scale;
  marker.id--;
  pub->publish(marker);

  // left/right
  marker.pose.position.x = 0;
  marker.pose.position.y = 0.22*marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = 0;
  marker.pose.position.y = -0.22*marker_scale;
  marker.id--;
  pub->publish(marker);

  // back left/right
  marker.pose.position.x = -0.19*marker_scale;
  marker.pose.position.y = 0.11*marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = -0.19*marker_scale;
  marker.pose.position.y = -0.11*marker_scale;
  marker.id--;
  pub->publish(marker);

  // make arms
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.44*marker_scale;
  marker.scale.y = 0.02*marker_scale;
  marker.scale.z = 0.01*marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = -0.015*marker_scale;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;

  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  pub->publish(marker);

  // 30 deg rotation  0.9659  0  0  0.2588
  marker.pose.orientation.w = 0.9659;
  marker.pose.orientation.z = 0.2588;
  marker.id--;
  pub->publish(marker);

  marker.pose.orientation.w = 0.9659;
  marker.pose.orientation.z = -0.2588;
  marker.id--;
  pub->publish(marker);
}

void publishQuadrocopterMarkers(const MarkerPublisher& pub,
                        const string& frame_id,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        const Vector3d& color)
{
  /*
   * Function by Markus Achtelik from libsfly_viz.
   * Thank you.
   */
  const double sqrt2_2 = sqrt(2) / 2;

  visualization_msgs::msg::Marker marker;

  // the marker will be displayed in frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = action;
  marker.id = id;
  marker.frame_locked = true;

  // make rotors
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = 0.2*marker_scale;
  marker.scale.y = 0.2*marker_scale;
  marker.scale.z = 0.01*marker_scale;
  marker.color.r = 0.4;
  marker.color.g = 0.4;
  marker.color.b = 0.4;
  marker.color.a = 0.8;
  marker.pose.position.z = 0;

  // front
  marker.pose.position.x = 0.22*marker_scale;
  marker.pose.position.y = 0.0;
  marker.id--;
  pub->publish(marker);

  // left/right
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.22*marker_scale;
  marker.id--;
  pub->publish(marker);

  marker.pose.position.x = 0.0;
  marker.pose.position.y = -0.22*marker_scale;
  marker.id--;
  pub->publish(marker);

  // back
  marker.pose.position.x = -0.22*marker_scale;
  marker.pose.position.y = 0.0*marker_scale;
  marker.id--;
  pub->publish(marker);

  // make arms
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.44*marker_scale;
  marker.scale.y = 0.02*marker_scale;
  marker.scale.z = 0.01*marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = -0.015*marker_scale;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;

  marker.pose.orientation.w = sqrt2_2;
  marker.pose.orientation.z = sqrt2_2;
  marker.id--;
  pub->publish(marker);

  // 0 deg rotation
  marker.pose.orientation.w = 0;
  marker.pose.orientation.z = 1;
  marker.id--;
  pub->publish(marker);
}

void
publishCameraMarker(const MarkerPublisher& pub,
                    const string& frame_id,
                    const string& ns,
                    const rclcpp::Time& timestamp,
                    int id, int action,
                    double marker_scale,
                    const Vector3d& color)
{
  /*
   * draw a pyramid as the camera marker
   */
  visualization_msgs::msg::Marker marker;

  // the marker will be displayed in frame_id
  marker.header.frame_id = frame_id;
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.action = action;
  marker.id = id;
  marker.frame_locked = true;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.scale.x = 0.04 * marker_scale;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1;
  marker.pose.orientation.w = 1;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header = marker.header;
  delete_marker.ns = ns;
  delete_marker.action = visualization_msgs::msg::Marker::DELETE;
  for (int old_id = id - 1; old_id >= id - 8; --old_id)
  {
    delete_marker.id = old_id;
    pub->publish(delete_marker);
  }

  const double half_width = 0.5 * marker_scale;
  const double half_height = 0.25 * marker_scale;
  const double z_plane = 0.5 * marker_scale;
  const geometry_msgs::msg::Point origin;
  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;
  geometry_msgs::msg::Point p2;
  geometry_msgs::msg::Point p3;
  p0.x = half_width;  p0.y = half_height;  p0.z = z_plane;
  p1.x = half_width;  p1.y = -half_height; p1.z = z_plane;
  p2.x = -half_width; p2.y = -half_height; p2.z = z_plane;
  p3.x = -half_width; p3.y = half_height;  p3.z = z_plane;

  auto add_line = [&marker](const geometry_msgs::msg::Point& start,
                            const geometry_msgs::msg::Point& end) {
    marker.points.push_back(start);
    marker.points.push_back(end);
  };
  add_line(p0, p1);
  add_line(p1, p2);
  add_line(p2, p3);
  add_line(p3, p0);
  add_line(origin, p0);
  add_line(origin, p1);
  add_line(origin, p2);
  add_line(origin, p3);
  pub->publish(marker);
}

void publishFrameMarker(const MarkerPublisher& pub,
                        const Matrix3d& rot,
                        const Vector3d& pos,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        rclcpp::Duration lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = timestamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = action; // 0 = add/modify
  marker.pose.orientation.w = 1.0;
  marker.points.reserve(6);
  marker.colors.reserve(6);
  marker.scale.x = 0.1*marker_scale;
  marker.color.a = 1.0;
  marker.lifetime = lifetime;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header = marker.header;
  delete_marker.ns = ns;
  delete_marker.action = visualization_msgs::msg::Marker::DELETE;
  delete_marker.id = id + 1;
  pub->publish(delete_marker);
  delete_marker.id = id + 2;
  pub->publish(delete_marker);

  geometry_msgs::msg::Point origin;
  origin.x = pos.x();
  origin.y = pos.y();
  origin.z = pos.z();

  auto add_axis = [&marker, &origin, &pos, &rot, marker_scale](
      int axis, double r, double g, double b) {
    geometry_msgs::msg::Point end;
    end.x = pos.x() + marker_scale * rot(0, axis);
    end.y = pos.y() + marker_scale * rot(1, axis);
    end.z = pos.z() + marker_scale * rot(2, axis);
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
    marker.points.push_back(origin);
    marker.points.push_back(end);
    marker.colors.push_back(color);
    marker.colors.push_back(color);
  };

  add_axis(0, 1.0, 0.0, 0.0);
  add_axis(1, 0.0, 1.0, 0.0);
  add_axis(2, 0.0, 0.0, 1.0);
  pub->publish(marker);
}

void publishGtsamPoseCovariance(
    const MarkerPublisher& pub,
    const Eigen::Vector3d& mean,
    const Eigen::Matrix3d& R_W_B, // Body in World-Frame
    const Eigen::Matrix<double, 6, 6>& covariance,
    const string& ns,
    int id,
    int action,
    double sigma_scale,
    const Vector4d& color)
{
  // Gtsam performs the translational perturbation as follows:
  // W_p_WB <- W_p_WB + R_WB * B_pert
  // since the perturbation is in the body frame, also the translational
  // covariance part is in the body frame. therefore, we will rotate
  // the covariance matrix accordingly to the world-frame.


  /*
  Eigen::EigenSolver<Eigen::Matrix3d> es(covariance);
  const std::complex<double> lambda_1 = es.eigenvalues()[0];
  const std::complex<double> lambda_2 = es.eigenvalues()[1];
  const std::complex<double> lambda_3 = es.eigenvalues()[2];
  CHECK_DOUBLE_EQ(std::imag(lambda_1), 0.0);
  CHECK_DOUBLE_EQ(std::imag(lambda_2), 0.0);
  CHECK_DOUBLE_EQ(std::imag(lambda_3), 0.0);
  auto R_complex = es.eigenvectors();
  VLOG(20) << "Eigenvalues are " << lambda_1 << ", " << lambda_2 << ", " << lambda_3;
  Eigen::Matrix3d R = R_complex.real();
  R.col(1) = -R.col(1);
  R = R.array().rowwise() / R.colwise().norm().array(); // normalize
  CHECK_DOUBLE_EQ(R.determinant(), 1.0);
  Eigen::Quaterniond q(R);
  */

  const Matrix3d pose_covariance = covariance.bottomRightCorner(3,3);
  Eigen::JacobiSVD<Matrix3d> svd(pose_covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto S = svd.singularValues();
  VLOG(20) << "Eigenvalues are " << S(0) << ", " << S(1) << ", " << S(2);
  Eigen::Matrix3d R_body = svd.matrixU();
  if(R_body.determinant() < 0.0)
  {
    LOG(ERROR) << "Can't plot covariance matrix.";
    return;
  }
  Eigen::Quaterniond q_world(R_W_B * R_body);

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = rclcpp::Time(0, 0);
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = action; // add/modify
//  m.scale.x = std::real(lambda_1)*sigma_scale;
//  m.scale.y = std::real(lambda_2)*sigma_scale;
//  m.scale.z = std::real(lambda_3)*sigma_scale;
  m.scale.x = S(0) * sigma_scale;
  m.scale.y = S(1) * sigma_scale;
  m.scale.z = S(2) * sigma_scale;
  m.color.r = color(1);
  m.color.g = color(2);
  m.color.b = color(3);
  m.color.a = color(3);
  m.pose.position.x = mean.x();
  m.pose.position.y = mean.y();
  m.pose.position.z = mean.z();
  m.pose.orientation.x = q_world.x();
  m.pose.orientation.y = q_world.y();
  m.pose.orientation.z = q_world.z();
  m.pose.orientation.w = q_world.w();
  pub->publish(m);
}

} // namespace output_helper
} // namespace vk
