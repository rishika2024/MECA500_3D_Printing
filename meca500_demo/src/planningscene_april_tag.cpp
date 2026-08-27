// Computes the print bed's real center + orientation in the robot's own base
// frame from the 4 corner AprilTags (tag0..tag3).
//
// Pipeline (deliberately split so it doesn't depend on chaining TF *up* through
// camera_color_optical_frame -- realsense2_camera also publishes that frame's
// parent, so link_0__base and the camera end up in two disconnected trees):
//   1. broadcast  -- for every tag apriltag currently sees, compose its
//                    camera_color_optical_frame->tag pose with the known camera
//                    pose in base, and broadcast link_0__base -> <tag>_base.
//   2. listen     -- read those <tag>_base frames back out of TF.
//   3. calc       -- fit the bed plane / centre from the base-frame tag points.
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{
Eigen::Isometry3d toEigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  q.normalize();
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = q.toRotationMatrix();
  T.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  return T;
}

geometry_msgs::msg::Transform fromEigen(const Eigen::Isometry3d & T)
{
  Eigen::Quaterniond q(T.rotation());
  q.normalize();
  geometry_msgs::msg::Transform t;
  t.translation.x = T.translation().x();
  t.translation.y = T.translation().y();
  t.translation.z = T.translation().z();
  t.rotation.x = q.x();
  t.rotation.y = q.y();
  t.rotation.z = q.z();
  t.rotation.w = q.w();
  return t;
}
}  // namespace

class Meca500PlanningScene : public rclcpp::Node
{
public:
  Meca500PlanningScene()
  : Node("meca500_planning_scene")
  {
    base_frame_ = this->declare_parameter("base_frame", std::string("link_0__base"));
    camera_frame_ = this->declare_parameter("camera_frame", std::string("camera_color_optical_frame"));
    tag_frames_ = this->declare_parameter("tag_frames",
      std::vector<std::string>{"tag0", "tag1", "tag2", "tag3"});
    expected_edge_ = this->declare_parameter("expected_edge", 0.205);

    // Camera pose in base_frame. The camera looks straight down but is mounted
    // rolled 180deg about its viewing axis (easy to miss -- apriltag detects
    // tags at any image rotation), so the optical->base axis map is
    // +X->+X, +Y->-Y, +Z->-Z: a 180deg rotation about X, quat xyzw (1,0,0,0).
    // Translation: Y from CAD (the 2020 profile L330 rod's URDF-known position,
    // ender3_environment.urdf, transformed into link_0__base); Z ~0.43-0.44m
    // above the base and X ~15cm off the rod edge, both measured on the real
    // hardware. This is what connects the camera's TF subtree to the robot's --
    // previously a static_transform_publisher in the launch, but that fought
    // realsense over camera_color_optical_frame's parent and split the tree, so
    // it lives here now and is applied in-process instead of via a TF chain.
    const auto ct = this->declare_parameter("camera_translation",
      std::vector<double>{-0.05, -0.1605, 0.435});
    const auto cq = this->declare_parameter("camera_rotation_xyzw",
      std::vector<double>{1.0, 0.0, 0.0, 0.0});
    Eigen::Quaterniond q(cq[3], cq[0], cq[1], cq[2]);
    q.normalize();
    T_base_cam_ = Eigen::Isometry3d::Identity();
    T_base_cam_.linear() = q.toRotationMatrix();
    T_base_cam_.translation() = Eigen::Vector3d(ct[0], ct[1], ct[2]);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    table_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "table_center_marker", 10);

    timer_ = this->create_wall_timer(200ms, [this]() { update(); });

    RCLCPP_INFO(this->get_logger(), "Bed-center-from-corner-tags node started.");
  }

private:
  std::string base_frame_;
  std::string camera_frame_;
  std::vector<std::string> tag_frames_;
  double expected_edge_;
  Eigen::Isometry3d T_base_cam_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr table_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Step 1: broadcast link_0__base -> <tag>_base for every tag apriltag sees.
  void broadcast_tags_in_base()
  {
    const rclcpp::Time stamp = this->get_clock()->now();
    for (const auto & tag : tag_frames_) {
      if (!tf_buffer_->canTransform(camera_frame_, tag, tf2::TimePointZero)) continue;
      geometry_msgs::msg::TransformStamped cam_to_tag;
      try {
        cam_to_tag = tf_buffer_->lookupTransform(camera_frame_, tag, tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        continue;
      }

      const Eigen::Isometry3d T_base_tag = T_base_cam_ * toEigen(cam_to_tag.transform);

      geometry_msgs::msg::TransformStamped out;
      out.header.stamp = stamp;
      out.header.frame_id = base_frame_;
      out.child_frame_id = tag + "_base";
      out.transform = fromEigen(T_base_tag);
      tf_broadcaster_->sendTransform(out);
    }
  }

  void update()
  {
    broadcast_tags_in_base();

    // Step 2: read the base-frame tag positions back out of TF. Indexed by
    // tag_frames_' own index so a tag dropping out mid-list doesn't shift the
    // entries after it.
    std::vector<bool> seen(tag_frames_.size(), false);
    std::vector<Eigen::Vector3d> by_index(tag_frames_.size(), Eigen::Vector3d::Zero());
    for (size_t i = 0; i < tag_frames_.size(); i++) {
      const std::string child = tag_frames_[i] + "_base";
      if (!tf_buffer_->canTransform(base_frame_, child, tf2::TimePointZero)) continue;
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform(base_frame_, child, tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        continue;
      }
      by_index[i] = Eigen::Vector3d(t.transform.translation.x,
        t.transform.translation.y, t.transform.translation.z);
      seen[i] = true;
      RCLCPP_INFO(this->get_logger(), "%s in %s: xyz=(%.4f, %.4f, %.4f)",
        child.c_str(), base_frame_.c_str(), by_index[i].x(), by_index[i].y(), by_index[i].z());
    }

    std::vector<Eigen::Vector3d> points;
    for (size_t i = 0; i < tag_frames_.size(); i++) {
      if (seen[i]) points.push_back(by_index[i]);
    }

    if (points.size() < 3) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Only %zu of %zu corner tags visible -- need at least 3 to fit the bed plane",
        points.size(), tag_frames_.size());
      return;
    }

    // Step 3: calc. Sanity-check whichever bed edges have both endpoints
    // visible against the real, measured bed (~0.205m each).
    for (size_t i = 0; i < tag_frames_.size(); i++) {
      size_t j = (i + 1) % tag_frames_.size();
      if (!seen[i] || !seen[j]) continue;
      double d = (by_index[j] - by_index[i]).norm();
      RCLCPP_INFO(this->get_logger(),
        "Edge %s-%s: measured=%.4fm (expected ~%.3fm, diff=%.4fm)",
        tag_frames_[i].c_str(), tag_frames_[j].c_str(), d, expected_edge_, d - expected_edge_);
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & p : points) centroid += p;
    centroid /= static_cast<double>(points.size());

    // Fit the bed plane via SVD of the centered points -- the normal is the
    // singular vector belonging to the smallest singular value.
    Eigen::MatrixXd centered(points.size(), 3);
    for (size_t i = 0; i < points.size(); i++) {
      centered.row(i) = (points[i] - centroid).transpose();
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeFullV);
    Eigen::Vector3d normal = svd.matrixV().col(2);
    if (normal.z() < 0) normal = -normal;  // keep pointing roughly up

    // Build a full orientation from just the normal: Z = plane normal, X =
    // base_frame's own X projected onto the plane, Y completes the frame.
    Eigen::Vector3d ref_x(1, 0, 0);
    Eigen::Vector3d x_axis = (ref_x - ref_x.dot(normal) * normal).normalized();
    Eigen::Vector3d y_axis = normal.cross(x_axis).normalized();
    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = normal;
    Eigen::Quaterniond q(R);

    RCLCPP_INFO(this->get_logger(),
      "Bed center (%zu tags): xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
      points.size(), centroid.x(), centroid.y(), centroid.z(), q.x(), q.y(), q.z(), q.w());

    visualization_msgs::msg::Marker table;
    table.header.frame_id = base_frame_;
    table.header.stamp = this->get_clock()->now();
    table.ns = "table_center";
    table.id = 0;
    table.type = visualization_msgs::msg::Marker::CUBE;
    table.action = visualization_msgs::msg::Marker::ADD;
    table.pose.position.x = centroid.x();
    table.pose.position.y = centroid.y();
    table.pose.position.z = centroid.z();
    table.pose.orientation.x = q.x();
    table.pose.orientation.y = q.y();
    table.pose.orientation.z = q.z();
    table.pose.orientation.w = q.w();
    table.scale.x = 0.200;
    table.scale.y = 0.220;
    table.scale.z = 0.001;
    table.color.r = 0.9f;
    table.color.g = 0.9f;
    table.color.b = 0.8f;
    table.color.a = 1.0f;
    table_marker_pub_->publish(table);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Meca500PlanningScene>());
  rclcpp::shutdown();
  return 0;
}
