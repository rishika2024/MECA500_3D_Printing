// bed_from_touches -- work out the print bed's pose and push it onto
// planningscene's /table_service, once, then exit. Runs as the first step of
// the print pipeline (before reachability + gcode). All inputs come from
// machine_settings.yaml / bed_settings.yaml (loaded via `<param from>` in the launch files):
//
//   default_bed          true  -> use default_bed_pose as-is.
//                        false -> fit the bed plane from bed_touch_poses.
//   default_bed_pose     [x y z qx qy qz qw] in link_0__base.
//   bed_touch_poses      /joint_states 'position' vectors (rad, j1..j6) with
//                        the nozzle touching the bed, flattened 6-at-a-time.
//                        Need >= 3 poses when default_bed is false.
//   bed_center_pose      optional single [j1..j6] at the bed centre -> its
//                        nozzle tip is the reported xyz. Empty -> use the
//                        centroid of bed_touch_poses.
//   use_extruder + tip_offset_with/without_extruder_m  pick the nozzle-frame
//                        -> contact-point offset applied down the tool +Z.
//
// The FK chain (link_0__base -> nozzle) is transcribed from meca500.urdf +
// ender3_extruder.urdf; all rotation / linear-algebra work is done with Eigen.
#include <rclcpp/rclcpp.hpp>

#include "msr_meca500_print_pipeline/srv/table.hpp"

#include <Eigen/Dense>

#include <array>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;
using namespace std::chrono_literals;

using Joints = std::array<double, 6>;

// Fallback nozzle-frame -> tip offset (m) if the config doesn't set it.
static constexpr double kTipOffsetM = 0.008;

// --- forward kinematics: link_0__base -> nozzle frame ----------------------
// Joint origins/axes from msr_meca500_robot/urdf/meca500.urdf (joint1..6) and the
// nozzle fixed joint from ender3_extruder.urdf; checked against a live tf2_echo
// of link_0__base -> nozzle to ~1mm. Every joint axis is (0,0,-1).
namespace {

struct JointDef {
  Vector3d xyz;
  double roll, pitch, yaw;   // URDF fixed-axis XYZ
  Vector3d axis;
};

// URDF <origin rpy="r p y"> => R = Rz(y) * Ry(p) * Rx(r)
Matrix3d rpy(double r, double p, double y) {
  return (AngleAxisd(y, Vector3d::UnitZ()) *
          AngleAxisd(p, Vector3d::UnitY()) *
          AngleAxisd(r, Vector3d::UnitX())).toRotationMatrix();
}

Isometry3d fixed_tf(const Vector3d& xyz, const Matrix3d& R) {
  Isometry3d T = Isometry3d::Identity();
  T.linear() = R;
  T.translation() = xyz;
  return T;
}

const std::array<JointDef, 6> kJoints = {{
  {{ 0, 0, 0 },                                    0,           0,          0,          {0, 0, -1}},
  {{ 0.0325, -2.8283679e-07, 0.0445 },             1.5707963,   3.6732051e-06, -1.5708,  {0, 0, -1}},
  {{ -3.5823462e-07, 0.135, 0 },                   0,           0,         -3.14159,     {0, 0, -1}},
  {{ -0.060999884, -0.038000084, 0.031500123 },   -0.62561375,  1.5707918,  2.5159789,   {0, 0, -1}},
  {{ -0.023999783, 4.4e-06, 0.059000088 },         3.1415927,   1.5707927,  3.1415927,   {0, 0, -1}},
  {{ -0.059000089, 2.2e-06, 0.024249783 },         3.1415927,  -1.5707927,  3.1415927,   {0, 0, -1}},
}};

const Isometry3d kNozzleTf =
    fixed_tf(Vector3d(-2.0041237e-07, 0.025000277, 0.075524908), Matrix3d::Identity());

Isometry3d fk_nozzle(const Joints& q) {
  Isometry3d T = Isometry3d::Identity();
  for (size_t i = 0; i < kJoints.size(); ++i) {
    const JointDef& j = kJoints[i];
    Isometry3d joint_rot = Isometry3d::Identity();
    joint_rot.linear() = AngleAxisd(q[i], j.axis.normalized()).toRotationMatrix();
    T = T * fixed_tf(j.xyz, rpy(j.roll, j.pitch, j.yaw)) * joint_rot;
  }
  return T * kNozzleTf;
}

Vector3d nozzle_tip(const Joints& q, double tip_offset_m) {
  Isometry3d T = fk_nozzle(q);
  return T.translation() + T.linear() * Vector3d(0.0, 0.0, tip_offset_m);
}

// Un-flatten a 6*N param array into N joint vectors.
std::vector<Joints> reshape6(const std::vector<double>& flat) {
  std::vector<Joints> out;
  for (size_t i = 0; i + 6 <= flat.size(); i += 6) {
    Joints j{};
    std::copy_n(flat.begin() + i, 6, j.begin());
    out.push_back(j);
  }
  return out;
}

}  // namespace

// --- bed fit --------------------------------------------------------------
struct BedFit {
  Vector3d center;
  Quaterniond quat;
  Vector3d normal;
  double tilt_deg;
  double rms_mm;
  double max_off_mm;
  std::vector<double> per_touch_off_mm;
  double center_off_plane_mm;
  bool center_from_pose;
};

BedFit bed_pose_from_touches(const std::vector<Joints>& touches,
                             const std::optional<Joints>& bed_center = std::nullopt,
                             double tip_offset_m = kTipOffsetM) {
  if (touches.size() < 3) {
    throw std::runtime_error("need >= 3 touch poses to fit a plane");
  }

  MatrixXd tips(touches.size(), 3);
  for (size_t i = 0; i < touches.size(); ++i) {
    tips.row(i) = nozzle_tip(touches[i], tip_offset_m).transpose();
  }

  Vector3d centroid = tips.colwise().mean();
  MatrixXd centered = tips.rowwise() - centroid.transpose();

  Eigen::JacobiSVD<MatrixXd> svd(centered, Eigen::ComputeFullV);
  Vector3d normal = svd.matrixV().col(2);
  if (normal.z() < 0.0) normal = -normal;   // keep it pointing up

  // bed frame: Z = plane normal, X = base +X projected into the plane, Y = Z x X
  Vector3d x_ref(1, 0, 0);
  if (std::abs(x_ref.dot(normal)) > 0.9) x_ref = Vector3d(0, 1, 0);  // near-vertical bed
  Vector3d x_axis = (x_ref - x_ref.dot(normal) * normal).normalized();
  Vector3d y_axis = normal.cross(x_axis);
  Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = normal;

  BedFit out;
  out.normal = normal;
  out.quat = Quaterniond(R);
  out.quat.normalize();
  out.tilt_deg = std::acos(std::abs(normal.z())) * 180.0 / M_PI;

  Eigen::VectorXd resid = centered * normal;
  out.rms_mm = std::sqrt(resid.array().square().mean()) * 1000.0;
  out.max_off_mm = resid.array().abs().maxCoeff() * 1000.0;
  for (int i = 0; i < resid.size(); ++i) out.per_touch_off_mm.push_back(resid(i) * 1000.0);

  out.center_from_pose = bed_center.has_value();
  out.center = out.center_from_pose ? nozzle_tip(*bed_center, tip_offset_m) : centroid;
  out.center_off_plane_mm = (out.center - centroid).dot(normal) * 1000.0;
  return out;
}

// --- node ---------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bed_from_touches");
  const auto log = node->get_logger();

  // All from machine_settings.yaml / bed_settings.yaml (<param from> in the launch files).
  const bool default_bed = node->declare_parameter("default_bed", true);
  const bool use_extruder = node->declare_parameter("use_extruder", true);
  const double tip_off_ext = node->declare_parameter("tip_offset_with_extruder_m", kTipOffsetM);
  const double tip_off_bare = node->declare_parameter("tip_offset_without_extruder_m", 0.0025);
  const std::vector<double> default_pose =
    node->declare_parameter("default_bed_pose", std::vector<double>{-0.054612, -0.201887, 0.0121794, 0.0, 0.0, 0.0, 1.0});
  const std::vector<double> touch_flat =
    node->declare_parameter("bed_touch_poses", std::vector<double>{});
  const std::vector<double> center_flat =
    node->declare_parameter("bed_center_pose", std::vector<double>{});

  const double tip_offset_m = use_extruder ? tip_off_ext : tip_off_bare;
  double x, y, z, qx, qy, qz, qw;

  const auto touches = reshape6(touch_flat);

  if (default_bed || touches.size() < 3) {
    if (!default_bed && touches.size() < 3) {
      RCLCPP_WARN(log, "default_bed:=false but only %zu valid touch poses -- using the default bed",
                  touches.size());
    }
    if (default_pose.size() != 7) {
      RCLCPP_ERROR(log, "default_bed_pose must have 7 values, got %zu", default_pose.size());
      rclcpp::shutdown();
      return 1;
    }
    x = default_pose[0]; y = default_pose[1]; z = default_pose[2];
    qx = default_pose[3]; qy = default_pose[4]; qz = default_pose[5]; qw = default_pose[6];
    RCLCPP_INFO(log, "default bed: xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
                x, y, z, qx, qy, qz, qw);
  } else {
    std::optional<Joints> center;
    if (center_flat.size() == 6) {
      Joints c{};
      std::copy_n(center_flat.begin(), 6, c.begin());
      center = c;
    } else if (!center_flat.empty()) {
      RCLCPP_WARN(log, "bed_center_pose has %zu values (want 6 or 0) -- using the centroid",
                  center_flat.size());
    }

    BedFit r;
    try {
      r = bed_pose_from_touches(touches, center, tip_offset_m);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(log, "bed fit failed: %s", e.what());
      rclcpp::shutdown();
      return 1;
    }
    x = r.center.x(); y = r.center.y(); z = r.center.z();
    qx = r.quat.x(); qy = r.quat.y(); qz = r.quat.z(); qw = r.quat.w();

    RCLCPP_INFO(log, "fitted bed from %zu touches: rms %.2f mm, worst %.2f mm, tilt %.1f deg",
                touches.size(), r.rms_mm, r.max_off_mm, r.tilt_deg);
    for (size_t i = 0; i < r.per_touch_off_mm.size(); ++i) {
      RCLCPP_INFO(log, "   touch %2zu: %+.2f mm off plane", i + 1, r.per_touch_off_mm[i]);
    }
    RCLCPP_INFO(log, "centre from %s: xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
                r.center_from_pose ? "bed-centre pose" : "centroid of touches",
                x, y, z, qx, qy, qz, qw);
    if (r.center_from_pose) {
      RCLCPP_INFO(log, "   bed-centre pose sits %+.2f mm off the fitted plane", r.center_off_plane_mm);
    }
  }

  auto client = node->create_client<msr_meca500_print_pipeline::srv::Table>("table_service");
  if (!client->wait_for_service(10s)) {
    RCLCPP_ERROR(log, "table_service unavailable -- is the planningscene node running?");
    rclcpp::shutdown();
    return 1;
  }

  auto req = std::make_shared<msr_meca500_print_pipeline::srv::Table::Request>();
  req->x = x; req->y = y; req->z = z;
  req->qx = qx; req->qy = qy; req->qz = qz; req->qw = qw;

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future, 10s) != rclcpp::FutureReturnCode::SUCCESS ||
      !future.get()->success) {
    RCLCPP_ERROR(log, "table_service call failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(log, "bed pose set.");
  rclcpp::shutdown();
  return 0;
}
