// Standalone hand-eye calibration data collector. Run in a separate
// terminal after trajectory.launch.xml (needs its move_group) AND the
// camera/apriltag pipeline (needs tag4 broadcasting TF) are both already
// up. Moves through the 12 previously-recorded calibration poses one at
// a time, waits for the arm to settle, samples tag4's pose in camera
// frame until it looks stable, records both the real reached joint
// values and the averaged tag reading to a CSV, then returns home.
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <cmath>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "handeye_capture", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, "meca500_arm");
  move_group->setPlanningPipelineId("ompl");
  move_group->setPlannerId("RRTConnect");
  move_group->setPlanningTime(5.0);
  move_group->setMaxVelocityScalingFactor(0.3);
  move_group->setMaxAccelerationScalingFactor(0.3);

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Now launched alongside realsense/apriltag_node in the same
  // handeye.launch.xml instead of a separate terminal -- give the camera
  // driver a head start before pose 1 needs tag4 actually detecting.
  RCLCPP_INFO(node->get_logger(), "Waiting for camera/apriltag to come up...");
  std::this_thread::sleep_for(8000ms);

  const std::vector<std::vector<double>> poses = {
    {0.18260507298990672, -0.2748893571891069, 0.28509953331327376, -0.015271630954950384, -1.03419484826924, 0.08290313946973064},
    {0.18258250588267844, -0.27487581343411144, 0.04454652719084183, -0.25292227670436335, -1.3810188581505043, 0.0825570755856454},
    {0.4401499414846397, 0.12107832477287467, 0.21886851189687873, -0.3332710570006302, -0.981009674319312, -0.06542808269401487},
    {-0.15226735850601333, 0.15320550788554532, 0.10646812124455257, -0.09103911395094985, -0.9437707359868153, 0.27989494403053417},
    {0.6949012185253379, 0.0004567352119543961, -0.41155156977340623, -0.6939587581825536, -1.4095057033225353, 1.0231157250703828},
    {0.7404513100360764, -0.5352892352742521, 0.7702858984564974, 0.052067726896338554, -0.7171304823703161, -0.41201683964540314},
    {0.4351105825221863, -0.30531720047557326, 0.5119718982669957, 0.0, -0.26152987405605643, 0.7199483164476609},
    {-0.3409898628840377, -0.28413532616800946, 0.4977066241587453, 0.6818753550788063, -0.8808594355274686, -1.6080612007200774},
    {0.6626785833558757, -0.45455414380134884, 0.4064683681401428, -0.2691167854078908, -0.5906380240847073, 1.2936412180774357},
    {-0.03330706059360387, -0.4195205336985021, 0.4448827508172727, 0.6048802598468315, -0.6740359643973785, -1.1084013262355479},
    {0.0, -0.2749, 0.2851, -0.015, -1.03, 0.083},
    {1.0123, -0.4546, 0.4065, -0.2691, -0.5906, 1.2941},   
  };

  std::ofstream csv("/home/rishika/ws/meca500/src/Final_Project/handeye_capture.csv");
  csv << "pose,j1,j2,j3,j4,j5,j6,cam_x,cam_y,cam_z,cam_qx,cam_qy,cam_qz,cam_qw,n_samples\n";

  for (size_t p = 0; p < poses.size(); p++) {
    RCLCPP_INFO(node->get_logger(), "=== Pose %zu/%zu ===", p + 1, poses.size());
    move_group->setJointValueTarget(poses[p]);
    auto result = move_group->move();

    csv << (p + 1);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Pose %zu: failed to plan/execute (code %d) -- skipping",
        p + 1, result.val);
      for (int k = 0; k < 6; k++) csv << ",na";
      csv << ",na,na,na,na,na,na,na,0\n";
      csv.flush();
      continue;
    }

    // 3 runs' worth of real data showed bad internal consistency every
    // time -- likely the arm/mount was still settling during the old
    // 1.5s wait + early-exit-at-8-samples window, biasing the average
    // toward whatever the system was doing mid-settle instead of its
    // true resting pose. Wait it out fully first, THEN sample.
    RCLCPP_INFO(node->get_logger(), "Pose %zu: waiting 10s to fully settle before sampling", p + 1);
    std::this_thread::sleep_for(10000ms);

    auto joint_values = move_group->getCurrentJointValues();
    for (double v : joint_values) csv << "," << v;

    // Now sample tag4 for a fixed 5s window and average everything that
    // comes in -- no early exit, so this always reflects the arm at
    // rest, not a partial sample of it still moving.
    std::vector<geometry_msgs::msg::Transform> samples;
    auto deadline = node->now() + rclcpp::Duration::from_seconds(5.0);
    while (node->now() < deadline) {
      if (tf_buffer.canTransform("camera_color_optical_frame", "tag4", tf2::TimePointZero)) {
        auto tf = tf_buffer.lookupTransform("camera_color_optical_frame", "tag4", tf2::TimePointZero).transform;
        RCLCPP_INFO(node->get_logger(),
          "Pose %zu sample %zu: tag4 xyz=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f)",
          p + 1, samples.size() + 1, tf.translation.x, tf.translation.y, tf.translation.z,
          tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
        samples.push_back(tf);
      }
      std::this_thread::sleep_for(200ms);
    }

    if (samples.empty()) {
      RCLCPP_WARN(node->get_logger(), "Pose %zu: tag4 never detected", p + 1);
      csv << ",na,na,na,na,na,na,na,0\n";
      csv.flush();
      continue;
    }

    double sx = 0, sy = 0, sz = 0, sqx = 0, sqy = 0, sqz = 0, sqw = 0;
    for (auto & t : samples) {
      sx += t.translation.x; sy += t.translation.y; sz += t.translation.z;
      sqx += t.rotation.x; sqy += t.rotation.y; sqz += t.rotation.z; sqw += t.rotation.w;
    }
    size_t n = samples.size();
    sx /= n; sy /= n; sz /= n; sqx /= n; sqy /= n; sqz /= n; sqw /= n;
    double qn = std::sqrt(sqx * sqx + sqy * sqy + sqz * sqz + sqw * sqw);
    sqx /= qn; sqy /= qn; sqz /= qn; sqw /= qn;

    RCLCPP_INFO(node->get_logger(),
      "Pose %zu: averaged tag4 xyz=(%.4f,%.4f,%.4f) quat=(%.4f,%.4f,%.4f,%.4f) from %zu samples",
      p + 1, sx, sy, sz, sqx, sqy, sqz, sqw, n);

    csv << "," << sx << "," << sy << "," << sz << ","
        << sqx << "," << sqy << "," << sqz << "," << sqw << "," << n << "\n";
    csv.flush();
  }

  RCLCPP_INFO(node->get_logger(), "Returning to home");
  move_group->clearPoseTargets();
  move_group->setNamedTarget("Home");
  if (move_group->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(node->get_logger(), "Could not plan/execute back to Home");
  }

  csv.close();
  RCLCPP_INFO(node->get_logger(), "Done. Saved to handeye_capture.csv");

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
