#include <memory>
#include <vector>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class ReachabilityMap : public rclcpp::Node
{
public:
  ReachabilityMap()
  : Node("reachability_map_node")
  {    
    this->declare_parameter("grid_n", 11);
    this->declare_parameter("z_offset", 0.001);
    this->declare_parameter("out_file", "reachable_points.csv");
    this->declare_parameter("use_extruder", true);
  }

  void init()
  {
     // Subscriber
    RCLCPP_INFO(this->get_logger(), "Ready. Waiting for G-code...");

    table_pos_sub = this->create_subscription<visualization_msgs::msg::Marker>(
      "table_marker", 10,
      [this](const visualization_msgs::msg::Marker::SharedPtr msg) {
        table_callback(msg);
      }
    );

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = msg;
      }
    );

    using moveit::planning_interface::MoveGroupInterface;
    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "meca500_arm");
    move_group_->startStateMonitor(3.0);

    
    // Wait out the full window regardless of whether a message arrives early --
    // table_callback keeps overwriting tx,ty,tz,... as messages come in, and we
    // want whatever is current when the window ends, not just the first message
    // received (which could be a stale reading if more than one node is ever
    // publishing table_marker at once).
    RCLCPP_INFO(this->get_logger(), "Waiting for table_marker...");
    auto wait_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() - wait_start < std::chrono::seconds(3)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!table_pose_received_) {
      RCLCPP_WARN(this->get_logger(),
        "No table_marker received after 3s, using default table pose");
    }

    use_extruder_ = this->get_parameter("use_extruder").as_bool();
    RCLCPP_INFO(this->get_logger(), "Extruder: %s", use_extruder_ ? "true" : "false");

    run_sweep();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
 
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr table_pos_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  double tx = 0.0, ty = -0.2, tz = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  double table_width_ = 0.33, table_depth_ = 0.33;  // from the marker's scale.x/scale.y
  bool table_pose_received_ = false;
  bool use_extruder_ = true;
  std::string end_effector_link_ = "nozzle";
  double threshold = 0.01;


  struct ReachPt { double lx, ly, rx, ry, rz; };

  bool finished = false;
  bool shutdown_requested = false;

  void table_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    tx = msg->pose.position.x;
    ty = msg->pose.position.y;
    tz = msg->pose.position.z;
    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;
    table_width_ = msg->scale.x;
    table_depth_ = msg->scale.y;
    table_pose_received_ = true;
  }


  // Block until every joint's reported velocity is below vel_threshold, or
  // timeout elapses. Guards against sending the next command while the arm
  // still has leftover velocity from the previous move -- ros2_control's
  // joint limiter clamps a position command that's kinematically infeasible
  // given the current velocity, silently landing the arm somewhere other
  // than the commanded target while move_group still reports SUCCEEDED.
  bool wait_until_settled(double vel_threshold = 0.01, double timeout_sec = 2.0)
  {
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() &&
           std::chrono::steady_clock::now() - start < std::chrono::duration<double>(timeout_sec)) {
      if (latest_joint_state_ && !latest_joint_state_->velocity.empty()) {
        bool settled = true;
        for (double v : latest_joint_state_->velocity) {
          if (std::abs(v) > vel_threshold) { settled = false; break; }
        }
        if (settled) return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    RCLCPP_WARN(this->get_logger(),
      "wait_until_settled: timed out after %.1fs, proceeding anyway", timeout_sec);
    return false;
  }

  bool plan_ompl(const geometry_msgs::msg::Pose& target)
  {
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnect");
    move_group_->setPlanningTime(10.0);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target, end_effector_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      wait_until_settled();
      return true;
    }
    return false;
  }

  bool plan_lin(const geometry_msgs::msg::Pose& target)
  {
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target, end_effector_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      wait_until_settled();
      return true;
    }
    return false;
  }

  void run_sweep()
  {

    if (use_extruder_) {
      end_effector_link_ = "nozzle";
      threshold = 0.011;
    } 
    
    else {
      end_effector_link_ = "link_6__flange";
      threshold = 0.0025;
    }
    
    int grid_n = this->get_parameter("grid_n").as_int();
    double z_off = this->get_parameter("z_offset").as_double();
    std::string out_file = this->get_parameter("out_file").as_string();

    Eigen::Quaterniond q_table(qw, qx, qy, qz);
    q_table.normalize();
    Eigen::Vector3d t_table(tx, ty, tz);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q_table.toRotationMatrix();
    T.block<3,1>(0,3) = t_table;

    Eigen::Matrix3d R_table = q_table.toRotationMatrix();
    Eigen::Vector3d table_center(tx, ty, tz);

    // Flip the table normal based on the vector from the actual live
    // end-effector position to the table center (matches trajectory.cpp) --
    // if the normal points away from where the arm actually is, flip it
    // toward the arm, instead of assuming the robot base sits at the world
    // origin and forcing a hardcoded downward direction.
    auto ee_pose = move_group_->getCurrentPose(end_effector_link_).pose;
    Eigen::Vector3d ee_to_table_vector(ee_pose.position.x - table_center.x(),
                                       ee_pose.position.y - table_center.y(),
                                       ee_pose.position.z - table_center.z());
    if (R_table.col(2).dot(ee_to_table_vector) < 0) {
      R_table.col(2) = -R_table.col(2);
    }

    Eigen::Vector3d table_normal = R_table.col(2);

    RCLCPP_INFO(this->get_logger(),
      "table_center=[%.3f,%.3f,%.3f] table_normal(after flip)=[%.3f,%.3f,%.3f]",
      table_center.x(), table_center.y(), table_center.z(),
      table_normal.x(), table_normal.y(), table_normal.z());

    // Force the end-effector's z-axis to point INTO the table (opposite the build direction)
    Eigen::Vector3d z_ee = -table_normal;
    Eigen::Vector3d vect = Eigen::Vector3d::UnitX();
    if (std::abs(z_ee.dot(vect)) > 0.9) vect = Eigen::Vector3d::UnitY();
    Eigen::Vector3d y_ee = z_ee.cross(vect).normalized();
    Eigen::Vector3d x_ee = y_ee.cross(z_ee).normalized();
    Eigen::Matrix3d R_ee;
    R_ee.col(0) = x_ee; R_ee.col(1) = y_ee; R_ee.col(2) = z_ee;
    Eigen::Quaterniond ee_orient(R_ee);
    ee_orient.normalize();

    // Commanding the nozzle *frame*, which sits 8mm short of the true tip along
    // local +Z, so pull the frame back by 8mm to land the tip on the target.
    Eigen::Vector3d tool_offset = ee_orient.toRotationMatrix() * Eigen::Vector3d(0, 0, -threshold);

    auto make_pose = [&](const Eigen::Vector3d& p) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = p.x() + tool_offset.x();
      pose.position.y = p.y() + tool_offset.y();
      pose.position.z = p.z() + tool_offset.z();
      pose.orientation.x = ee_orient.x();
      pose.orientation.y = ee_orient.y();
      pose.orientation.z = ee_orient.z();
      pose.orientation.w = ee_orient.w();
      return pose;
    };

    RCLCPP_INFO(this->get_logger(),
      "sweeping %dx%d grid over %.3f x %.3f m board at table(%.3f,%.3f,%.3f)",
      grid_n, grid_n, table_width_, table_depth_, tx, ty, tz);

    // approach: hover above the board with OMPL, same as trajectory.cpp
    bool approached = false;
    for (double t = 0.05; t <= 0.35; t += 0.02) {
      // Standoff moves OUT from the table center, along the build direction (table_normal)
      // which for your pose also moves toward the base
      Eigen::Vector3d p = table_center + t * table_normal;
      geometry_msgs::msg::Pose target = make_pose(p);
      RCLCPP_INFO(this->get_logger(), "approach: trying standoff t=%.2f [%.3f,%.3f,%.3f]",
        t, p.x(), p.y(), p.z());
      if (plan_ompl(target)) {
        RCLCPP_INFO(this->get_logger(), "approach: standoff reached at t=%.2f", t);
        approached = true;
        break;
      }
    }

    if (!approached) {
      RCLCPP_ERROR(this->get_logger(),
        "approach FAILED: cannot reach any standoff pose above the board.");
      RCLCPP_INFO(this->get_logger(), "Shutting down ROS after failed approach");
      rclcpp::shutdown();
      return;
    }

    // ################################ BEGIN CITATION [] ######################################

    // LIN down to the surface center, same as trajectory.cpp
    {
      geometry_msgs::msg::Pose target = make_pose(table_center);
      if (plan_lin(target)) {
        RCLCPP_INFO(this->get_logger(), "reached board surface center");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "could not LIN to surface center, continuing sweep from standoff");
      }
    }

    // sweep: each dot is a Pilz LIN from wherever the arm currently is
    
    int reachable = 0, total = 0;
    std::vector<ReachPt> reachable_pts;

    double half_x = table_width_ / 2.0;
    double half_y = table_depth_ / 2.0;
    double step_x = (grid_n > 1) ? table_width_ / (grid_n - 1) : 0.0;
    double step_y = (grid_n > 1) ? table_depth_ / (grid_n - 1) : 0.0;

    for (int iy = grid_n - 1; iy >= 0; iy--) {
      for (int ix = grid_n - 1; ix >= 0; ix--) {
        double lx = -half_x + ix * step_x;
        double ly = -half_y + iy * step_y;
        total++;

        Eigen::Vector4d P_table(lx, ly, z_off, 1.0);
        Eigen::Vector4d P_robot = T * P_table;
        Eigen::Vector3d p(P_robot.x(), P_robot.y(), P_robot.z());
        geometry_msgs::msg::Pose target = make_pose(p);

        bool ok = plan_lin(target);

        if (ok) {
          reachable++;
          reachable_pts.push_back({lx, ly, p.x(), p.y(), p.z()});
        }    

        RCLCPP_INFO(this->get_logger(),
          "dot[%d,%d] local(%.3f,%.3f) robot(%.3f,%.3f,%.3f) -> %s",
          ix, iy, lx, ly, p.x(), p.y(), p.z(), ok ? "REACHABLE" : "skip");
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "=== %d / %d dots reachable (%.0f%%) ===",
      reachable, total, 100.0 * reachable / std::max(1, total));

    RCLCPP_INFO(this->get_logger(), "=== REACHABLE POINTS (%zu) ===", reachable_pts.size());
    RCLCPP_INFO(this->get_logger(), "  idx |   local x,y (m)   |     robot x,y,z (m)");
    for (size_t k = 0; k < reachable_pts.size(); k++) {
      const auto& p = reachable_pts[k];
      RCLCPP_INFO(this->get_logger(), "  %3zu | %7.4f %7.4f | %7.4f %7.4f %7.4f",
        k, p.lx, p.ly, p.rx, p.ry, p.rz);
    }
    RCLCPP_INFO(this->get_logger(), "=== END REACHABLE POINTS ===");

    if (!out_file.empty()) {
      std::ofstream f(out_file);
      if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open %s for writing", out_file.c_str());
      }
      else {
        f << "idx,local_x,local_y,robot_x,robot_y,robot_z\n";
        for (size_t k = 0; k < reachable_pts.size(); k++) {
          const auto& p = reachable_pts[k];
          f << k << "," << p.lx << "," << p.ly << ","
            << p.rx << "," << p.ry << "," << p.rz << "\n";
        }
        f.close();
        RCLCPP_INFO(this->get_logger(), "Wrote %zu reachable points to %s",
          reachable_pts.size(), out_file.c_str());        
      }
    }
    // ################################ END CITATION [] ######################################

    finished = true;

    if (finished) {
      RCLCPP_INFO(this->get_logger(), "Sweep finished, going back to home position");
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setPlannerId("RRTConnect");
      move_group_->setPlanningTime(5.0);
      move_group_->setMaxVelocityScalingFactor(0.5);
      move_group_->setMaxAccelerationScalingFactor(0.5);
      move_group_->clearPoseTargets();
      move_group_->setNamedTarget("Home");

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_->execute(plan);
        // Every other move in this file waits here before moving on; this
        // one shut down right after execute() returned instead, which per
        // wait_until_settled()'s own comment can report SUCCESS before the
        // arm has actually stopped moving/reached the commanded position.
        wait_until_settled();
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Could not plan to home position");
      }
      shutdown_requested = true;
    }

    if (shutdown_requested) {
      RCLCPP_INFO(this->get_logger(), "Shutting down ROS after reachability sweep");
      rclcpp::shutdown();
      return;
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReachabilityMap>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  node->init();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}