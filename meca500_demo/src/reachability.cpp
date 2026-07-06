#include <memory>
#include <vector>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class ReachabilityMap : public rclcpp::Node
{
public:
  ReachabilityMap()
  : Node("reachability_map_node")
  {    
    this->declare_parameter("edge_length", 0.33);
    this->declare_parameter("grid_n", 11);
    this->declare_parameter("z_offset", 0.001);
    this->declare_parameter("out_file", "reachable_points.csv");
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

    using moveit::planning_interface::MoveGroupInterface;
    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "meca500_arm");
    move_group_->startStateMonitor(3.0);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "reachability_dots", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(this->get_logger(), "Waiting for table_marker...");
    auto wait_start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !table_pose_received_ &&
           std::chrono::steady_clock::now() - wait_start < std::chrono::seconds(3)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!table_pose_received_) {
      RCLCPP_WARN(this->get_logger(),
        "No table_marker received after 3s, using default table pose");
    }

    run_sweep();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr table_pos_sub;
  double tx = 0.0, ty = 0.2, tz = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  bool table_pose_received_ = false;


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
    table_pose_received_ = true;
  }


  bool plan_ompl(const geometry_msgs::msg::Pose& target)
  {
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnect");
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target, "link_6__flange");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      return true;
    }
    return false;
  }

  bool plan_lin(const geometry_msgs::msg::Pose& target)
  {
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");
    move_group_->setPlanningTime(5.0);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);
    move_group_->clearPoseTargets();
    move_group_->setPoseTarget(target, "link_6__flange");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->execute(plan);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      return true;
    }
    return false;
  }

  void run_sweep()
  {
    
    double edge = this->get_parameter("edge_length").as_double();
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
    Eigen::Vector3d table_normal = R_table.col(2);
    Eigen::Vector3d to_robot = -table_center;
    if (table_normal.dot(to_robot) < 0) table_normal = -table_normal;
    if (table_normal.z() < 0) table_normal = -table_normal;

    Eigen::Vector3d z_ee = table_normal;
    Eigen::Vector3d vect = Eigen::Vector3d::UnitX();
    if (std::abs(z_ee.dot(vect)) > 0.9) vect = Eigen::Vector3d::UnitY();
    Eigen::Vector3d y_ee = z_ee.cross(vect).normalized();
    Eigen::Vector3d x_ee = y_ee.cross(z_ee).normalized();
    Eigen::Matrix3d R_ee;
    R_ee.col(0) = x_ee; R_ee.col(1) = y_ee; R_ee.col(2) = z_ee;
    Eigen::Quaterniond ee_orient(R_ee);
    ee_orient.normalize();

    Eigen::Vector3d tool_offset = ee_orient.toRotationMatrix() * Eigen::Vector3d(0, 0, 0.015);

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
      "sweeping %dx%d grid over %.3f m board at table(%.3f,%.3f,%.3f)",
      grid_n, grid_n, edge, tx, ty, tz);

    // approach: hover above the board with OMPL, same as trajectory.cpp
    bool approached = false;
    for (double t = 0.05; t <= 0.35; t += 0.02) {
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
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    int reachable = 0, total = 0;
    std::vector<ReachPt> reachable_pts;

    double half = edge / 2.0;
    double step = (grid_n > 1) ? edge / (grid_n - 1) : 0.0;

    for (int iy = 0; iy < grid_n; iy++) {
      for (int ix = 0; ix < grid_n; ix++) {
        double lx = -half + ix * step;
        double ly = -half + iy * step;
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

        visualization_msgs::msg::Marker dot;
        dot.header.frame_id = "world";
        dot.header.stamp = this->now();
        dot.ns = "reachability";
        dot.id = id++;
        dot.type = visualization_msgs::msg::Marker::SPHERE;
        dot.action = visualization_msgs::msg::Marker::ADD;
        dot.pose.position.x = p.x();
        dot.pose.position.y = p.y();
        dot.pose.position.z = p.z();
        dot.pose.orientation.w = 1.0;
        dot.scale.x = dot.scale.y = dot.scale.z = 0.005;
        dot.color.a = 1.0;
        dot.color.r = ok ? 0.0f : 1.0f;
        dot.color.g = ok ? 1.0f : 0.0f;
        dot.color.b = 0.0f;
        dot.lifetime = rclcpp::Duration(0, 0);
        markers.markers.push_back(dot);

        RCLCPP_INFO(this->get_logger(),
          "dot[%d,%d] local(%.3f,%.3f) robot(%.3f,%.3f,%.3f) -> %s",
          ix, iy, lx, ly, p.x(), p.y(), p.z(), ok ? "REACHABLE" : "skip");
      }
    }

    marker_pub_->publish(markers);
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