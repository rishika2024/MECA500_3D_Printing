#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <Eigen/Dense>

#include "gcode.hpp"
#include "meca500_demo/srv/goal.hpp"

using namespace std::chrono_literals;

class Meca500Trajectory : public rclcpp::Node
{
public:
  Meca500Trajectory()
  : Node("meca500_trajectory_node",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {}

  // Called from main after the executor is spinning
  void init() {

    using moveit::planning_interface::MoveGroupInterface;

    // Initialize MoveGroupInterface
    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "meca500_arm");

    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "link_0__base", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_->getRobotModel());
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();
    visual_tools_->waitForMarkerSub();

    // Initialize action client for trajectory execution
    action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        shared_from_this(), "meca500_arm_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "Waiting for controller...");
    while (rclcpp::ok() &&
           !action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Still waiting...");
    }
    RCLCPP_INFO(this->get_logger(), "Controller ready.");

    move_group_->startStateMonitor(3.0);
    RCLCPP_INFO(this->get_logger(), "State monitor ready.");

    //Publisher
    goal_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("goal_marker", 10);

    perpendicular_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("perpendicular_marker", 10);

    // Subscriber
    // gcode_sub_ = this->create_subscription<std_msgs::msg::String>(
    //     "gcode_input", 10,
    //     [this](const std_msgs::msg::String::SharedPtr msg)
    //     {gcode_callback(msg);
    //     }
    //   );

    RCLCPP_INFO(this->get_logger(), "Ready. Waiting for G-code...");

    table_pos_sub = this->create_subscription<visualization_msgs::msg::Marker>(
      "table_marker", 10,
      [this](const visualization_msgs::msg::Marker::SharedPtr msg) {
        table_callback(msg);
      }
    );

    // Service
    goal_server = this->create_service<meca500_demo::srv::Goal>(
      "goal_service",
      std::bind(&Meca500Trajectory::goal_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Service client to get motion sequence from the planner
    sequence_client_ = this->create_client<moveit_msgs::srv::GetMotionSequence>("/plan_sequence_path");
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gcode_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr table_pos_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr perpendicular_marker_pub;
  rclcpp::Service<meca500_demo::srv::Goal>::SharedPtr goal_server;
  rclcpp::Client<moveit_msgs::srv::GetMotionSequence>::SharedPtr sequence_client_;
  std::vector<std::array<double, 3>> goal_array; // {{x1, y1, z1}, {x2, y2, z2}, ...}
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  double x, y, z, qx, qy, qz, qw;
  bool moved_to_bed_ = false;
  Eigen::Quaterniond ee_orient;

  void table_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;
  }

  void goal_callback(
    const std::shared_ptr<meca500_demo::srv::Goal::Request>  req,
          std::shared_ptr<meca500_demo::srv::Goal::Response> res) {

    goal_array.clear();
    auto jmg = move_group_->getRobotModel()->getJointModelGroup("meca500_arm");

    // Parse gcode
    gcode::Program program = gcode::parse(req->gcode);
    RCLCPP_INFO(this->get_logger(), "Parsed %zu G moves", program.size());

    Eigen::Quaterniond q_table(qw, qx, qy, qz); // table orientation
    Eigen::Vector3d t_table(x, y, z); // table position
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q_table.normalized().toRotationMatrix();  // rotation part
    T.block<3,1>(0,3) = t_table;    // translation part

    for (const auto& move : program.moves) {
        double pt_z = move.z / 1000.0; //to give a slight offset from the table surface
        if (pt_z < 0.001) pt_z = 0.001;
        Eigen::Vector4d P_table(move.x / 1000.0, move.y / 1000.0, pt_z, 1.0);
        Eigen::Vector4d P_robot = T * P_table;
        goal_array.push_back({P_robot.x(), P_robot.y(), P_robot.z()});
    }

    res->success = true;

    // getting the table's normal vector and center position in the robot frame
    Eigen::Matrix3d R_table = q_table.normalized().toRotationMatrix(); // rotation matrix of the table
    Eigen::Vector3d table_center(x, y, z); // center of the table
    Eigen::Vector3d table_normal = R_table.col(2);

    if (!moved_to_bed_) {
      moved_to_bed_ = true;

      // Build perpendicular orientation
      // Pick the normal direction that points toward the robot base (world origin)
      Eigen::Vector3d to_robot = -table_center;  // vector from table center to origin
      if (table_normal.dot(to_robot) < 0) {
        table_normal = -table_normal;  // flip so it points toward the robot
      }
      Eigen::Vector3d z_ee = table_normal;
      // choose vect to be an axis that is not parallel to z_ee
      // cross product between z_ee and vect will give y_ee, which is perpendicular to z_ee and vect
      // now, x_ee is perpendicular to both z_ee and y_ee,
      // so x_ee = y_ee cross z_ee
      // This way, we can ensure the end-effector's z-axis is aligned with the table normal,
      // and the x and y axes are perpendicular to it, forming a right-handed coordinate system
      Eigen::Vector3d vect = Eigen::Vector3d::UnitX();
      if (std::abs(z_ee.dot(vect)) > 0.9) {
        vect = Eigen::Vector3d::UnitY();
      }
      Eigen::Vector3d y_ee = z_ee.cross(vect).normalized();
      Eigen::Vector3d x_ee = y_ee.cross(z_ee).normalized();

      // End-effector orientation as a rotation matrix and quaternion
      Eigen::Matrix3d R_ee;
      R_ee.col(0) = x_ee;
      R_ee.col(1) = y_ee;
      R_ee.col(2) = z_ee;
      ee_orient = Eigen::Quaterniond(R_ee);
      ee_orient.normalize();
      bool success = false;

      // for the 1st goal point
      // Sample along perpendicular line
      // draw a line from the table surface (t=0) to a point above the table (t=0.35) passing
      // through the goal point
      // try to plan and execute a trajectory for each point along the line until one succeeds

      for (double t = 0.05; t <= 0.35; t += 0.02) {
        Eigen::Vector3d table_origin(0.0, 0.0, 0.001);
        Eigen::Vector3d p = table_center + t * table_normal;

        geometry_msgs::msg::Pose target;
        target.position.x = p.x();
        target.position.y = p.y();
        target.position.z = p.z();
        target.orientation.x = ee_orient.x();
        target.orientation.y = ee_orient.y();
        target.orientation.z = ee_orient.z();
        target.orientation.w = ee_orient.w();

        RCLCPP_INFO(this->get_logger(), "Trying t=%.2f pos[%.3f, %.3f, %.3f]",
            t, p.x(), p.y(), p.z());

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnect");
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.05);
        move_group_->setMaxAccelerationScalingFactor(0.05);
        move_group_->setPoseTarget(target, "link_6__flange");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "SUCCESS at t=%.2f! Executing...", t);
          visual_tools_->publishTrajectoryLine(plan.trajectory, jmg);
          visual_tools_->trigger();
          move_group_->execute(plan);
          success = true;
          // Wait for state monitor to update
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          break;
        }
        move_group_->clearPoseTargets();
      }

      // Now reaching the table surface
      if (success) {
        geometry_msgs::msg::Pose target;

        target.position.x = table_center.x();
        target.position.y = table_center.y();
        target.position.z = table_center.z();
        target.orientation.x = ee_orient.x();
        target.orientation.y = ee_orient.y();
        target.orientation.z = ee_orient.z();
        target.orientation.w = ee_orient.w();

        move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_->setPlannerId("LIN");
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.05);
        move_group_->setMaxAccelerationScalingFactor(0.05);
        move_group_->setPoseTarget(target, "link_6__flange");
        moveit::planning_interface::MoveGroupInterface::Plan lin_to_table_plan;
        if (move_group_->plan(lin_to_table_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Planning to table surface SUCCESS! Executing...");
          visual_tools_->publishTrajectoryLine(lin_to_table_plan.trajectory, jmg);
          visual_tools_->trigger();
          move_group_->execute(lin_to_table_plan);
        }
        RCLCPP_INFO(this->get_logger(), "Reached table origin!");
      }
    }

    if (moved_to_bed_) {
      for (size_t i = 0; i < goal_array.size(); i++) {
        geometry_msgs::msg::Pose target;
        target.position.x = goal_array[i][0];
        target.position.y = goal_array[i][1];
        target.position.z = goal_array[i][2];
        target.orientation.x = ee_orient.x();
        target.orientation.y = ee_orient.y();
        target.orientation.z = ee_orient.z();
        target.orientation.w = ee_orient.w();

        move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_->setPlannerId("LIN");
        move_group_->setMaxVelocityScalingFactor(0.05);
        move_group_->setMaxAccelerationScalingFactor(0.05);
        move_group_->setPoseTarget(target, "link_6__flange");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "LIN to point %zu SUCCESS", i);
          visual_tools_->publishTrajectoryLine(plan.trajectory, jmg);
          visual_tools_->trigger();
          move_group_->execute(plan);
        } else {
          RCLCPP_ERROR(this->get_logger(), "LIN to point %zu FAILED", i);
          res->success = false;
          return;
        }
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Meca500Trajectory>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  node->init();

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
