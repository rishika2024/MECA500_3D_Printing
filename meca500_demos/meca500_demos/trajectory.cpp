#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "meca500_trajectory_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto const logger = rclcpp::get_logger("meca500_trajectory_node");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "meca500_arm");

  // Wait for controller
  RCLCPP_INFO(logger, "Waiting for controller...");
  auto action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node, "meca500_arm_controller/follow_joint_trajectory");
  while (rclcpp::ok() &&
         !action_client->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_INFO(logger, "Still waiting for meca500_arm_controller...");
  }
  RCLCPP_INFO(logger, "Controller ready.");

  // Visual tools for trajectory lines
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "link_0__base",
          rviz_visual_tools::RVIZ_MARKER_TOPIC,
          move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.waitForMarkerSub();

  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup("meca500_arm")](
          auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };

  auto const plan_and_execute = [&](const std::string& title) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = static_cast<bool>(move_group_interface.plan(plan));
    if (ok)
    {
      RCLCPP_INFO(logger, "%s: planned, visualizing and executing...", title.c_str());
      draw_trajectory_tool_path(plan.trajectory);
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
      RCLCPP_INFO(logger, "%s: done.", title.c_str());
    }
    else
    {
      RCLCPP_ERROR(logger, "%s: planning failed!", title.c_str());
    }
  };

  move_group_interface.setPlanningPipelineId("pilz_industrial_motion_planner");

  {
    move_group_interface.setPlannerId("PTP");
    auto const pre_grasp_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.5;
      msg.pose.orientation.y = 0.5;
      msg.pose.orientation.z = -0.5;
      msg.pose.orientation.w = 0.5;
      msg.pose.position.x = 0.0;
      msg.pose.position.y = 0.12;
      msg.pose.position.z = 0.18;
      return msg;
    }();
    move_group_interface.setPoseTarget(pre_grasp_pose, "link_6__flange");
    plan_and_execute("[PTP] Approach");
  }

  {
    move_group_interface.setPlannerId("LIN");
    auto const grasp_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.5;
      msg.pose.orientation.y = 0.5;
      msg.pose.orientation.z = -0.5;
      msg.pose.orientation.w = 0.5;
      msg.pose.position.x = 0.0;
      msg.pose.position.y = 0.12;
      msg.pose.position.z = 0.12;
      return msg;
    }();
    move_group_interface.setPoseTarget(grasp_pose, "link_6__flange");
    plan_and_execute("[LIN] Grasp");
  }

  {
    move_group_interface.setPlannerId("CIRC");
    auto const goal_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.5;
      msg.pose.orientation.y = 0.5;
      msg.pose.orientation.z = -0.5;
      msg.pose.orientation.w = 0.5;
      msg.pose.position.x = 0.0;
      msg.pose.position.y = 0.12;
      msg.pose.position.z = 0.18;
      return msg;
    }();
    move_group_interface.setPoseTarget(goal_pose, "link_6__flange");

    auto const center_pose = [] {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.pose.orientation.x = 0.5;
      msg.pose.orientation.y = 0.5;
      msg.pose.orientation.z = -0.5;
      msg.pose.orientation.w = 0.5;
      msg.pose.position.x = 0.05;
      msg.pose.position.y = 0.12;
      msg.pose.position.z = 0.15;
      return msg;
    }();
    moveit_msgs::msg::Constraints constraints;
    constraints.name = "interim";
    moveit_msgs::msg::PositionConstraint pos_constraint;
    pos_constraint.header.frame_id = center_pose.header.frame_id;
    pos_constraint.link_name = "link_6__flange";
    pos_constraint.constraint_region.primitive_poses.push_back(center_pose.pose);
    pos_constraint.weight = 1.0;
    constraints.position_constraints.push_back(pos_constraint);
    move_group_interface.setPathConstraints(constraints);

    plan_and_execute("[CIRC] Turn");
  }

  {
    move_group_interface.setPlannerId("PTP");
    move_group_interface.setNamedTarget("Home");
    plan_and_execute("[PTP] Return");
  }

  RCLCPP_INFO(logger, "All motions complete. Ctrl+C to exit.");
  spinner.join();
  rclcpp::shutdown();
  return 0;
}