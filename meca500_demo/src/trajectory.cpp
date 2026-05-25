#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>

#include "gcode.hpp"

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

    //Subscriber
    // gcode_sub = this->create_subscription<std_msgs::msg::String>(
    //     "gcode_input", 10,
    //     [this](const std_msgs::msg::String::SharedPtr msg)
    //     {gcode_callback(msg);
    //     }      
    //   );

    RCLCPP_INFO(this->get_logger(), "Ready. Waiting for G-code...");

    table_pos_sub = this->create_subscription<visualization_msgs::msg::Marker>(
      "table_marker", 10,
      [this](const visualization_msgs::msg::Marker::SharedPtr msg)
      {table_callback(msg);
      }
    );
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gcode_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr table_pos_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;
  double x, y, z, qx, qy, qz, qw;

  // ---------------- PLAN + EXECUTE ----------------
  void plan_and_execute(const std::string& title)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group_->plan(plan))
    {
      RCLCPP_INFO(this->get_logger(), "%s planning OK", title.c_str());
      move_group_->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "%s planning FAILED", title.c_str());
    }
  }

  // ---------------- EXECUTE GCODE ----------------
  void execute(const gcode::GcodeProgram& program)
  {
    move_group_->startStateMonitor(3.0);

    geometry_msgs::msg::PoseStamped current_pose =
        move_group_->getCurrentPose("link_6__flange");

    geometry_msgs::msg::Pose pose = current_pose.pose;

    for (const auto& move : program.moves)
    {
      if (move.command.empty())
        continue;

      // ---------------- G1 ----------------
      if (move.command == "G1")
      {
        RCLCPP_INFO(this->get_logger(), move.command.c_str());

        // move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        // move_group_->setPlannerId("LIN");

        // if (move.has_x) pose.position.x = move.x / 100.0;
        // if (move.has_y) pose.position.y = move.y / 100.0 - 115.0 / 1000.0;
        // if (move.has_z) pose.position.z = move.z / 100.0;

        // geometry_msgs::msg::PoseStamped target;
        // target.header.frame_id = "world";
        // target.pose = pose;

        // RCLCPP_INFO(this->get_logger(), "[LIN] target: x=%.4f y=%.4f z=%.4f  qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
        //   pose.position.x, pose.position.y, pose.position.z,
        //   pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        // move_group_->setPoseTarget(target, "link_6__flange");
        // plan_and_execute("[LIN]");
      }

      // // ---------------- G2 / G3 (still simplified) ----------------
      // else if (move.command == "G2" || move.command == "G3")
      // {
      //   move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      //   move_group_->setPlannerId("CIRC");

      //   if (move.has_x) pose.position.x = move.x / 1000.0;
      //   if (move.has_y) pose.position.y = move.y / 1000.0;
      //   if (move.has_z) pose.position.z = move.z / 1000.0;

      //   geometry_msgs::msg::PoseStamped target;
      //   target.header.frame_id = "world";
      //   target.pose = pose;

      //   move_group_->setPoseTarget(target, "link_6__flange");
      //   plan_and_execute("[CIRC]");
      // }
    }
  }

  // ---------------- GCODE SUBSCRIBER CALLBACK ----------------
  // void gcode_callback(const std_msgs::msg::String::SharedPtr msg)
  // {
  //   RCLCPP_INFO(this->get_logger(), "Received G-code:\n%s", msg->data.c_str());
  //   auto program = gcode::parse(msg->data);
  //   execute(program);
  // }

 

  void table_callback(const visualization_msgs::msg::Marker::SharedPtr msg){
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;

    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Vector3d t(x, y, z);
    
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.normalized().toRotationMatrix();  // rotation part
    T.block<3,1>(0,3) = t;    // translation part
    Eigen::Vector4d P_print(0.1, -0.1, 0.0, 1.0);  // homogeneous coordinates
    Eigen::Vector4d P_robot = T * P_print;
    RCLCPP_INFO(this->get_logger(),
    "Transformed point in robot frame: x=%.3f y=%.3f z=%.3f",
    P_robot.x(), P_robot.y(), P_robot.z());

    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = "world";
    goal_marker.header.stamp = this->get_clock()->now();
    goal_marker.ns = "red";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position.x = P_robot.x();
    goal_marker.pose.position.y = P_robot.y();
    goal_marker.pose.position.z = P_robot.z();
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.02;
    goal_marker.scale.y = 0.02;
    goal_marker.scale.z = 0.02;
    goal_marker.color.r = 0.0f;
    goal_marker.color.g = 0.0f;
    goal_marker.color.b = 0.0f;
    goal_marker.color.a = 1.0;
    goal_marker_pub->publish(goal_marker);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnect");
    move_group_->setPositionTarget(P_robot.x(), P_robot.y(), P_robot.z(), "link_6__flange");
    plan_and_execute("move_to_target");
    
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
