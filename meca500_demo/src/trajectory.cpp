#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
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
  bool moved_to_bed_ = false;

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
    goal_marker.color.r = 1.0f;
    goal_marker.color.g = 0.0f;
    goal_marker.color.b = 0.0f;
    goal_marker.color.a = 1.0;
    goal_marker_pub->publish(goal_marker);

    if (!moved_to_bed_) {
      moved_to_bed_ = true;

      // Compute FK at home config to get a safe EE orientation
      // (prevents KDL from finding joint6 = 628 rad with the raw table orientation)
      const auto* jmg = move_group_->getRobotModel()->getJointModelGroup("meca500_arm");
      auto seed = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());
      std::vector<double> home = {0.0, 0.7069, 0.6140, -1.4471, 0.0352, 0.0};
      seed->setJointGroupPositions(jmg, home);
      seed->update();
      Eigen::Isometry3d home_tf = seed->getGlobalLinkTransform("link_6__flange");
      Eigen::Quaterniond home_q(home_tf.rotation());

      geometry_msgs::msg::Pose target;
      target.position.x = P_robot.x();
      target.position.y = P_robot.y();
      target.position.z = P_robot.z();
      target.orientation.x = home_q.x();
      target.orientation.y = home_q.y();
      target.orientation.z = home_q.z();
      target.orientation.w = home_q.w();

      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("PTP");
      move_group_->setMaxVelocityScalingFactor(1.0);
      move_group_->setMaxAccelerationScalingFactor(1.0);
      move_group_->setPoseTarget(target, "link_6__flange");
      
      plan_and_execute("move_to_target");
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
