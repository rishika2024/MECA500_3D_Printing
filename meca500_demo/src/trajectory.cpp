#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <Eigen/Dense>
#include <sensor_msgs/msg/joint_state.hpp>
#include "gcode.hpp"
#include "meca500_demo/srv/goal.hpp"
#include <fstream>
#include "meca500_demo/srv/gcode_file.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class Meca500Trajectory : public rclcpp::Node
{
public:
  Meca500Trajectory()
  : Node("meca500_trajectory_node",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))

         // instead of this->declare_parameter<bool>("use_mock_hardware", true);
         // then later this->get_parameter("use_mock_hardware").as_bool();
         // automatically_declare_parameters_from_overrides(true) allows you to set 
         // parameters from the command line or a YAML file without having to declare 
         // them explicitly in the code.
         
  {}

  void init() {

    using moveit::planning_interface::MoveGroupInterface;

    // Initialize MoveGroupInterface
    move_group = std::make_shared<MoveGroupInterface>(shared_from_this(), "meca500_arm");
    // move group interface can be initialized in 2 ways:
    // using a pointer to the node (shared_from_this()) and the name of the move group ("meca500_arm")
    // or instead of the name of the move group, it can be customized to use a 
    // specific planning pipeline, urdf, or other parameters.
    // basically option 1 is the default way to initialize the move group interface, 
    // and option 2 is more advanced and allows for more customization.


    visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
      shared_from_this(), "link_0__base", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group->getRobotModel());
      // visual tools needs a pointer to the node (shared_from_this()), 
      // the name of the base link ("link_0__base"), and the topic for the markers.
      // last option could be the robot model or planning scene monitor
    
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
    visual_tools->waitForMarkerSub();

    // Initialize action client for trajectory execution
    action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        shared_from_this(), "meca500_arm_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "Waiting for controller...");

    // check every second if the action server is available, 
    // and print a message every second until it is available
    while (rclcpp::ok() &&
           !action_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Still waiting for action server...");
    }
    RCLCPP_INFO(this->get_logger(), "Controller ready.");

    move_group->startStateMonitor(3.0);
    RCLCPP_INFO(this->get_logger(), "State monitor ready.");

    use_mock_hardware = this->get_parameter("use_mock_hardware").as_bool();
    RCLCPP_INFO(this->get_logger(), "Mock hardware: %s", use_mock_hardware ? "true" : "false");

    use_extruder = this->get_parameter("use_extruder").as_bool();
    RCLCPP_INFO(this->get_logger(), "Extruder: %s", use_extruder ? "true" : "false");

    real_print = this->get_parameter("real_print").as_bool();
    RCLCPP_INFO(this->get_logger(), "Real print: %s", real_print ? "true" : "false");

    //Publisher
    trace_pub = this->create_publisher<visualization_msgs::msg::Marker>("ee_trace", 10);
    print_trace_pub = this->create_publisher<visualization_msgs::msg::Marker>("print_trace", 10);
    joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // reentrant callback group allows multiple callbacks to be executed concurrently
    // mutually exclusive callback group allows only one callback to be executed at a time
    joint_state_timer = this->create_wall_timer(50ms, [this]() {
      // when using mock hardware, the joint states generally not published by the controller
      // hence doing it manually here
      // lock_guard ensures that the last_joint_state is not being modified while we are reading it to publish
      std::lock_guard<std::mutex> lock(last_joint_state_mutex);
      if (!last_joint_state.name.empty()) {
        last_joint_state.header.stamp = this->now();
        joint_state_pub->publish(last_joint_state);
      }
    }, timer_cb_group);

    // Subscriber
    RCLCPP_INFO(this->get_logger(), "Ready. Waiting for G-code...");

    table_pos_sub = this->create_subscription<visualization_msgs::msg::Marker>(
      "table_marker", 10,
      [this](const visualization_msgs::msg::Marker::SharedPtr msg) {
        table_callback(msg);
      }
    );

    // Service
    service_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    goal_server = this->create_service<meca500_demo::srv::Goal>(
      "goal_service",
      std::bind(&Meca500Trajectory::goal_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_cb_group);

    gcode_file_server = this->create_service<meca500_demo::srv::GcodeFile>(
      "gcode_file_service",
      std::bind(&Meca500Trajectory::gcode_file_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_cb_group);

    action_sequence_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_sequence_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroupSequence>(
      shared_from_this(), "/sequence_move_group", action_sequence_cb_group);

    // End effector link threshold
    if (use_extruder) {
      threshold = 0.009;
      end_effector_link = "nozzle";
    }     
    else {
      threshold = 0.0005;
      end_effector_link = "link_6__flange";
    }
  

  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr table_pos_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trace_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr print_trace_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  sensor_msgs::msg::JointState last_joint_state;
  std::mutex last_joint_state_mutex;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group;
  rclcpp::CallbackGroup::SharedPtr service_cb_group;
  rclcpp::CallbackGroup::SharedPtr action_sequence_cb_group;
  rclcpp::TimerBase::SharedPtr joint_state_timer;
  std::vector<geometry_msgs::msg::Point> trace_points;
  std::vector<geometry_msgs::msg::Point> print_trace_points;
  rclcpp::Service<meca500_demo::srv::Goal>::SharedPtr goal_server;
  rclcpp::Service<meca500_demo::srv::GcodeFile>::SharedPtr gcode_file_server;
  rclcpp_action::Client<moveit_msgs::action::MoveGroupSequence>::SharedPtr action_sequence_client;

  // ################################ BEGIN CITATION [] ######################################
  static constexpr const char* kEnderPort = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0";
  // ################################## END CITATION [] ######################################


 
  // Print: e > 1e-6 Extrusion when mock hardware is not used.
  // Retract: e < -1e-6 Retract when mock hardware is not used
  //(taken as pure travel when mock hardware is used)
  // e approximately zero: pure travel (no extrusion)
  enum struct MoveType { Print, Retract, None };

  // One move, fully resolved: target pose has tool_offset + orientation
  // already applied, and mid_pt (for arcs) is precomputed -- nothing
  // downstream ever recomputes either from raw coordinates.
  struct Trajectories {
      std::string cmd;                       // "G1", "G2", or "G3"
      geometry_msgs::msg::PoseStamped pose;   // target pose (tool_offset + orientation already applied)
      geometry_msgs::msg::Pose mid_pt;        // interim point; only meaningful when cmd == "G2"/"G3"
      double e;
      double f;
      
  };
  using Trajectory = std::vector<Trajectories>;

  struct TrajectoryBatch {
    Trajectory trajectory;
    MoveType type;
  };

  std::vector<TrajectoryBatch> trajectory_batches;

  // Data type to store ik failure information
  struct IkFailure {
    Eigen::Vector3d position;   // target this point was trying to reach
    std::string original_cmd;   // "G1", "G2", or "G3" -- before any demotion
    Eigen::Vector3d start;      // remembered_pose -- where this move was supposed to start from
    bool was_arc;               // true if original_cmd was G2/G3
    Eigen::Vector3d mid_pt;     // interim point sent to Pilz -- only meaningful if was_arc
    double cross_norm;          // (mid-start) x (target-start) norm -- only meaningful if was_arc
    std::string reason;
  };

  std::vector<IkFailure> ik_failure_log;
  size_t total_moves = 0;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
  double x, y, z, qx, qy, qz, qw;
  bool moved_to_bed = false;
  bool use_mock_hardware = true;
  std::string end_effector_link = "nozzle";
  bool use_extruder = true;
  Eigen::Quaterniond ee_orient;
  bool is_print = true;
  int print_seg_id = 0;
  bool print_finished = false;
  double threshold = 0.008;
  bool real_print = true;
  double f_max;
  double f_min;
  // Real (not estimated) range of matched_f -- discovered by a plan-only
  // reconnaissance pass over the whole print before execution starts, used
  // to rescale each batch's extrusion feed rate into the extruder's real
  // safe range instead of flattening most of the print to one flat number.
  double matched_f_min_seen;
  double matched_f_max_seen;

  // Scale factor for extrusion, applied to the E value in G-code. Default is 1.0 (no scaling).
  double extrusion_multiplier = 1.0;

  int ender_fd = -1;
  bool ender_ready = false;

  // Table Callback
  void table_callback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
    qx = msg->pose.orientation.x;
    qy = msg->pose.orientation.y;
    qz = msg->pose.orientation.z;
    qw = msg->pose.orientation.w;
  }
  
  // Calculate the midpoint of an arc given the start, end, and center points, and the command (G2 or G3)
  Eigen::Vector3d get_arc_center(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                  const Eigen::Vector3d& center, const std::string& command) {

    double radius = (start - center).norm();

    double start_angle = atan2(start.y() - center.y(), start.x() - center.x());
    double end_angle = atan2(end.y() - center.y(), end.x() - center.x());
    double mid_angle;
    double angle_diff;

    if (command == "G2") { // Clockwise
      angle_diff = start_angle - end_angle;
      if (angle_diff <= 0) {
        angle_diff += 2 * M_PI;
      }
      mid_angle = start_angle - angle_diff / 2.0;
    }
    else if (command == "G3") { // Counterclockwise
      angle_diff = end_angle - start_angle;
      if (angle_diff <= 0) {
        angle_diff += 2 * M_PI;
      }
      mid_angle = start_angle + angle_diff / 2.0;
    }

    Eigen::Vector3d mid_point;
    mid_point.x() = center.x() + radius * cos(mid_angle);
    mid_point.y() = center.y() + radius * sin(mid_angle);
    mid_point.z() = center.z();

    return mid_point;
  }
  
  // Function to scale feed rate
  double f_scaling(double f_max, double f_min, double f, bool has_f) {
    if (!has_f) {
      return 0.1;
    }
    if (f_max == f_min) {
      return 0.5;
    }
    return std::max(0.05, (f - f_min) / (f_max - f_min));
  }


  // ################################ Begin Citation[]###############################

  // Opens the Ender3's serial port for the duration of one print job --
  // called at the start of goal_callback(), after printer.py has already
  // run (and closed its own connection) in the reachability -> printer.py
  // -> trajectory pipeline, so only one process ever holds the port at a
  // time.
  bool open_ender() {
    ender_fd = open(kEnderPort, O_RDWR | O_NOCTTY);
    if (ender_fd < 0) {
      RCLCPP_WARN(this->get_logger(),
        "Print setup not connected (could not open %s), continuing without real printing",
        kEnderPort);
      return false;
    }
    struct termios tty;
    tcgetattr(ender_fd, &tty);
    cfsetspeed(&tty, B115200);
    tcsetattr(ender_fd, TCSANOW, &tty);
    sleep(2);
    // Discard the boot banner so the first send_ender() call can't match
    // stale text instead of the real reply to the first command sent.
    tcflush(ender_fd, TCIFLUSH);
    RCLCPP_INFO(this->get_logger(), "Ender serial open");
    return true;
  }

  // Releases the port so printer.py can cleanly claim it again for the
  // next print job instead of fighting trajectory for it.
  void close_ender() {
    if (ender_fd >= 0) {
      close(ender_fd);
      ender_fd = -1;
    }
    ender_ready = false;
  }

 
  // Blocks until the Ender3 acks the given gcode line with "ok". Only ever
  // called when ender_ready is true (ender_fd is a live connection) --
  // calling this with ender_fd == -1 would spin/hang forever.
  void send_ender(const std::string& cmd) {
    std::string line = cmd + "\n";
    write(ender_fd, line.c_str(), line.size());
    char buf[64];
    std::string response;
    while (response.find("ok") == std::string::npos) {
      int n = read(ender_fd, buf, sizeof(buf));
      if (n > 0){
        response += std::string(buf, n);
      }
    }
    RCLCPP_INFO(this->get_logger(), "send_ender(\"%s\") -> \"%s\"", cmd.c_str(), response.c_str());
  }
  // ################################ End Citation[]###############################

  // Function to execute a trajectory plan and trace the end-effector path
  void execute_with_trace(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                           bool is_print) {
    if (use_extruder) {
      threshold = 0.008;
      end_effector_link = "nozzle";
    } 
    else {
      threshold = 0.0025;
      end_effector_link = "link_6__flange";
    }

    if (use_mock_hardware) {
      // Mock hardware never publishes /joint_states during execution — use FK on plan waypoints
      auto robot_state = std::make_shared<moveit::core::RobotState>(move_group->getRobotModel());
      
      std::vector<std::pair<double, geometry_msgs::msg::Point>> timed_points;
      for (const auto& points : plan.trajectory.joint_trajectory.points) { // for pt in jt.points
        for (size_t j = 0; j < plan.trajectory.joint_trajectory.joint_names.size(); j++) {
          robot_state->setJointPositions(plan.trajectory.joint_trajectory.joint_names[j], &points.positions[j]); // set joint positions
        }
        robot_state->updateLinkTransforms(); // update tf so it does fk

        const Eigen::Isometry3d& ee_tf = robot_state->getGlobalLinkTransform(end_effector_link);
       // Compute the tip position in the world frame by adding the threshold along the end-effector's local +Z axis
        Eigen::Vector3d tip = ee_tf.translation() + ee_tf.rotation() * Eigen::Vector3d(0, 0, threshold);
        double t = points.time_from_start.sec + points.time_from_start.nanosec * 1e-9;
        geometry_msgs::msg::Point p;
        p.x = tip.x();
        p.y = tip.y();
        p.z = tip.z();
        timed_points.push_back({t, p});
      }
      auto joint_trajectory_copy = plan.trajectory.joint_trajectory;
      std::thread tracer([this, timed_points, is_print, joint_trajectory_copy]() {
        auto exec_start = std::chrono::steady_clock::now();
        visualization_msgs::msg::Marker marker_ee;
        marker_ee.header.frame_id = "world";
        marker_ee.ns = "ee_trace";
        marker_ee.id = 0;
        marker_ee.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_ee.action = visualization_msgs::msg::Marker::ADD;
        marker_ee.scale.x = 0.005;
        marker_ee.color.r = 0.0f;
        marker_ee.color.g = 1.0f;
        marker_ee.color.b = 0.0f;
        marker_ee.color.a = 1.0f;
        marker_ee.lifetime = rclcpp::Duration(0, 0);
        visualization_msgs::msg::Marker marker_print;
        marker_print.header.frame_id = "world";
        marker_print.ns = "print_trace";  marker_print.id = print_seg_id;
        marker_print.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_print.action = visualization_msgs::msg::Marker::ADD;
        marker_print.scale.x = 0.005;
        marker_print.color.r = 0.5f;
        marker_print.color.g = 0.0f;
        marker_print.color.b = 0.5f;
        marker_print.color.a = 1.0f;
        marker_print.lifetime = rclcpp::Duration(0, 0);

        for (size_t idx = 0; idx < timed_points.size(); idx++) {
          const auto& [t, p] = timed_points[idx];
          std::this_thread::sleep_until(exec_start + std::chrono::duration<double>(t));
          // lock_guard ensures that the last_joint_state is not being modified while we are writing it to publish
          {
            std::lock_guard<std::mutex> lock(last_joint_state_mutex);
            last_joint_state.name = joint_trajectory_copy.joint_names;
            last_joint_state.position = std::vector<double>(
              joint_trajectory_copy.points[idx].positions.begin(),
              joint_trajectory_copy.points[idx].positions.end());
          }
          trace_points.push_back(p);
          marker_ee.header.stamp = this->now();
          marker_ee.points = trace_points;
          trace_pub->publish(marker_ee);
          if (is_print) {
            print_trace_points.push_back(p);
            marker_print.header.stamp = this->now();
            marker_print.points = print_trace_points;
            print_trace_pub->publish(marker_print);
          }
        }
      });
      move_group->execute(plan);
      tracer.join();
    } else {
      // Real hardware: /joint_states updates live — poll getCurrentPose at 20 Hz     
      double dur_sec = plan.trajectory.joint_trajectory.points.empty() ? 5.0 :
          plan.trajectory.joint_trajectory.points.back().time_from_start.sec + 
          plan.trajectory.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;

      auto stop_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(dur_sec + 0.2);
      std::thread tracer([this, stop_time, is_print]() {
        visualization_msgs::msg::Marker marker_ee;
        marker_ee.header.frame_id = "world";
        marker_ee.ns = "ee_trace";  
        marker_ee.id = 0;
        marker_ee.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_ee.action = visualization_msgs::msg::Marker::ADD;
        marker_ee.scale.x = 0.002;
        marker_ee.color.r = 0.0f;  
        marker_ee.color.g = 1.0f;
        marker_ee.color.b = 0.0f;  
        marker_ee.color.a = 1.0f;
        marker_ee.lifetime = rclcpp::Duration(0, 0);
        visualization_msgs::msg::Marker marker_print;
        marker_print.header.frame_id = "world";
        marker_print.ns = "print_trace";
        marker_print.id = print_seg_id;
        marker_print.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_print.action = visualization_msgs::msg::Marker::ADD;
        marker_print.scale.x = 0.002;
        marker_print.color.r = 0.5f;  
        marker_print.color.g = 0.0f;
        marker_print.color.b = 0.5f;  
        marker_print.color.a = 1.0f;
        marker_print.lifetime = rclcpp::Duration(0, 0);

        while (std::chrono::steady_clock::now() < stop_time) {
          auto pose = move_group->getCurrentPose(end_effector_link);
          Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x,
                               pose.pose.orientation.y, pose.pose.orientation.z);
          Eigen::Vector3d origin(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
          // nozzle frame is an estimated center; true tip sits 8mm further along
          // the same direction the tool points (local +Z)
          Eigen::Vector3d tip = origin + q.toRotationMatrix() * Eigen::Vector3d(0, 0, threshold);
          geometry_msgs::msg::Point p;
          p.x = tip.x();
          p.y = tip.y();
          p.z = tip.z();
          trace_points.push_back(p);
          marker_ee.header.stamp = this->now();
          marker_ee.points = trace_points;
          trace_pub->publish(marker_ee);
          if (is_print) {
            print_trace_points.push_back(p);
            marker_print.header.stamp = this->now();
            marker_print.points = print_trace_points;
            print_trace_pub->publish(marker_print);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      });
      move_group->execute(plan);
      tracer.join();
    }

  }

  bool recover_point(Trajectory& batch, size_t k, MoveType type) {
    Trajectories& pt = batch.at(k);
    const std::string original_cmd = pt.cmd;  // pt.cmd may get demoted to G1 below
    auto remembered_state = *move_group->getCurrentState();
    auto remembered_pose = move_group->getCurrentPose(end_effector_link).pose;

    geometry_msgs::msg::Pose retreat_pose = remembered_pose;
    retreat_pose.position.z += 0.01;
    move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group->setPlannerId("LIN");
    move_group->setMaxVelocityScalingFactor(0.2);
    move_group->setMaxAccelerationScalingFactor(0.1);
    move_group->setPoseTarget(retreat_pose, end_effector_link);
    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    if (move_group->plan(retreat_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      execute_with_trace(retreat_plan, false);
    }

    bool found = false;
    moveit::planning_interface::MoveGroupInterface::Plan saved_plan;

    if (pt.cmd == "G2" || pt.cmd == "G3") {
      moveit_msgs::msg::Constraints constraints;
      constraints.name = "interim";
      moveit_msgs::msg::PositionConstraint pos_constraint;
      // Must match the goal pose's frame (getPlanningFrame(), not a
      // hardcoded "world") -- Pilz resolves this via
      // scene->getFrameTransform() to place the interim point before
      // computing the arc's plane, and a name it can't resolve silently
      // falls back to identity instead of erroring outright.
      pos_constraint.header.frame_id = move_group->getPlanningFrame();
      pos_constraint.link_name = end_effector_link;
      pos_constraint.constraint_region.primitive_poses.push_back(pt.mid_pt);
      pos_constraint.weight = 1.0;
      constraints.position_constraints.push_back(pos_constraint);

      move_group->setStartState(remembered_state);
      move_group->setPathConstraints(constraints);
      move_group->setPlannerId("CIRC");
      move_group->setMaxVelocityScalingFactor(0.2);
      move_group->setMaxAccelerationScalingFactor(0.05);
      move_group->setPoseTarget(pt.pose.pose, end_effector_link);
      found = (move_group->plan(saved_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      move_group->clearPathConstraints();

      if (!found) {
        // Same cross-product Pilz's own MAX_COLINEAR_NORM check runs
        // (path_circle_generator.hpp) -- computed here independently so a
        // failure can be checked by hand against what our own build-time
        // validity check (2.5e-9) already passed it on.
        Eigen::Vector3d start_v(remembered_pose.position.x, remembered_pose.position.y, remembered_pose.position.z);
        Eigen::Vector3d mid_v(pt.mid_pt.position.x, pt.mid_pt.position.y, pt.mid_pt.position.z);
        Eigen::Vector3d target_v(pt.pose.pose.position.x, pt.pose.pose.position.y, pt.pose.pose.position.z);
        double cn = (mid_v - start_v).cross(target_v - start_v).norm();

        RCLCPP_WARN(this->get_logger(),
          "Point %zu: arc unrecoverable, demoting to G1 -- start=[%.6f,%.6f,%.6f] mid=[%.6f,%.6f,%.6f] "
          "target=[%.6f,%.6f,%.6f] cross_norm=%.3e",
          k, start_v.x(), start_v.y(), start_v.z(), mid_v.x(), mid_v.y(), mid_v.z(),
          target_v.x(), target_v.y(), target_v.z(), cn);
        ik_failure_log.push_back({target_v, original_cmd, start_v, true, mid_v, cn,
                                    "arc unrecoverable (Pilz rejected -- see cross_norm), demoted to G1"});
        pt.cmd = "G1";
      }
    }

    if (!found) {
      move_group->setStartState(remembered_state);
      move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group->setPlannerId("LIN");
      move_group->setMaxVelocityScalingFactor(0.2);
      move_group->setMaxAccelerationScalingFactor(0.1);
      move_group->setPoseTarget(pt.pose.pose, end_effector_link);
      found = (move_group->plan(saved_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    move_group->setStartStateToCurrentState();  // don't leak into future plans

    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Point %zu FAILED (no recovery possible)", k);
      Eigen::Vector3d start_v(remembered_pose.position.x, remembered_pose.position.y, remembered_pose.position.z);
      Eigen::Vector3d target_v(pt.pose.pose.position.x, pt.pose.pose.position.y, pt.pose.pose.position.z);
      Eigen::Vector3d mid_v(pt.mid_pt.position.x, pt.mid_pt.position.y, pt.mid_pt.position.z);
      bool was_arc = (original_cmd == "G2" || original_cmd == "G3");
      double cn = was_arc ? (mid_v - start_v).cross(target_v - start_v).norm() : 0.0;
      ik_failure_log.push_back({target_v, original_cmd, start_v, was_arc, mid_v, cn,
                                  "G1 (demoted or original) also failed -- no recovery possible"});
      return false;
    }

    // Genuinely get back to the remembered start before running saved_plan
    // -- it was planned assuming that's where the robot is.
    move_group->setPlannerId("LIN");
    move_group->setMaxVelocityScalingFactor(0.2);
    move_group->setMaxAccelerationScalingFactor(0.1);
    move_group->setPoseTarget(remembered_pose, end_effector_link);
    moveit::planning_interface::MoveGroupInterface::Plan return_plan;
    if (move_group->plan(return_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      Eigen::Vector3d start_v(remembered_pose.position.x, remembered_pose.position.y, remembered_pose.position.z);
      ik_failure_log.push_back({start_v, original_cmd, start_v, false, start_v, 0.0,
                                  "could not return to remembered start after recovery -- point never executed"});
      return false;
    }
    execute_with_trace(return_plan, false);  // now genuinely at the plan's assumed start

    bool extruding = use_extruder && ender_ready && std::abs(pt.e) > 1e-6;
    if (extruding) {
      auto& jt = saved_plan.trajectory.joint_trajectory.points;
      double dur = jt.empty() ? 1.0 : jt.back().time_from_start.sec + jt.back().time_from_start.nanosec * 1e-9;
      if (dur < 0.05) dur = 0.05;
      double e_val = pt.e * extrusion_multiplier;
      double matched_f = (std::abs(e_val) / dur) * 60.0;
      std::thread extrude_thread([this, e_val, matched_f]() {
        send_ender("G1 E" + std::to_string(e_val) + " F" + std::to_string(matched_f));
      });
      execute_with_trace(saved_plan, type == MoveType::Print);
      extrude_thread.join();
    } else {
      execute_with_trace(saved_plan, type == MoveType::Print);
    }
    return true;
  }

  void gcode_file_callback(
      const std::shared_ptr<meca500_demo::srv::GcodeFile::Request>  req,
            std::shared_ptr<meca500_demo::srv::GcodeFile::Response> res)
  {
    // Read file
    std::ifstream file(req->file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", req->file_path.c_str());
      res->success = false;
      res->message = "Cannot open file: " + req->file_path;
      res->moves_sent = 0;
      return;
    }

    std::string gcode((std::istreambuf_iterator<char>(file)),
                       std::istreambuf_iterator<char>());
    file.close();

    RCLCPP_INFO(this->get_logger(), "Loaded file: %s (%zu bytes)",
                req->file_path.c_str(), gcode.size());

    // Reuse goal_callback by building a Goal request
    auto goal_req = std::make_shared<meca500_demo::srv::Goal::Request>();
    auto goal_res = std::make_shared<meca500_demo::srv::Goal::Response>();
    goal_req->gcode = gcode;

    goal_callback(goal_req, goal_res);

    res->success  = goal_res->success;
    res->message  = goal_res->success ? "OK" : "goal_callback failed";
    res->moves_sent = static_cast<int32_t>(total_moves);
  }
  
  // This function creates a table pose matrix from the table's position and orientation parameters
  // and then uses it to transform G-code coordinates from the table frame to the robot frame. 
  // It also computes the end-effector orientation based on the table's normal vector, ensuring that 
  // the end-effector approaches the table perpendicularly. The function then plans and executes a 
  // trajectory for each G-code move, handling both linear and arc movements, and scales the feed rate as necessary.
  
  void goal_callback(
    const std::shared_ptr<meca500_demo::srv::Goal::Request>  request,
          std::shared_ptr<meca500_demo::srv::Goal::Response> response) {

    if (use_extruder && real_print) {
      ender_ready = open_ender();
    }

    // Parse gcode
    gcode::Program program = gcode::parse(request->gcode);
    RCLCPP_INFO(this->get_logger(), "Parsed %zu G moves", program.size());
    total_moves = program.size();

    // Assert whichever extrusion mode gcode_parser.py actually declared in
    // the file (OUTPUT_RELATIVE_E there, read back via gcode.cpp's M82/M83
    // handling) instead of hardcoding an assumption independently here --
    // this is the single source of truth for what the per-move G1 E
    // deltas below assume.
    if (ender_ready) {
      send_ender(program.e_relative_mode ? "M83" : "M82");
    }

    // getting the table's normal vector and center position in the world frame
    Eigen::Quaterniond q_table(qw, qx, qy, qz); // table orientation
    q_table.normalize();
    Eigen::Matrix3d R_table = q_table.toRotationMatrix(); // rotation matrix of the table
    Eigen::Vector3d table_center(x, y, z); // center of the table// vector from ee to table center

    auto ee_pose = move_group->getCurrentPose(end_effector_link).pose;
    Eigen::Vector3d ee_to_table_vector(table_center.x() - ee_pose.position.x,
                                       table_center.y() - ee_pose.position.y,
                                       table_center.z() - ee_pose.position.z);

    // if the table normal is opposite to the vector from ee to table center, flip the normal
    if (R_table.col(2).dot(ee_to_table_vector) > 0) {
      R_table.col(2) = -R_table.col(2);
      q_table = Eigen::Quaterniond(R_table);
      q_table.normalize();
    }

    // Form the table's transformation matrix from its rotation and translation
    // Build T explicitly from the flipped right-handed frame
    Eigen::Matrix3d R_table_final = Eigen::Matrix3d::Identity();



    // Ensure right-handed: z = x cross y
    if ((R_table.col(0).cross(R_table.col(1)) - R_table.col(2)).norm() > 0.1) {
      R_table.col(1) = -R_table.col(1);  // flip y to make it right-handed
    }

    R_table_final.col(0) = R_table.col(0);  // x-axis
    R_table_final.col(1) = R_table.col(1);  // y-axis
    R_table_final.col(2) = R_table.col(2);  // z-axis (table normal)

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R_table_final;
    T.block<3,1>(0,3) = table_center;

    // Force the end-effector's z-axis to point INTO the table (opposite the build direction)
    Eigen::Vector3d z_ee = -R_table_final.col(2);  // nozzle points INTO the table, not out
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

    // Fill trajectory_batches now that ee_orient is known -- every point's
    // target pose (and, for arcs, interim point) gets resolved once here.
    Eigen::Vector3d tool_offset = ee_orient.toRotationMatrix() * Eigen::Vector3d(0, 0, -threshold);

    //==============================================================================================
    //                               Build trajectory_batches
    //==============================================================================================

    trajectory_batches.clear();
    Eigen::Vector3d prev_pose(0.0, 0.0, 0.0);  // previous move's target position (post tool_offset)
    bool have_prev = false;

    for (const auto& move : program.moves) {

      double pt_z = move.z / 1000.0;
      // setting minimum z to 1mm above the table to avoid collision with the table
      if (pt_z < 0.001) {
        pt_z = 0.001;
      }
      // get the target position in robot frame
      Eigen::Vector4d P_table(move.x / 1000.0, move.y / 1000.0, pt_z, 1.0);
      Eigen::Vector4d P_robot = T * P_table;
      Eigen::Vector4d IJ_table(move.i / 1000.0, move.j / 1000.0, 0.0, 0.0);
      Eigen::Vector4d IJ_robot = T * IJ_table; // I, J in robot frame

      Trajectories target;
      target.pose.header.frame_id = move_group->getPlanningFrame();
      target.pose.pose.position.x = P_robot.x() + tool_offset.x();
      target.pose.pose.position.y = P_robot.y() + tool_offset.y();
      target.pose.pose.position.z = P_robot.z() + tool_offset.z();
      target.pose.pose.orientation.x = ee_orient.x();
      target.pose.pose.orientation.y = ee_orient.y();
      target.pose.pose.orientation.z = ee_orient.z();
      target.pose.pose.orientation.w = ee_orient.w();
      target.e = move.e;
      target.f = move.f;
      target.cmd = move.cmd;
      if (!have_prev) {
        f_max = move.f;
        f_min = move.f;
      } else {
        if (move.f > f_max) f_max = move.f;
        if (move.f < f_min) f_min = move.f;
      }

      if (target.cmd == "G2" || target.cmd == "G3") {
        if (!have_prev) {
          target.cmd = "G1";  // first move of the print can't be an arc -- no anchor
        }
        else {
          Eigen::Vector2d arc_center(prev_pose.x() + IJ_robot.x(), prev_pose.y() + IJ_robot.y());
          bool valid = true;

          if (valid) {
            // angle between the start and end points of the arc
            // measured from the center of the arc
            double a0 = atan2(prev_pose.y() - arc_center.y(),
                              prev_pose.x() - arc_center.x());
            double a1 = atan2(target.pose.pose.position.y - arc_center.y(),
                              target.pose.pose.position.x - arc_center.x());

            // G2 is clockwise, G3 is counterclockwise
            // Angle wrapping to make sure angle wrapping is handled correctly

            double diff = (target.cmd == "G2") ? (a0 - a1) : (a1 - a0);
            if (diff < 0) {
              diff += 2 * M_PI;
            }
             // since in slicer i made sure none of the arcs are more than 180 degrees
            if (diff*180.0/M_PI > 185.0) {
              valid = false;
            }


          }
          if (valid) {
            Eigen::Vector3d target_pos(target.pose.pose.position.x, target.pose.pose.position.y,
                                        target.pose.pose.position.z);
            Eigen::Vector3d center(arc_center.x(), arc_center.y(), prev_pose.z());
            Eigen::Vector3d mid_point = get_arc_center(prev_pose, target_pos, center, target.cmd);

            // threshold 2.5e-11 = 5um x 5um = robot resolution limit
            if (((mid_point - prev_pose).cross(target_pos - prev_pose)).norm() < 2.5e-11){
              valid = false;
            }

            if (valid) {
              target.mid_pt.position.x = mid_point.x();
              target.mid_pt.position.y = mid_point.y();
              target.mid_pt.position.z = mid_point.z();
            }
          }

          if (!valid) {
            target.cmd = "G1";
          }
        }
      }

      prev_pose.x() = target.pose.pose.position.x;
      prev_pose.y() = target.pose.pose.position.y;
      prev_pose.z() = target.pose.pose.position.z;
      have_prev = true;


      // writing if the movetype was a print, retract or none (pure travel)
      MoveType type;
      if (target.e > 1e-6) {
        type = MoveType::Print;
      }
      else if (target.e < -1e-6) {
        type = MoveType::Retract;
      }
      else {
        type = MoveType::None;
      }

      // Only start a new batch on a genuine type change now -- a long run
      // of consecutive same-type points (e.g. one continuous printed wall)
      // stays as one batch instead of being artificially chopped into
      // fixed-size chunks that then each pay their own plan/execute
      // overhead for no real reason, since they were never separate moves
      // to begin with.
      if (trajectory_batches.empty() || trajectory_batches.back().type != type) {
        trajectory_batches.push_back({{}, type});
      }
      trajectory_batches.back().trajectory.push_back(target);
    }

    response->success = true;

    // Plans batch[start, end) as one blended MotionSequenceRequest -- G1
    // points become LIN items, G2/G3 points become CIRC items with their
    // precomputed mid_pt carried as that item's own path_constraints
    // (each item plans as if it were solitary, so the interim-point
    // constraint has to travel with the item, not with move_group).
    // Planning only -- doesn't execute anything, so it's safe to call
    // while the arm is parked mid-transition, pre-planning the next
    // batch, or before the arm has even left home during reconnaissance.
    auto plan_batch = [this](Trajectory& batch, size_t start, size_t end,
                              moveit::planning_interface::MoveGroupInterface::Plan& out_plan,
                              const moveit::core::RobotState* start_override = nullptr) -> bool {
      if (!action_sequence_client->action_server_is_ready() || end <= start) return false;

      moveit_msgs::msg::MotionSequenceRequest seq_req;
      // Cartesian reference for seg_dist below. Normally the live robot
      // pose -- but when start_override is given (chaining plan-only
      // calls during the reconnaissance pass, where nothing is actually
      // executing), the live pose never changes between batches, so use
      // FK on the override instead of querying a robot that isn't moving.
      Eigen::Vector3d prev_ee_pose;
      if (start_override != nullptr) {
        const Eigen::Isometry3d& ee_tf = start_override->getGlobalLinkTransform(end_effector_link);
        prev_ee_pose = ee_tf.translation();
      } else {
        auto current_ee_pose = move_group->getCurrentPose(end_effector_link).pose;
        prev_ee_pose = Eigen::Vector3d(current_ee_pose.position.x,
                                        current_ee_pose.position.y,
                                        current_ee_pose.position.z);
      }
      std::vector<double> seg_dist(end - start, 0.0);

      for (size_t i = start; i < end; i++) {
        const auto& pt = batch.at(i);
        size_t idx = i - start;
        Eigen::Vector3d p(pt.pose.pose.position.x, pt.pose.pose.position.y, pt.pose.pose.position.z);
        seg_dist.at(idx) = (p - prev_ee_pose).norm();
        prev_ee_pose = p;

        double vel = f_scaling(f_max, f_min, pt.f, true);

        moveit_msgs::msg::MotionSequenceItem item;
        item.req.group_name = move_group->getName();
        item.req.pipeline_id = "pilz_industrial_motion_planner";
        item.req.max_velocity_scaling_factor = vel;
        item.req.max_acceleration_scaling_factor = 0.05;
        item.req.goal_constraints.push_back(
          kinematic_constraints::constructGoalConstraints(end_effector_link, pt.pose));

        // First item only: if the caller passed a remembered state (used
        // when pre-planning while parked away from where this batch will
        // actually resume from), anchor planning to THAT instead of
        // letting it default to wherever the arm physically is right now
        // -- otherwise the plan comes out assuming the wrong start and
        // fails MoveIt's start-tolerance check once the arm is back in
        // position and we go to execute it.
        if (i == start && start_override != nullptr) {
          item.req.start_state.is_diff = false;
          moveit::core::robotStateToRobotStateMsg(*start_override, item.req.start_state);
        }

        if (pt.cmd == "G2" || pt.cmd == "G3") {
          item.req.planner_id = "CIRC";
          moveit_msgs::msg::Constraints constraints;
          constraints.name = "interim";
          moveit_msgs::msg::PositionConstraint pos_constraint;
          pos_constraint.header.frame_id = "world";
          pos_constraint.link_name = end_effector_link;
          pos_constraint.constraint_region.primitive_poses.push_back(pt.mid_pt);
          pos_constraint.weight = 1.0;
          constraints.position_constraints.push_back(pos_constraint);
          item.req.path_constraints = constraints;
        } else {
          item.req.planner_id = "LIN";
        }

        if (i > start && i < end - 1) {
          // Blend radius is 30% of the smaller of the two adjacent segment lengths, capped at 2mm
          item.blend_radius = std::min(std::min(seg_dist.at(idx), seg_dist.at(idx + 1)) * 0.3, 0.002);
        }
        seq_req.items.push_back(item);
      }

      auto goal_msg = moveit_msgs::action::MoveGroupSequence::Goal();
      goal_msg.request = seq_req;
      // plan_only -- same contract the GetMotionSequence service had;
      // execute_batch() below still drives actual execution through
      // execute_with_trace() for trace publishing + matched extrusion.
      goal_msg.planning_options.plan_only = true;
      goal_msg.planning_options.planning_scene_diff.is_diff = true;
      goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;

      auto goal_handle = action_sequence_client->async_send_goal(goal_msg).get();
      if (!goal_handle) {
        return false;  // rejected
      }

      auto wrapped_result = action_sequence_client->async_get_result(goal_handle).get();
      if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
          wrapped_result.result->response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
          wrapped_result.result->response.planned_trajectories.empty()) {
        return false;
      }
      out_plan.trajectory = wrapped_result.result->response.planned_trajectories.back();
      return true;
    };

    // Reconnaissance pass: plan-only (goal_msg.planning_options.plan_only
    // is already true in plan_batch(), so nothing here ever executes)
    // through every batch once, from wherever the arm currently is (home)
    // -- run before the approach-to-bed sequence below, since this never
    // touches the real robot anyway, no reason to wait at the bed for it.
    // Purely to discover the real range of matched_f this print will
    // produce -- needed to rescale extrusion feed rate into the
    // extruder's real safe range (see execute_batch() further down)
    // instead of a fixed floor that flattens most of the print to one
    // flat rate, since most segments here are accel-limited and
    // naturally cluster low. A batch that fails to plan here is just
    // skipped for statistics -- this never touches execution or
    // recovery, the real print's own recovery logic handles it properly
    // when it actually runs.
    // Plan (never execute) the standoff hover + move-to-first-point
    // sequence below, purely to get a realistic starting joint state for
    // the reconnaissance pass. Starting reconnaissance from the real
    // current state (home) meant its first batch planned a huge,
    // unrepresentative jump straight from home to the first gcode point
    // -- different enough from what real execution actually does (which
    // starts from the first gcode point, already reached via this same
    // approach sequence) that it triggered joint-velocity-limit failures
    // cascading through nearly every batch. This mirrors the real
    // approach sequence exactly, just without execute_with_trace(), so
    // the arm never actually moves here -- the real approach sequence
    // (further below, inside if (!moved_to_bed)) still runs for real
    // after reconnaissance finishes.
    moveit::core::RobotState expected_state = *move_group->getCurrentState();
    expected_state.updateLinkTransforms();
    {
      bool standoff_planned = false;
      for (double t = 0.05; t <= 0.35; t += 0.02) {
        Eigen::Vector3d p = table_center + t * R_table_final.col(2);
        geometry_msgs::msg::Pose target;
        target.position.x = p.x();
        target.position.y = p.y();
        target.position.z = p.z();
        target.orientation.x = ee_orient.x();
        target.orientation.y = ee_orient.y();
        target.orientation.z = ee_orient.z();
        target.orientation.w = ee_orient.w();

        move_group->setPlanningPipelineId("ompl");
        move_group->setPlannerId("RRTConnect");
        move_group->setPlanningTime(10.0);
        move_group->setMaxVelocityScalingFactor(0.5);
        move_group->setMaxAccelerationScalingFactor(0.5);
        move_group->setPoseTarget(target, end_effector_link);

        moveit::planning_interface::MoveGroupInterface::Plan standoff_plan;
        if (move_group->plan(standoff_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          auto& jt = standoff_plan.trajectory.joint_trajectory;
          if (!jt.points.empty()) {
            for (size_t k = 0; k < jt.joint_names.size(); k++) {
              expected_state.setJointPositions(jt.joint_names[k], &jt.points.back().positions[k]);
            }
            expected_state.updateLinkTransforms();
          }
          standoff_planned = true;
          break;
        }
        move_group->clearPoseTargets();
      }

      if (standoff_planned && !trajectory_batches.empty() && !trajectory_batches.front().trajectory.empty()) {
        const geometry_msgs::msg::Pose& target = trajectory_batches.front().trajectory.front().pose.pose;
        move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group->setPlannerId("LIN");
        move_group->setPlanningTime(10.0);
        move_group->setMaxVelocityScalingFactor(0.5);
        move_group->setMaxAccelerationScalingFactor(0.5);
        move_group->setStartState(expected_state);
        move_group->setPoseTarget(target, end_effector_link);
        moveit::planning_interface::MoveGroupInterface::Plan first_point_plan;
        if (move_group->plan(first_point_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          auto& jt = first_point_plan.trajectory.joint_trajectory;
          if (!jt.points.empty()) {
            for (size_t k = 0; k < jt.joint_names.size(); k++) {
              expected_state.setJointPositions(jt.joint_names[k], &jt.points.back().positions[k]);
            }
            expected_state.updateLinkTransforms();
          }
        }
      }
      // Undo the setStartState() override above -- nothing in this block
      // actually executed, so the real approach sequence further below
      // must still plan from the real current state (home), not this
      // fictional "expected" one.
      move_group->setStartStateToCurrentState();
      move_group->clearPoseTargets();
    }

    {
      RCLCPP_INFO(this->get_logger(), "=== Reconnaissance pass: %zu batches ===", trajectory_batches.size());
      auto virtual_state = expected_state;
      matched_f_min_seen = -1.0;
      matched_f_max_seen = -1.0;
      size_t recon_idx = 0;
      for (auto& tb : trajectory_batches) {
        recon_idx++;
        RCLCPP_INFO(this->get_logger(), "Reconnaissance: planning batch %zu/%zu",
          recon_idx, trajectory_batches.size());
        auto& batch = tb.trajectory;
        if (batch.empty()) continue;
        moveit::planning_interface::MoveGroupInterface::Plan recon_plan;
        if (!plan_batch(batch, 0, batch.size(), recon_plan, &virtual_state)) {
          RCLCPP_WARN(this->get_logger(), "Reconnaissance: batch %zu/%zu failed to plan, skipping for statistics",
            recon_idx, trajectory_batches.size());
          // Still advance virtual_state to roughly where this batch's own
          // points intended to end, via IK on the last target seeded from
          // the current state -- otherwise the next batch gets planned
          // assuming the arm is still wherever it was several batches
          // ago, a huge unintended jump that's virtually guaranteed to
          // also fail. Left unfixed, one failure cascades through every
          // remaining batch in the print (this is what happened: the
          // whole pass came back with no usable data at all).
          const auto* jmg = move_group->getRobotModel()->getJointModelGroup(move_group->getName());
          if (virtual_state.setFromIK(jmg, batch.back().pose.pose)) {
            virtual_state.updateLinkTransforms();
          }
          continue;
        }

        if (tb.type != MoveType::None) {
          double e_sum = 0.0;
          for (auto& pt : batch) e_sum += pt.e * extrusion_multiplier;
          auto& jt = recon_plan.trajectory.joint_trajectory.points;
          double dur = jt.empty() ? 1.0 : jt.back().time_from_start.sec + jt.back().time_from_start.nanosec * 1e-9;
          if (dur < 0.05) dur = 0.05;
          double raw_matched_f = (std::abs(e_sum) / dur) * 60.0;
          if (matched_f_min_seen < 0.0 || raw_matched_f < matched_f_min_seen) matched_f_min_seen = raw_matched_f;
          if (raw_matched_f > matched_f_max_seen) matched_f_max_seen = raw_matched_f;
        }

        auto& jt = recon_plan.trajectory.joint_trajectory;
        if (!jt.points.empty()) {
          for (size_t k = 0; k < jt.joint_names.size(); k++) {
            virtual_state.setJointPositions(jt.joint_names[k], &jt.points.back().positions[k]);
          }
          virtual_state.updateLinkTransforms();
        }
      }
      RCLCPP_INFO(this->get_logger(), "=== Reconnaissance done: matched_f range [%.2f, %.2f] mm/min ===",
        matched_f_min_seen, matched_f_max_seen);
    }

    if (!moved_to_bed) {
      // Home -> table approach is never a print move -- is_print defaults
      // to true at construction and nothing sets it false before this is
      // the very first thing goal_callback() ever executes, so without
      // this the standoff hover + move-to-first-point sequence gets drawn
      // into the purple print trace even though nothing has been printed yet.
      is_print = false;
      bool above_bed_hover_success = false;

      // for the 1st goal point (basically hovering above table center)
      // draw a line from the table surface (t=0) to a point above the table (t=0.35) passing
      // through the goal point, parallel to table normal
      // try to plan and execute a trajectory for each point along the line until one succeeds

      for (double t = 0.05; t <= 0.35; t += 0.02) {
        // Standoff moves OUT from the table center, along the build direction (table_normal)
        // which for your pose also moves toward the base
        Eigen::Vector3d p = table_center + t * R_table_final.col(2);

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

        move_group->setPlanningPipelineId("ompl");
        move_group->setPlannerId("RRTConnect");
        move_group->setPlanningTime(10.0);
        move_group->setMaxVelocityScalingFactor(0.5);
        move_group->setMaxAccelerationScalingFactor(0.5);
        move_group->setPoseTarget(target, end_effector_link);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "SUCCESS at t=%.2f! Executing...", t);
          execute_with_trace(plan, is_print);
          above_bed_hover_success = true;
          // Wait for state monitor to update
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          break;

        }

        move_group->clearPoseTargets();
      }

      // Go straight to the first real gcode point from the standoff, same as
      // the proven KDL-era version -- NOT the table's literal center. Every
      // gcode point (including the first) gets a minimum 1mm clearance above
      // the table during parsing (pt_z clamp above); table_center itself has
      // zero clearance and sits close enough to the frame/bed assembly that
      // the nozzle has no collision-free IK solution there at all, which is
      // what was stalling every print before this reverted back.
      // moved_to_bed only becomes true once BOTH this and the standoff above
      // actually succeeded -- a failed first-point move must not let
      // printing start under the assumption the arm is properly seated.
      if (above_bed_hover_success &&
          !trajectory_batches.empty() && !trajectory_batches.front().trajectory.empty()) {
        const geometry_msgs::msg::Pose& target = trajectory_batches.front().trajectory.front().pose.pose;

        move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group->setPlannerId("LIN");
        move_group->setPlanningTime(10.0);
        move_group->setMaxVelocityScalingFactor(0.5);
        move_group->setMaxAccelerationScalingFactor(0.5);
        move_group->setPoseTarget(target, end_effector_link);
        moveit::planning_interface::MoveGroupInterface::Plan lin_to_first_point_plan;
        if (move_group->plan(lin_to_first_point_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Planning to first gcode point SUCCESS! Executing...");
          execute_with_trace(lin_to_first_point_plan, is_print);
          RCLCPP_INFO(this->get_logger(), "Reached first gcode point!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Could not plan to first gcode point");
          above_bed_hover_success = false;
        }
      }

      moved_to_bed = above_bed_hover_success;
    }

    if (moved_to_bed) {

      // Executes a plan already produced by plan_batch() for batch[start,
      // end), including the matched extrusion command.
      auto execute_batch = [this](moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                   Trajectory& batch, size_t start, size_t end, MoveType type) {
        bool is_print_batch = (type == MoveType::Print);

        // type != None, not just Print: a Retract run still needs its
        // command sent to the printer even though it doesn't draw the purple
        // trace -- same bug class as the earlier per-point retraction-skip fix.
        // ############################ Begin Citation[]#############################################
        if (use_extruder && ender_ready && type != MoveType::None) {
          double e_sum = 0.0;
          for (size_t i = start; i < end; i++) e_sum += batch.at(i).e * extrusion_multiplier;
          auto& jt = plan.trajectory.joint_trajectory.points;
          double dur = jt.empty() ? 1.0 : jt.back().time_from_start.sec + jt.back().time_from_start.nanosec * 1e-9;
          if (dur < 0.05) dur = 0.05;
          double matched_f = (std::abs(e_sum) / dur) * 60.0;
          // Most segments here are accel-limited, not velocity-limited --
          // confirmed empirically, neither raising Pilz's acceleration
          // scaling (caused visible motion jank -- no jerk limiting in
          // Pilz's trapezoidal profile) nor narrowing the slicer's speed
          // range (verified the re-sliced gcode's F values actually
          // changed) moved matched_f. So most raw values cluster low
          // regardless of print settings, and a flat floor would flatten
          // nearly the whole print to one flat rate. Rescale using the
          // real range the reconnaissance pass discovered for this print
          // (see just above the main loop) into this hotend's real safe
          // volumetric range for stock Ender3 + 0.4mm nozzle + PLA
          // (~10-12 mm^3/s before the extruder gear slips), converted to
          // linear feed rate via the 1.75mm filament cross-section
          // (2.4053 mm^2 -- same constant the source slicer itself uses
          // internally) -- preserves relative differences between
          // batches instead of collapsing them to a single number.
          const double kMinFeedRate = 125.0;   // mm/min (5 mm^3/s / 2.4053 mm^2 * 60)
          const double kMaxFeedRate = 175.0;   // mm/min (7 mm^3/s / 2.4053 mm^2 * 60)
          if (matched_f_max_seen > matched_f_min_seen) {
            double t = (matched_f - matched_f_min_seen) / (matched_f_max_seen - matched_f_min_seen);
            t = std::max(0.0, std::min(t, 1.0));
            matched_f = kMinFeedRate + (kMaxFeedRate - kMinFeedRate) * t;
          } else {
            matched_f = std::max(kMinFeedRate, std::min(matched_f, kMaxFeedRate));
          }
          // dur (the plan's own time_from_start) undershoots how long
          // execute_with_trace() actually takes on the real controller --
          // logging both here, next to MoveIt's own "Execute request
          // accepted"/"success" timestamps, so the real gap can be pulled
          // from roslogs and compared instead of guessed at.
          RCLCPP_INFO(this->get_logger(), "Extrude timing: dur=%.4f e_sum=%.4f matched_f=%.2f",
            dur, e_sum, matched_f);
          std::thread extrude_thread([this, e_sum, matched_f]() {
            send_ender("G1 E" + std::to_string(e_sum) + " F" + std::to_string(matched_f));
          });
          execute_with_trace(plan, is_print_batch);
          extrude_thread.join();
        } else {
          execute_with_trace(plan, is_print_batch);
        }
        // ############################ End Citation[]#############################################
      };

      // Drives the print one batch at a time. Within a batch, cursor tracks
      // how far in we've gotten: try the whole remaining [cursor, end) as
      // one blended sequence; if that fails, isolate down to just cursor
      // (a size-1 sequence attempt) to find out whether it's really this
      // point that's the problem; if even that fails, recover_point()
      // pauses, retreats, and runs it point-by-point (demoting an arc to G1
      // if it's truly unreachable). Either way, cursor advances and the
      // loop goes right back to trying the sequence for whatever's left of
      // this batch. Once a whole batch is done, retract + hop 1cm up,
      // pre-plan the NEXT batch while parked there, then hop back down to
      // exactly where this batch left off and hand the pre-built plan to
      // the next iteration so it executes immediately.
      moveit::planning_interface::MoveGroupInterface::Plan pending_plan;
      bool have_pending_plan = false;

      RCLCPP_INFO(this->get_logger(), "=== Starting print: %zu batches ===", trajectory_batches.size());
      auto print_start_time = std::chrono::steady_clock::now();

      for (size_t b = 0; b < trajectory_batches.size(); b++) {
        auto& batch = trajectory_batches[b].trajectory;
        MoveType type = trajectory_batches[b].type;

        if (type == MoveType::Print && !is_print) {
          print_trace_points.clear();
          print_seg_id++;
        }
        is_print = (type == MoveType::Print);

        size_t cursor = 0;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool have_plan;
        if (have_pending_plan) {
          plan = pending_plan;
          have_plan = true;
          have_pending_plan = false;
        } else {
          have_plan = plan_batch(batch, cursor, batch.size(), plan);
        }

        while (cursor < batch.size()) {
          if (have_plan) {
            execute_batch(plan, batch, cursor, batch.size(), type);
            cursor = batch.size();
            break;
          }

          // Whole remaining batch didn't plan -- isolate down to cursor
          // alone to find out whether it's really this point.
          moveit::planning_interface::MoveGroupInterface::Plan single_plan;
          if (plan_batch(batch, cursor, cursor + 1, single_plan)) {
            execute_batch(single_plan, batch, cursor, cursor + 1, type);
          } else if (!recover_point(batch, cursor, type)) {
            response->success = false;
          }
          cursor++;

          if (cursor < batch.size()) {
            have_plan = plan_batch(batch, cursor, batch.size(), plan);
          }
        }

        {
          double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - print_start_time).count();
          RCLCPP_INFO(this->get_logger(), "Batch %zu/%zu complete (%zu points) -- %.1fs elapsed",
            b + 1, trajectory_batches.size(), batch.size(), elapsed);
        }

        // Between batches: retract, hop 1cm up, pre-plan the next batch
        // while parked there, then hop back down to exactly where this
        // batch left off. remembered_state is captured right here, before
        // anything else moves -- it's the joint-space snapshot of exactly
        // where this batch ended, and gets passed to plan_batch() below so
        // the next batch is planned assuming THIS as its start, not
        // wherever the arm physically is once it's hopped up to pause.
        if (b + 1 < trajectory_batches.size()) {
          auto remembered_state = *move_group->getCurrentState();
          // plan_batch() does FK on this via getGlobalLinkTransform() when
          // passed as start_override, which asserts on dirty transforms --
          // same fix as virtual_state in the reconnaissance pass above.
          remembered_state.updateLinkTransforms();
          auto pause_pose = move_group->getCurrentPose(end_effector_link).pose;

          auto& next_batch = trajectory_batches[b + 1].trajectory;
          const auto& next_target = next_batch.front().pose.pose.position;
          double gap = std::sqrt(
            std::pow(next_target.x - pause_pose.position.x, 2) +
            std::pow(next_target.y - pause_pose.position.y, 2) +
            std::pow(next_target.z - pause_pose.position.z, 2));

          // Batches this model actually produces are often just a few points
          // (see build_batches() -- zigzag support/infill legitimately flips
          // MoveType almost every point), so most "batch boundaries" here are
          // a ~0.5mm corner nudge, not a real travel move. Retreating,
          // retracting, and replanning for that is pure overhead. Skip the
          // whole pause cycle when the next batch starts essentially where
          // this one ended.
          const double kSkipPauseDist = 0.003;  // 3mm

          if (gap < kSkipPauseDist) {
            have_pending_plan = plan_batch(next_batch, 0, next_batch.size(), pending_plan, &remembered_state);
          } else {
            if (use_extruder && ender_ready) {
              // Pause-transition retract -- distinct from the gcode's own
              // retract batches, just here so the nozzle doesn't ooze while
              // parked planning the next batch.
              send_ender("G1 E-1 F1200");
            }

            geometry_msgs::msg::Pose hop_pose = pause_pose;
            hop_pose.position.z += 0.01;
            move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group->setPlannerId("LIN");
            move_group->setMaxVelocityScalingFactor(0.2);
            move_group->setMaxAccelerationScalingFactor(0.1);
            move_group->setPoseTarget(hop_pose, end_effector_link);
            moveit::planning_interface::MoveGroupInterface::Plan hop_up_plan;
            if (move_group->plan(hop_up_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
              execute_with_trace(hop_up_plan, false);
            }

            have_pending_plan = plan_batch(next_batch, 0, next_batch.size(), pending_plan, &remembered_state);

            move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group->setPlannerId("LIN");
            move_group->setMaxVelocityScalingFactor(0.2);
            move_group->setMaxAccelerationScalingFactor(0.1);
            move_group->setPoseTarget(pause_pose, end_effector_link);
            moveit::planning_interface::MoveGroupInterface::Plan hop_down_plan;
            if (move_group->plan(hop_down_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
              execute_with_trace(hop_down_plan, false);
            }

            if (use_extruder && ender_ready) {
              // Undo the pause-transition retract above now that we're back
              // down and about to resume real extrusion -- without this the
              // retract is never compensated, and with batches often only a
              // few points long (zigzag support/infill flips type almost
              // every point) it fires often enough to starve the print of
              // forward extrusion almost entirely.
              send_ender("G1 E1 F1200");
            }
          }
        }
      }
      is_print = false;
    }
    print_finished = true;

    RCLCPP_INFO(this->get_logger(), "=== %zu IK/planning failures during this print ===", ik_failure_log.size());
    for (auto& f : ik_failure_log) {
      if (f.was_arc) {
        RCLCPP_INFO(this->get_logger(),
          "  target=[%.6f,%.6f,%.6f] cmd=%s start=[%.6f,%.6f,%.6f] mid=[%.6f,%.6f,%.6f] "
          "cross_norm=%.3e -- %s",
          f.position.x(), f.position.y(), f.position.z(), f.original_cmd.c_str(),
          f.start.x(), f.start.y(), f.start.z(), f.mid_pt.x(), f.mid_pt.y(), f.mid_pt.z(),
          f.cross_norm, f.reason.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(),
          "  target=[%.6f,%.6f,%.6f] cmd=%s start=[%.6f,%.6f,%.6f] -- %s",
          f.position.x(), f.position.y(), f.position.z(), f.original_cmd.c_str(),
          f.start.x(), f.start.y(), f.start.z(), f.reason.c_str());
      }
    }

    if (print_finished) {
      RCLCPP_INFO(this->get_logger(), "Print finished, going back to home position");
      move_group->setPlanningPipelineId("ompl");
      move_group->setPlannerId("RRTConnect");
      move_group->setPlanningTime(5.0);
      move_group->setMaxVelocityScalingFactor(0.5);
      move_group->setMaxAccelerationScalingFactor(0.5);
      move_group->clearPoseTargets();
      move_group->setNamedTarget("Home");

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        execute_with_trace(plan, is_print);
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Could not plan to home position");
      }
    }

    // Release the port now that this print job is done, so printer.py can
    // cleanly claim it again for the next one instead of fighting over it.
    close_ender();

  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Meca500Trajectory>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  node->init();

  spinner.join();
  rclcpp::shutdown();
  return 0;
}