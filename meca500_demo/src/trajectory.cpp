#include <memory>
#include <mutex>
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
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
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
  {}

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

    use_mock_hardware_ = this->get_parameter("use_mock_hardware").as_bool();
    RCLCPP_INFO(this->get_logger(), "Mock hardware: %s", use_mock_hardware_ ? "true" : "false");

    use_extruder_ = this->get_parameter("use_extruder").as_bool();
    RCLCPP_INFO(this->get_logger(), "Extruder: %s", use_extruder_ ? "true" : "false");

    real_print_ = this->get_parameter("real_print").as_bool();
    RCLCPP_INFO(this->get_logger(), "Real print: %s", real_print_ ? "true" : "false");

    //Publisher
    trace_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ee_trace", 10);
    print_trace_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("print_trace", 10);
    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    js_timer_ = this->create_wall_timer(50ms, [this]() {
      std::lock_guard<std::mutex> lock(last_js_mutex_);
      if (!last_js_.name.empty()) {
        last_js_.header.stamp = this->now();
        js_pub_->publish(last_js_);
      }
    }, timer_cb_group_);

    // Subscriber
    RCLCPP_INFO(this->get_logger(), "Ready. Waiting for G-code...");

    table_pos_sub = this->create_subscription<visualization_msgs::msg::Marker>(
      "table_marker", 10,
      [this](const visualization_msgs::msg::Marker::SharedPtr msg) {
        table_callback(msg);
      }
    );

    // Service
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    goal_server = this->create_service<meca500_demo::srv::Goal>(
      "goal_service",
      std::bind(&Meca500Trajectory::goal_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_cb_group_);

    gcode_file_server = this->create_service<meca500_demo::srv::GcodeFile>(
      "gcode_file_service",
      std::bind(&Meca500Trajectory::gcode_file_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(),
      service_cb_group_);

    // End effector link threshold
    if (use_extruder_) {
      threshold = 0.009;
      end_effector_link_ = "nozzle";
    }     
    else {
      threshold = 0.0005;
      end_effector_link_ = "link_6__flange";
    }

    // Deliberately NOT opening /dev/ttyUSB0 here at startup -- printer.py
    // (run later, standalone, between reachability and this node's own
    // gcode_file_service call) needs exclusive use of that same port to
    // heat/prime. Holding it open for this whole node's lifetime meant the
    // two processes fought over one serial port whenever both were live at
    // once. open_ender()/close_ender() below claim and release it only for
    // the duration of an actual print, in goal_callback().

  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr table_pos_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trace_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr print_trace_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  sensor_msgs::msg::JointState last_js_;
  std::mutex last_js_mutex_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::TimerBase::SharedPtr js_timer_;
  std::vector<geometry_msgs::msg::Point> trace_points_;
  std::vector<geometry_msgs::msg::Point> print_trace_points_;
  rclcpp::Service<meca500_demo::srv::Goal>::SharedPtr goal_server;
  rclcpp::Service<meca500_demo::srv::GcodeFile>::SharedPtr gcode_file_server;


 
  struct TrajectoryPoints{
      std::string cmd;
      double x, y, z, i, j, e, f;
      double raw_f;
      bool has_i;
      bool has_j;
      bool has_e;
      bool has_f;
  };

  std::vector<TrajectoryPoints> goal_array;  
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
  double x, y, z, qx, qy, qz, qw;
  bool moved_to_bed_ = false;
  bool use_mock_hardware_ = true;
  std::string end_effector_link_ = "nozzle";
  bool use_extruder_ = true;
  Eigen::Quaterniond ee_orient;
  std::vector<double> mid_point_array{0.0, 0.0, 0.0};
  bool is_print = true;
  int print_seg_id_ = 0;
  bool print_finished = false;
  double threshold = 0.008;

  // Scales every per-move extrusion amount (goal_array.at(i).e) before it's
  // sent to the Ender3 -- tune this if the print is over/under-extruding,
  // rather than touching the gcode generation itself.
  double extrusion_multiplier_ = 1.0;

  int ender_fd_ = -1;
  bool real_print_ = true;
  // True only while a print job currently holds the ender connection open
  // (between open_ender()/close_ender() in goal_callback()) -- distinct
  // from real_print_, which is just the launch-time intent and shouldn't
  // get latched false forever by one job's connection failure.
  bool ender_ready_ = false;


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
  std::vector<double> get_arc_center(std::vector<double> start, std::vector<double> end, std::vector<double> center, std::string command) {
    
    double radius = sqrt(pow(start.at(0) - center.at(0), 2) +
                         pow(start.at(1) - center.at(1), 2) +
                         pow(start.at(2) - center.at(2), 2));

    double start_angle = atan2(start.at(1) - center.at(1), start.at(0) - center.at(0));
    double end_angle = atan2(end.at(1) - center.at(1), end.at(0) - center.at(0));
    double mid_angle;
    double angle_diff;
    std::vector<double> mid_point(3, 0.0);

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

    mid_point.at(0) = center.at(0) + radius * cos(mid_angle);
    mid_point.at(1) = center.at(1) + radius * sin(mid_angle);
    mid_point.at(2) = center.at(2);

    

    return mid_point;
  }
  
  // Function to scale feed rate
  double f_scaling(double f, bool has_f) {    
    double min_f = 1.0; // mm/min
    double max_f = 500.0; // mm/min

    if (has_f) {
      if (f < min_f) {
        f = min_f;
      }
      if (f > max_f) {
        f = max_f;
      }
      double slope = (max_f - f)/(f-min_f);
      double scaled_f = (1.0 + (max_f/min_f) * slope) / (1.0 + slope);
      return scaled_f;
    } else {
      return 0.1;
    }
  }
  // Opens /dev/ttyUSB0 for the duration of one print job -- called at the
  // start of goal_callback(), after printer.py has already run (and closed
  // its own connection) in the reachability -> printer.py -> trajectory
  // pipeline, so only one process ever holds the port at a time.
  bool open_ender() {
    ender_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (ender_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(),
        "Print setup not connected (could not open /dev/ttyUSB0), continuing without real printing");
      return false;
    }
    struct termios tty;
    tcgetattr(ender_fd_, &tty);
    cfsetspeed(&tty, B115200);
    tcsetattr(ender_fd_, TCSANOW, &tty);
    sleep(2);
    // Discard the boot banner so the first send_ender() call can't match
    // stale text instead of the real reply to the first command sent.
    tcflush(ender_fd_, TCIFLUSH);
    RCLCPP_INFO(this->get_logger(), "Ender serial open");
    return true;
  }

  // Releases the port so printer.py can cleanly claim it again for the
  // next print job instead of fighting trajectory for it.
  void close_ender() {
    if (ender_fd_ >= 0) {
      close(ender_fd_);
      ender_fd_ = -1;
    }
    ender_ready_ = false;
  }

  // ################################ Begin Citation[]###############################
  // Blocks until the Ender3 acks the given gcode line with "ok". Only ever
  // called when ender_ready_ is true (ender_fd_ is a live connection) --
  // calling this with ender_fd_ == -1 would spin/hang forever.
  void send_ender(const std::string& cmd) {
    std::string line = cmd + "\n";
    write(ender_fd_, line.c_str(), line.size());
    char buf[64];
    std::string response;
    while (response.find("ok") == std::string::npos) {
      int n = read(ender_fd_, buf, sizeof(buf));
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
    if (use_extruder_) {
      threshold = 0.008;
      end_effector_link_ = "nozzle";
    } 
    else {
      threshold = 0.0025;
      end_effector_link_ = "link_6__flange";
    }

    if (use_mock_hardware_) {
      // Mock hardware never publishes /joint_states during execution — use FK on plan waypoints
      auto robot_state = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());
      const auto& jt = plan.trajectory.joint_trajectory;
      std::vector<std::pair<double, geometry_msgs::msg::Point>> timed_points;
      for (const auto& pt : jt.points) {
        for (size_t j = 0; j < jt.joint_names.size(); j++) {
          robot_state->setJointPositions(jt.joint_names[j], &pt.positions[j]);
        }
        robot_state->updateLinkTransforms();
        const Eigen::Isometry3d& ee_tf = robot_state->getGlobalLinkTransform(end_effector_link_);
        // nozzle frame is an estimated center; true tip sits 8mm further along
        // the same direction the tool points (local +Z)
        Eigen::Vector3d tip = ee_tf.translation() + ee_tf.rotation() * Eigen::Vector3d(0, 0, threshold);
        double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
        geometry_msgs::msg::Point p;
        p.x = tip.x();
        p.y = tip.y();
        p.z = tip.z();
        timed_points.push_back({t, p});
      }
      auto jt_copy = jt;
      std::thread tracer([this, timed_points, is_print, jt_copy]() {
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
        marker_print.ns = "print_trace";  marker_print.id = print_seg_id_;
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
          {
            std::lock_guard<std::mutex> lock(last_js_mutex_);
            last_js_.name = jt_copy.joint_names;
            last_js_.position = std::vector<double>(
              jt_copy.points[idx].positions.begin(),
              jt_copy.points[idx].positions.end());
          }
          trace_points_.push_back(p);
          marker_ee.header.stamp = this->now();
          marker_ee.points = trace_points_;
          trace_pub_->publish(marker_ee);
          if (is_print) {
            print_trace_points_.push_back(p);
            marker_print.header.stamp = this->now();
            marker_print.points = print_trace_points_;
            print_trace_pub_->publish(marker_print);
          }
        }
      });
      move_group_->execute(plan);
      tracer.join();
    } else {
      // Real hardware: /joint_states updates live — poll getCurrentPose at 20 Hz
      auto& pts = plan.trajectory.joint_trajectory.points;
      double dur_sec = pts.empty() ? 5.0 :
          pts.back().time_from_start.sec + pts.back().time_from_start.nanosec * 1e-9;
      auto stop_time = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(dur_sec + 0.2);
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
        marker_print.id = 0;
        marker_print.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker_print.action = visualization_msgs::msg::Marker::ADD;
        marker_print.scale.x = 0.002;
        marker_print.color.r = 0.5f;  
        marker_print.color.g = 0.0f;
        marker_print.color.b = 0.5f;  
        marker_print.color.a = 1.0f;
        marker_print.lifetime = rclcpp::Duration(0, 0);
        while (std::chrono::steady_clock::now() < stop_time) {
          auto pose = move_group_->getCurrentPose(end_effector_link_);
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
          trace_points_.push_back(p);
          marker_ee.header.stamp = this->now();
          marker_ee.points = trace_points_;
          trace_pub_->publish(marker_ee);
          if (is_print) {
            print_trace_points_.push_back(p);
            marker_print.header.stamp = this->now();
            marker_print.points = print_trace_points_;
            print_trace_pub_->publish(marker_print);
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      });
      move_group_->execute(plan);
      tracer.join();
    }

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
    res->moves_sent = static_cast<int32_t>(goal_array.size());
  }
  
  // This function creates a table pose matrix from the table's position and orientation parameters
  // and then uses it to transform G-code coordinates from the table frame to the robot frame. 
  // It also computes the end-effector orientation based on the table's normal vector, ensuring that 
  // the end-effector approaches the table perpendicularly. The function then plans and executes a 
  // trajectory for each G-code move, handling both linear and arc movements, and scales the feed rate as necessary.
  
  void goal_callback(
    const std::shared_ptr<meca500_demo::srv::Goal::Request>  req,
          std::shared_ptr<meca500_demo::srv::Goal::Response> res) {

    goal_array.clear();

    if (use_extruder_ && real_print_) {
      ender_ready_ = open_ender();
    }

    // Parse gcode
    gcode::Program program = gcode::parse(req->gcode);
    RCLCPP_INFO(this->get_logger(), "Parsed %zu G moves", program.size());

    // Assert whichever extrusion mode gcode_parser.py actually declared in
    // the file (OUTPUT_RELATIVE_E there, read back via gcode.cpp's M82/M83
    // handling) instead of hardcoding an assumption independently here --
    // this is the single source of truth for what the per-move G1 E
    // deltas below assume.
    if (ender_ready_) {
      send_ender(program.e_relative_mode ? "M83" : "M82");
    }

    // getting the table's normal vector and center position in the world frame
    Eigen::Quaterniond q_table(qw, qx, qy, qz); // table orientation
    q_table.normalize();
    Eigen::Matrix3d R_table = q_table.toRotationMatrix(); // rotation matrix of the table
    Eigen::Vector3d table_center(x, y, z); // center of the table// vector from ee to table center

    auto ee_pose = move_group_->getCurrentPose(end_effector_link_).pose;
    Eigen::Vector3d ee_to_table_vector(ee_pose.position.x - table_center.x(),
                                       ee_pose.position.y - table_center.y(),
                                       ee_pose.position.z - table_center.z());
    
    // if the table normal is opposite to the vector from ee to table center, flip the normal
    if (R_table.col(2).dot(ee_to_table_vector) < 0) {
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
  
    for (const auto& move : program.moves) {

        double pt_z = move.z / 1000.0; //to give a slight offset from the table surface
        if (pt_z < 0.001) pt_z = 0.001;

        // X, Y, Z
        Eigen::Vector4d P_table(move.x / 1000.0, move.y / 1000.0, pt_z, 1.0);
        Eigen::Vector4d P_robot = T * P_table;    
        
        // I, J
        Eigen::Vector4d IJ_table(move.i / 1000.0, move.j / 1000.0, 0.0, 0.0);
        Eigen::Vector4d IJ_robot = T * IJ_table;   
        
        //F             
        double f_scaled = f_scaling(move.f, move.has_f);

        goal_array.push_back({move.cmd, P_robot.x(), P_robot.y(), P_robot.z(),
                              IJ_robot.x(), IJ_robot.y(), move.e, f_scaled, move.f, move.has_i,
                              move.has_j, move.has_e, move.has_f});
    }

    res->success = true;

    if (!moved_to_bed_) {
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
      bool success = false;

      // for the 1st goal point
      // Sample along perpendicular line
      // draw a line from the table surface (t=0) to a point above the table (t=0.35) passing
      // through the goal point
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

        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnect");
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(target, end_effector_link_);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "SUCCESS at t=%.2f! Executing...", t);
          execute_with_trace(plan, is_print);
          success = true;
          // Wait for state monitor to update
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          break;

        }
        
        move_group_->clearPoseTargets();
      }
      
      moved_to_bed_ = success;

      // Now reaching the first gcode point       
      if (success && !goal_array.empty()) {
        geometry_msgs::msg::Pose target;

        // Commanding the end-effector *frame*, which sits `threshold` short of the
        // true tip along local +Z, so pull the frame back to land the tip on the target.
        Eigen::Vector3d table_tool_offset = ee_orient.toRotationMatrix() * Eigen::Vector3d(0, 0, -threshold);
        target.position.x = goal_array.at(0).x + table_tool_offset.x();
        target.position.y = goal_array.at(0).y + table_tool_offset.y();
        target.position.z = goal_array.at(0).z + table_tool_offset.z();
        target.orientation.x = ee_orient.x();
        target.orientation.y = ee_orient.y();
        target.orientation.z = ee_orient.z();
        target.orientation.w = ee_orient.w();

        move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_->setPlannerId("LIN");
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        move_group_->setPoseTarget(target, end_effector_link_);
        moveit::planning_interface::MoveGroupInterface::Plan lin_to_table_plan;
        if (move_group_->plan(lin_to_table_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(this->get_logger(), "Planning to table surface SUCCESS! Executing...");
          execute_with_trace(lin_to_table_plan, is_print);
        }
        RCLCPP_INFO(this->get_logger(), "Reached table origin!");
      }
    }

    if (moved_to_bed_) {
      // Commanding the end-effector *frame*, which sits `threshold` short of the
      // true tip along local +Z, so pull the frame back to land the tip on the gcode coordinate
      Eigen::Vector3d tool_offset = ee_orient.toRotationMatrix() * Eigen::Vector3d(0, 0, -threshold);
      for (size_t i = 0; i < goal_array.size(); i++) {
        geometry_msgs::msg::Pose target;

        target.position.x = goal_array.at(i).x + tool_offset.x();
        target.position.y = goal_array.at(i).y + tool_offset.y();
        target.position.z = goal_array.at(i).z + tool_offset.z();
        target.orientation.x = ee_orient.x();
        target.orientation.y = ee_orient.y();
        target.orientation.z = ee_orient.z();
        target.orientation.w = ee_orient.w();


        // If G1
        if (goal_array.at(i).cmd == "G1") {
          if (goal_array.at(i).has_e && goal_array.at(i).e > 1e-6) {
            if (!is_print) {
              print_trace_points_.clear();
              print_seg_id_++;
            }
            is_print = true;
          }
         
          RCLCPP_INFO(this->get_logger(), "Planning G1 to point %zu: [%.3f, %.3f, %.3f]",
          i, goal_array.at(i).x, goal_array.at(i).y, goal_array.at(i).z);

          
          move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
          move_group_->setPlannerId("LIN");
          {
            auto cp = move_group_->getCurrentPose(end_effector_link_).pose;
            double dx = target.position.x - cp.position.x;
            double dy = target.position.y - cp.position.y;
            double dz = target.position.z - cp.position.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            double vel = std::min(goal_array.at(i).f, std::max(0.1, 0.5 * (0.01 / std::max(dist, 0.001))));
            move_group_->setMaxVelocityScalingFactor(vel);
          }
          move_group_->setMaxAccelerationScalingFactor(0.05);
          move_group_->setPoseTarget(target, end_effector_link_);

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "LIN to point %zu SUCCESS", i);

            if (use_extruder_ && ender_ready_ && goal_array.at(i).has_e && std::abs(goal_array.at(i).e) > 1e-6) {
              // Match the extrusion feedrate to how long the arm's own plan
              // takes, instead of using the gcode's raw feedrate, so the
              // extrusion and the arm motion finish at the same time rather
              // than one running out well before the other.
              auto& pts = plan.trajectory.joint_trajectory.points;
              double move_duration = pts.empty() ? 1.0 :
                  pts.back().time_from_start.sec + pts.back().time_from_start.nanosec * 1e-9;
              // Guard against a near-zero-duration plan (e.g. a single-
              // waypoint trajectory) driving matched_f to something huge
              // or infinite/NaN.
              if (move_duration < 0.05){
                move_duration = 0.05;
              }

              double e_val = goal_array.at(i).e * extrusion_multiplier_;
              double matched_f = (std::abs(e_val) / move_duration) * 60.0;

              RCLCPP_INFO(this->get_logger(),
                "Extruding at point %zu: e_val=%.4f matched_f=%.2f move_duration=%.3f ender_ready_=%d",
                i, e_val, matched_f, move_duration, ender_ready_);

              std::thread extrude_thread([this, e_val, matched_f]() {
                send_ender("G1 E" + std::to_string(e_val) + " F" + std::to_string(matched_f));
              });

              execute_with_trace(plan, is_print);  // arm on this thread
              extrude_thread.join();               // wait for both
            } else {
              RCLCPP_INFO(this->get_logger(),
                "Point %zu: extrusion skipped (use_extruder_=%d ender_ready_=%d has_e=%d e=%.4f)",
                i, use_extruder_, ender_ready_, goal_array.at(i).has_e, goal_array.at(i).e);
              execute_with_trace(plan, is_print);
            }
            is_print = false;
          } else {
            RCLCPP_ERROR(this->get_logger(), "LIN to point %zu FAILED", i);
            res->success = false;
            continue;;
          }
        }
        // if G2 or G3
        else if (goal_array.at(i).cmd == "G2" || goal_array.at(i).cmd == "G3") {
          if (goal_array.at(i).has_e && goal_array.at(i).e > 1e-6) {
            if (!is_print) {
              print_trace_points_.clear();
              print_seg_id_++;
            }
            is_print = true;
          }

          RCLCPP_INFO(this->get_logger(), "Planning %s to point %zu: [%.3f, %.3f, %.3f] with center offset [%.3f, %.3f]",
            goal_array.at(i).cmd.c_str(), i,
            goal_array.at(i).x, goal_array.at(i).y, goal_array.at(i).z,
            goal_array.at(i).i, goal_array.at(i).j);

          // NEW (change 1 of 4): anchor the arc start to the gcode's intended
          // previous point instead of the live robot pose. The slicer defines
          // I/J relative to that previous point; reading the start from
          // getCurrentPose() instead means any offset between the robot and
          // that point (a skipped move, tracking residue, stale joint state)
          // shifts the computed center and rotates both atan2 angles inside
          // get_arc_center(). For a shallow arc the whole commanded sweep is
          // under a degree, so a sub-mm offset is enough to push start_angle
          // past end_angle, the +2pi branch fires, and the interim lands on
          // the far side of the circle -- Pilz then sweeps the near-full
          // circle through it. Same correct math, wrong input.
          double start_x, start_y, start_z;
          if (i > 0) {
            start_x = goal_array.at(i - 1).x + tool_offset.x();
            start_y = goal_array.at(i - 1).y + tool_offset.y();
            start_z = goal_array.at(i - 1).z + tool_offset.z();
          } else {
            auto current_pose = move_group_->getCurrentPose(end_effector_link_).pose;
            start_x = current_pose.position.x;
            start_y = current_pose.position.y;
            start_z = current_pose.position.z;
          }

          // NEW (change 2 of 4): the interim computed from the anchor is only
          // valid if Pilz will actually start there -- Pilz always plans from
          // the robot's real state, and it never sees G2/G3: the direction is
          // carried purely by which side of the chord the interim sits on.
          // If the robot is not at the anchor (an earlier move failed and got
          // skipped), don't plan an arc at all: demote to G1, since a straight
          // line to an absolute target cannot loop from anywhere.
          if (i > 0) {
            auto live_pose = move_group_->getCurrentPose(end_effector_link_).pose;
            double desync = std::hypot(live_pose.position.x - start_x,
                                        live_pose.position.y - start_y);
            if (desync > 0.0001) {  // 0.1 mm
              RCLCPP_WARN(this->get_logger(),
                "Point %zu: robot is %.3fmm off the arc's start (upstream move skipped), demoting arc to LIN",
                i, desync * 1000.0);
              goal_array.at(i).cmd = "G1";
              goal_array.at(i).has_i = false;
              goal_array.at(i).has_j = false;
              i--;
              continue;
            }
          }

          // target is now declared above the if/else block so it's valid here
          geometry_msgs::msg::PoseStamped center;
          center.header.frame_id = "world";
          center.pose.position.x = start_x + goal_array.at(i).i;
          center.pose.position.y = start_y + goal_array.at(i).j;
          center.pose.position.z = start_z;

          double r_start = std::hypot(start_x - center.pose.position.x,
                                       start_y - center.pose.position.y);
          double r_end   = std::hypot(target.position.x - center.pose.position.x,
                                       target.position.y - center.pose.position.y);

          RCLCPP_INFO(get_logger(), "r_start=%.6f r_end=%.6f diff=%.9f",
            r_start, r_end, fabs(r_start - r_end));

          if (r_start < 1e-9 || std::fabs(r_end - r_start) > std::max(0.05 * r_start, 0.00005)) {
            // target isn't actually on the circle I/J implies (e.g. a stale
            // wipe/retraction arc, or a target shifted by the tool offset in
            // a way the source I/J never accounted for) -- get_arc_center()
            // below only uses r_start, so feeding it a mismatched target
            // produces a bogus interim waypoint and CIRC sweeps out a loop
            // that was never actually in the gcode. Plan as LIN instead.
            RCLCPP_WARN(this->get_logger(),
              "Point %zu: arc endpoint not on I/J circle (r_start=%.6f r_end=%.6f), planning as LIN",
              i, r_start, r_end);
            goal_array.at(i).cmd = "G1";
            goal_array.at(i).has_i = false;
            goal_array.at(i).has_j = false;
            i--;
            continue;
          }

          // NEW (change 3 of 4): sweep sanity check. The parser guarantees
          // every arc it writes sweeps well under 180 degrees, so a computed
          // sweep near a full circle can only mean inconsistent inputs (a
          // direction flip), never a real command -- demote instead of
          // letting CIRC execute it.
          {
            double a0 = atan2(start_y - center.pose.position.y, start_x - center.pose.position.x);
            double a1 = atan2(target.position.y - center.pose.position.y, target.position.x - center.pose.position.x);
            double angle_diff = (goal_array.at(i).cmd == "G2") ? (a0 - a1) : (a1 - a0);
            if (angle_diff <= 0) angle_diff += 2 * M_PI;
            if (angle_diff * 180.0 / M_PI > 185.0) {
              RCLCPP_WARN(this->get_logger(),
                "Point %zu: computed sweep %.1f deg exceeds 185, direction flip -- planning as LIN",
                i, angle_diff * 180.0 / M_PI);
              goal_array.at(i).cmd = "G1";
              goal_array.at(i).has_i = false;
              goal_array.at(i).has_j = false;
              i--;
              continue;
            }
          }

          // get_arc_center
          mid_point_array = get_arc_center(
            {start_x, start_y, start_z},
            {target.position.x, target.position.y, target.position.z},
            {center.pose.position.x, center.pose.position.y, center.pose.position.z},
            goal_array.at(i).cmd);

          // mid_point_array
          Eigen::Vector3d a(start_x, start_y, start_z);
          Eigen::Vector3d b(mid_point_array[0], mid_point_array[1], mid_point_array[2]);
          Eigen::Vector3d c(target.position.x, target.position.y, target.position.z);

          // cross product norm = 2 * triangle area = same check Pilz uses internally
          // threshold 2.5e-11 = 5µm × 5µm = robot resolution limit
          double cross_norm = ((b - a).cross(c - a)).norm();

          if (cross_norm < 2.5e-11) {
            RCLCPP_WARN(this->get_logger(),
              "Point %zu: arc below robot resolution (cross_norm=%.2e), planning as LIN", i, cross_norm);
            // demote to G1, don't erase: the robot still needs to reach this position
            goal_array.at(i).cmd = "G1";
            goal_array.at(i).has_i = false;
            goal_array.at(i).has_j = false;
            // fall back to G1 branch on next iteration of same index
            i--;
            continue;
          }
          geometry_msgs::msg::Pose interim_pose;
          interim_pose.position.x = mid_point_array.at(0);
          interim_pose.position.y = mid_point_array.at(1);
          interim_pose.position.z = mid_point_array.at(2);

          moveit_msgs::msg::Constraints constraints;
          constraints.name = "interim";
          moveit_msgs::msg::PositionConstraint pos_constraint;
          pos_constraint.header.frame_id = "world";
          pos_constraint.link_name = end_effector_link_;
          pos_constraint.constraint_region.primitive_poses.push_back(interim_pose);
          pos_constraint.weight = 1.0;
          constraints.position_constraints.push_back(pos_constraint);
          move_group_->setPathConstraints(constraints);

          move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
          move_group_->setPlannerId("CIRC");
          move_group_->setPlanningTime(10.0);
          {
            auto cp = move_group_->getCurrentPose(end_effector_link_).pose;
            double dx = target.position.x - cp.position.x;
            double dy = target.position.y - cp.position.y;
            double dz = target.position.z - cp.position.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            double vel = std::min(goal_array.at(i).f, std::max(0.1, 0.5 * (0.01 / std::max(dist, 0.001))));
            move_group_->setMaxVelocityScalingFactor(vel);
          }
          move_group_->setMaxAccelerationScalingFactor(0.05);
          move_group_->setPoseTarget(target, end_effector_link_);

          moveit::planning_interface::MoveGroupInterface::Plan plan;
          if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "%s to point %zu SUCCESS", goal_array.at(i).cmd.c_str(), i);

            if (use_extruder_ && ender_ready_ && goal_array.at(i).has_e && goal_array.at(i).e > 1e-6) {
              auto& pts = plan.trajectory.joint_trajectory.points;
              double move_duration = pts.empty() ? 1.0 :
                  pts.back().time_from_start.sec + pts.back().time_from_start.nanosec * 1e-9;
              // Guard against a near-zero-duration plan (e.g. a single-
              // waypoint trajectory) driving matched_f to something huge
              // or infinite/NaN.
              if (move_duration < 0.05) move_duration = 0.05;

              double e_val = goal_array.at(i).e * extrusion_multiplier_;
              double matched_f = (std::abs(e_val) / move_duration) * 60.0;

              RCLCPP_INFO(this->get_logger(),
                "Extruding at point %zu: e_val=%.4f matched_f=%.2f move_duration=%.3f ender_ready_=%d",
                i, e_val, matched_f, move_duration, ender_ready_);

              std::thread extrude_thread([this, e_val, matched_f]() {
                send_ender("G1 E" + std::to_string(e_val) + " F" + std::to_string(matched_f));
              });

              execute_with_trace(plan, is_print);
              extrude_thread.join();
            } else {
              execute_with_trace(plan, is_print);
            }
            is_print = false;
          } else {
            // NEW (change 4 of 4): don't skip the point on CIRC failure. A
            // skipped point leaves the robot short of goal_array.at(i), which
            // is exactly what desyncs the NEXT arc's anchor from reality --
            // the seed of every loop. Demote to G1 and reach the same
            // absolute target as a straight line instead.
            RCLCPP_ERROR(this->get_logger(), "%s to point %zu FAILED, demoting to LIN",
              goal_array.at(i).cmd.c_str(), i);
            res->success = false;
            move_group_->clearPathConstraints();
            goal_array.at(i).cmd = "G1";
            goal_array.at(i).has_i = false;
            goal_array.at(i).has_j = false;
            i--;
            continue;
          }
          move_group_->clearPathConstraints();
        }
      }
    }
    print_finished = true;

    if (print_finished) {
      RCLCPP_INFO(this->get_logger(), "Print finished, going back to home position");
      move_group_->setPlanningPipelineId("ompl");
      move_group_->setPlannerId("RRTConnect");
      move_group_->setPlanningTime(5.0);
      move_group_->setMaxVelocityScalingFactor(0.5);
      move_group_->setMaxAccelerationScalingFactor(0.5);
      move_group_->clearPoseTargets();
      move_group_->setNamedTarget("Home");

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
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