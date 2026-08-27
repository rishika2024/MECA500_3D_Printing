#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "meca500_demo/srv/table.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

class Meca500PlanningScene : public rclcpp::Node
{
public:
  Meca500PlanningScene()
  : Node("meca500_planning_scene")
  {
    // Bed pose in world (= link_0__base), from a nozzle-touch probe: 6 points
    // touched around the bed, forward-kinematics'd through meca500.urdf to the
    // nozzle tip. Plane fit was 0.87mm RMS. Bed is tilted ~15.6deg about X
    // (drops toward +Y / the robot). Centre pushed out to y=-0.205 (where probe
    // touches 5 and 6 landed -- known-reachable, not the far edge). z is the
    // fitted-plane value minus 12mm: RViz showed the marker ~1-1.5cm high, i.e.
    // the real nozzle-frame-to-tip offset is ~20mm, not the 8mm the FK used.
    // Overridable at runtime via table_service.
    this->declare_parameter("x", -0.0197);
    this->declare_parameter("y", -0.2050);
    this->declare_parameter("z", 0.0395);
    this->declare_parameter("qx", -0.1359);
    this->declare_parameter("qy", -0.0046);
    this->declare_parameter("qz", 0.0006);
    this->declare_parameter("qw", 0.9907);

    // Give the planning scene interface time to connect to move_group
    rclcpp::sleep_for(500ms);

    // moveit::planning_interface::PlanningSceneInterface psi;
    // psi.applyCollisionObjects({ make_build_plate(), make_table() });

    RCLCPP_INFO(this->get_logger(), "Planning scene objects added.");

    // Publishers
    table_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("table_marker", 10);   

    // Service server
    table_server_ = this->create_service<meca500_demo::srv::Table>(
      "table_service",
      std::bind(&Meca500PlanningScene::table_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Timer
    timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { timer_callback(); });

  }


private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr table_marker_pub;
  rclcpp::Service<meca500_demo::srv::Table>::SharedPtr table_server_;
  rclcpp::TimerBase::SharedPtr timer;

  void table_callback(
    const std::shared_ptr<meca500_demo::srv::Table::Request>  req,
          std::shared_ptr<meca500_demo::srv::Table::Response> res)
  {
    this->set_parameter(rclcpp::Parameter("x",  req->x));
    this->set_parameter(rclcpp::Parameter("y",  req->y));
    this->set_parameter(rclcpp::Parameter("z",  req->z));
    this->set_parameter(rclcpp::Parameter("qx", req->qx));
    this->set_parameter(rclcpp::Parameter("qy", req->qy));
    this->set_parameter(rclcpp::Parameter("qz", req->qz));
    this->set_parameter(rclcpp::Parameter("qw", req->qw));
    res->success = true;
  }

  void timer_callback() {
    Eigen::Quaterniond q(
    this->get_parameter("qw").as_double(),
    this->get_parameter("qx").as_double(),
    this->get_parameter("qy").as_double(),
    this->get_parameter("qz").as_double());
    q.normalize();
     visualization_msgs::msg::Marker table;
      table.header.frame_id = "world";
      table.header.stamp = this->get_clock()->now();
      table.ns = "red";
      table.id = 0;
      table.type = visualization_msgs::msg::Marker::CUBE;
      table.action = visualization_msgs::msg::Marker::ADD;
      table.pose.position.x = this->get_parameter("x").as_double();
      table.pose.position.y = this->get_parameter("y").as_double();
      table.pose.position.z = this->get_parameter("z").as_double();
      table.pose.orientation.x = q.x();
      table.pose.orientation.y = q.y();
      table.pose.orientation.z = q.z();
      table.pose.orientation.w = q.w();
      table.scale.x = 0.200;
      table.scale.y = 0.220;
      table.scale.z = 0.001;
      table.color.r = 0.9f;
      table.color.g = 0.9f;
      table.color.b = 0.8f;
      table.color.a = 1.0f;  
      
      table_marker_pub->publish(table);
  }

  
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Meca500PlanningScene>());
  rclcpp::shutdown();
  return 0;
}
