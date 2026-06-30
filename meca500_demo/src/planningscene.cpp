#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "meca500_demo/srv/table.hpp"

using namespace std::chrono_literals;

class Meca500PlanningScene : public rclcpp::Node
{
public:
  Meca500PlanningScene()
  : Node("meca500_planning_scene")
  {
    this->declare_parameter("x", 0.0);  
    this->declare_parameter("y", 0.2);
    this->declare_parameter("z", 0.00);
    this->declare_parameter("qx", 0.0);
    this->declare_parameter("qy", 0.0);
    this->declare_parameter("qz", 0.0);
    this->declare_parameter("qw", 1.0);

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
      table.pose.orientation.x = this->get_parameter("qx").as_double();
      table.pose.orientation.y = this->get_parameter("qy").as_double();
      table.pose.orientation.z = this->get_parameter("qz").as_double();
      table.pose.orientation.w = this->get_parameter("qw").as_double();
      table.scale.x = 0.33;
      table.scale.y = 0.33;
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
