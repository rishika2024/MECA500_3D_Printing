#!/usr/bin/env python3
"""ROS2 node that loads objects.yaml into the MoveIt planning scene."""
import rclpy
from rclpy.node import Node
from meca500_demos.planningscene import PlanningSceneClass


class SceneNode(Node):

    def __init__(self):
        super().__init__('scene_node')
        self.scene = PlanningSceneClass(self)
        self.published = False
        # Republish every 5s so move_group receives it regardless of startup order
        self.timer = self.create_timer(5.0, self.publish_scene)

    def publish_scene(self):
        self.scene.load_scene('objects.yaml')
        if not self.published:
            self.get_logger().info('Planning scene published.')
            self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = SceneNode()
    rclpy.spin(node)
    rclpy.shutdown()
