#!/usr/bin/env python3

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA

import rclpy
from rclpy.node import Node


FRAME_ID = "base_footprint"


def make_pose(x, y, z):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    return pose


def make_box_object(object_id, pose, size):
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]

    collision_object = CollisionObject()
    collision_object.header.frame_id = FRAME_ID
    collision_object.id = object_id
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD
    return collision_object


def make_cylinder_object(object_id, pose, height, radius):
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.CYLINDER
    primitive.dimensions = [float(height), float(radius)]

    collision_object = CollisionObject()
    collision_object.header.frame_id = FRAME_ID
    collision_object.id = object_id
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(pose)
    collision_object.operation = CollisionObject.ADD
    return collision_object


def make_color(object_id, rgba):
    color = ObjectColor()
    color.id = object_id
    color.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
    return color


class TabletopScenePublisher(Node):
    def __init__(self):
        super().__init__("tabletop_planning_scene_publisher")
        self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.publish_count = 0
        self.timer = self.create_timer(1.0, self.publish_scene)

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        scene.world.collision_objects.extend(
            [
                make_box_object("task_table", make_pose(0.32, 0.04, 0.12), (0.36, 0.34, 0.03)),
                make_cylinder_object("task_can", make_pose(0.24, 0.10, 0.192), 0.11, 0.025),
                make_box_object("task_bin_base", make_pose(0.39, -0.07, 0.145), (0.12, 0.12, 0.01)),
                make_box_object("task_bin_front_wall", make_pose(0.334, -0.07, 0.18), (0.008, 0.12, 0.07)),
                make_box_object("task_bin_back_wall", make_pose(0.446, -0.07, 0.18), (0.008, 0.12, 0.07)),
                make_box_object("task_bin_left_wall", make_pose(0.39, -0.014, 0.18), (0.12, 0.008, 0.07)),
                make_box_object("task_bin_right_wall", make_pose(0.39, -0.126, 0.18), (0.12, 0.008, 0.07)),
            ]
        )

        scene.object_colors.extend(
            [
                make_color("task_table", (0.55, 0.34, 0.18, 1.0)),
                make_color("task_can", (0.95, 0.10, 0.06, 1.0)),
                make_color("task_bin_base", (0.08, 0.34, 0.90, 1.0)),
                make_color("task_bin_front_wall", (0.08, 0.34, 0.90, 1.0)),
                make_color("task_bin_back_wall", (0.08, 0.34, 0.90, 1.0)),
                make_color("task_bin_left_wall", (0.08, 0.34, 0.90, 1.0)),
                make_color("task_bin_right_wall", (0.08, 0.34, 0.90, 1.0)),
            ]
        )

        self.publisher.publish(scene)
        self.publish_count += 1
        if self.publish_count == 1:
            self.get_logger().info("Published tabletop planning scene objects")
        if self.publish_count >= 5:
            self.timer.cancel()


def main():
    rclpy.init()
    node = TabletopScenePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
