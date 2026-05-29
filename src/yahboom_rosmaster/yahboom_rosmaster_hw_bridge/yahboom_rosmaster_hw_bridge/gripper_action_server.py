"""FollowJointTrajectory action server for the X3 Plus 1-DOF gripper.

Phase 0 stub mirroring arm_action_server. Phase 2 wires the close loop
to Rosmaster_Lib.set_uart_servo_angle() and re-tunes the contact
thresholds against the real servo (Phase 4).
"""
from __future__ import annotations

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node

from ._rosmaster_lib import acquire_driver


class GripperActionServer(Node):
    def __init__(self) -> None:
        super().__init__("gripper_action_server")
        self.declare_parameter(
            "action_name", "/gripper_controller/follow_joint_trajectory"
        )
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("serial_baud", 115200)

        action_name = self.get_parameter("action_name").value
        self._driver = acquire_driver(
            self.get_parameter("serial_port").value,
            int(self.get_parameter("serial_baud").value),
        )
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
        )
        if self._driver is None:
            self.get_logger().warn(
                f"[gripper] up on '{action_name}' in stub mode (no Rosmaster_Lib)"
            )
        else:
            self.get_logger().info(f"[gripper] up on '{action_name}' with live driver")

    def _goal_callback(self, _goal_request) -> GoalResponse:
        if self._driver is None:
            self.get_logger().warn("[gripper] rejecting goal: stub mode")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        self.get_logger().error("[gripper] execute not implemented (Phase 2)")
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        return result


def main() -> None:
    rclpy.init()
    node = GripperActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
