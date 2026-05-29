"""FollowJointTrajectory action server for the X3 Plus 5-DOF arm.

Phase 0 stub: declares the action server, accepts goals, rejects them
with INVALID_GOAL until Phase 2 wires the trajectory interpolation +
Rosmaster_Lib.set_uart_servo_angle() loop. The intent here is just to
prove the entry point, action namespace, and parameter loading work.
"""
from __future__ import annotations

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node

from ._rosmaster_lib import acquire_driver


class ArmActionServer(Node):
    def __init__(self) -> None:
        super().__init__("arm_action_server")
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
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
                f"[arm] up on '{action_name}' in stub mode (no Rosmaster_Lib)"
            )
        else:
            self.get_logger().info(f"[arm] up on '{action_name}' with live driver")

    def _goal_callback(self, _goal_request) -> GoalResponse:
        if self._driver is None:
            self.get_logger().warn("[arm] rejecting goal: stub mode")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def _execute(self, goal_handle):
        # Phase 2: interpolate goal.trajectory points → set_uart_servo_angle.
        self.get_logger().error("[arm] execute not implemented (Phase 2)")
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        return result


def main() -> None:
    rclpy.init()
    node = ArmActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
