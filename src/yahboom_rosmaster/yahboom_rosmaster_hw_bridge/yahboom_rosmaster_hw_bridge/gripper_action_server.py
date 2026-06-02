"""FollowJointTrajectory action server for the X3 Plus 1-DOF gripper.

Phase 1 stub mirroring arm_action_server. Does not open the serial port
(base_driver owns it). Phase 2 wires the close loop to
Rosmaster_Lib.set_uart_servo_angle() inside the consolidated
yahboom_bridge_node, and Phase 4 re-tunes the contact thresholds.
"""
from __future__ import annotations

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node


class GripperActionServer(Node):
    def __init__(self) -> None:
        super().__init__("gripper_action_server")
        self.declare_parameter(
            "action_name", "/gripper_controller/follow_joint_trajectory"
        )
        # serial_* params accepted for launch parity; unused in Phase 1.
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("serial_baud", 115200)

        action_name = self.get_parameter("action_name").value
        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
        )
        self.get_logger().warn(
            f"[gripper] up on '{action_name}' (stub — Phase 1, all goals rejected)"
        )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        self.get_logger().warn("[gripper] rejecting goal: stub mode (Phase 1)")
        return GoalResponse.REJECT

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
