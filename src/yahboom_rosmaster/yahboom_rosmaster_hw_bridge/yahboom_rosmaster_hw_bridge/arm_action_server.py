"""FollowJointTrajectory action server for the X3 Plus 5-DOF arm.

Phase 1 stub: declares the action server, rejects every goal, will
be replaced in Phase 2 by the consolidated yahboom_bridge_node. Does
NOT open the serial port — base_driver owns /dev/myserial in Phase 1.

The intent here is just to make sure the action namespace exists so
MoveIt can resolve and bind to it, and so the launch graph is the
same shape it will be in Phase 2.
"""
from __future__ import annotations

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node


class ArmActionServer(Node):
    def __init__(self) -> None:
        super().__init__("arm_action_server")
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
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
            f"[arm] up on '{action_name}' (stub — Phase 1, all goals rejected)"
        )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        self.get_logger().warn("[arm] rejecting goal: stub mode (Phase 1)")
        return GoalResponse.REJECT

    async def _execute(self, goal_handle):
        # Unreachable while _goal_callback rejects — kept for Phase 2.
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
