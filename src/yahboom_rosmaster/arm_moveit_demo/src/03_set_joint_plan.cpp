#include <iostream>
#include "arm_moveit_demo/moveit_demo_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

int main(int argc, char **argv) {
    arm_moveit_demo::ScopedMoveItNode demo(argc, argv, "set_joint_plan_cpp");
    const auto logger = demo.node()->get_logger();
    moveit::planning_interface::MoveGroupInterface yahboomcar(demo.node(), "arm_group");
    yahboomcar.allowReplanning(true);
    // 规划的时间(单位：秒)
    yahboomcar.setPlanningTime(5);
    yahboomcar.setNumPlanningAttempts(10);
    // 设置允许目标角度误差
    yahboomcar.setGoalJointTolerance(0.01);
    // 设置允许的最大速度和加速度
    yahboomcar.setMaxVelocityScalingFactor(1.0);
    yahboomcar.setMaxAccelerationScalingFactor(1.0);
    yahboomcar.setNamedTarget("up");
    yahboomcar.move();
    //设置具体位置
    vector<double> pose{0, 0.79, -1.57, -1.57, 0};
    yahboomcar.setJointValueTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const moveit::core::MoveItErrorCode &code = yahboomcar.plan(plan);
    if (arm_moveit_demo::succeeded(code)) {
        RCLCPP_INFO(logger, "plan success");
        // 显示轨迹
        string frame = yahboomcar.getPlanningFrame();
        moveit_visual_tools::MoveItVisualTools tool(demo.node(), frame);
        tool.deleteAllMarkers();
        tool.publishTrajectoryLine(plan.trajectory_, yahboomcar.getCurrentState()->getJointModelGroup("arm_group"));
        tool.trigger();
        yahboomcar.execute(plan);
    } else {
        RCLCPP_INFO(logger, "plan error");
    }
    return 0;
}
