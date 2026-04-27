#include <iostream>
#include <chrono>
#include <thread>

#include "arm_moveit_demo/moveit_demo_utils.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std;

int main(int argc, char **argv) {
    arm_moveit_demo::ScopedMoveItNode demo(argc, argv, "cartesian_plan_cpp");
    const auto logger = demo.node()->get_logger();
    moveit::planning_interface::MoveGroupInterface yahboomcar(demo.node(), "arm_group");
    string frame = yahboomcar.getPlanningFrame();
    moveit_visual_tools::MoveItVisualTools tool(demo.node(), frame);
    tool.deleteAllMarkers();
    yahboomcar.allowReplanning(true);
    // 规划的时间(单位：秒)
    yahboomcar.setPlanningTime(50);
    yahboomcar.setNumPlanningAttempts(10);
    // 设置允许目标角度误差
    yahboomcar.setGoalJointTolerance(0.01);
    yahboomcar.setGoalPositionTolerance(0.01);
    yahboomcar.setGoalOrientationTolerance(0.01);
    yahboomcar.setGoalTolerance(0.01);
    // 设置允许的最大速度和加速度
    yahboomcar.setMaxVelocityScalingFactor(1.0);
    yahboomcar.setMaxAccelerationScalingFactor(1.0);
    RCLCPP_INFO(logger, "Set Init Pose.");
    //设置具体位置
    vector<double> pose{0, -0.69, -0.17, 0.86, 0};
    yahboomcar.setJointValueTarget(pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    yahboomcar.plan(plan);
    yahboomcar.execute(plan);
    // 获取当前位姿数据最为机械臂运动的起始位姿
    geometry_msgs::msg::Pose start_pose = yahboomcar.getCurrentPose(yahboomcar.getEndEffectorLink()).pose;
    //初始化路径点向量
    std::vector<geometry_msgs::msg::Pose> waypoints;
    //将初始位姿加入路点列表
    waypoints.push_back(start_pose);
    start_pose.position.x -= 0.04;
    waypoints.push_back(start_pose);
    start_pose.position.z -= 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.x += 0.04;
    waypoints.push_back(start_pose);
    start_pose.position.z -= 0.02;
    waypoints.push_back(start_pose);
    start_pose.position.x += 0.03;
    waypoints.push_back(start_pose);
    // 笛卡尔空间下的路径规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.1;
    double fraction = 0.0;
    int maxtries = 1000;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    while (fraction < 1.0 && attempts < maxtries) {
        fraction = yahboomcar.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        if (attempts % 10 == 0) RCLCPP_INFO(logger, "Still trying after %d attempts...", attempts);
    }
    if (fraction == 1) {
        RCLCPP_INFO(logger, "Path computed successfully. Moving the arm.");
        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        // 显示轨迹
        tool.publishTrajectoryLine(plan.trajectory_, yahboomcar.getCurrentState()->getJointModelGroup("arm_group"));
        tool.trigger();
        // 执行运动
        yahboomcar.execute(plan);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
        RCLCPP_INFO(logger, "Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }
    return 0;
}
