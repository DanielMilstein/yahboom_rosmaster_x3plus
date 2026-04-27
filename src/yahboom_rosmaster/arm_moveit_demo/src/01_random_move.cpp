#include <iostream>
#include <chrono>
#include <thread>

#include "arm_moveit_demo/moveit_demo_utils.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv) {
    arm_moveit_demo::ScopedMoveItNode demo(argc, argv, "yahboomcar_random_move_cpp");
    moveit::planning_interface::MoveGroupInterface yahboomcar(demo.node(), "arm_group");
	// 设置最大速度
    yahboomcar.setMaxVelocityScalingFactor(1.0);
    // 设置最大加速度
    yahboomcar.setMaxAccelerationScalingFactor(1.0);
    //设置目标点
    yahboomcar.setNamedTarget("down");
    //开始移动
    yahboomcar.move();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (rclcpp::ok()){
    	//设置随机目标点
    	yahboomcar.setRandomTarget();
    	yahboomcar.move();
    	std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}
