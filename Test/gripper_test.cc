//
// Created by Jingyan on 2022/2/28.
//

#include <franka/exception.h>
#include <franka/gripper.h>

#include <parameters.hpp>
#include <iostream>

using namespace std;
using namespace CSIR;

int main(int argc, char** argv){
    try{

        double grasping_width = 0.01;
        franka::Gripper gripper(robot_ip);

        gripper.homing();

        // Check for the maximum grasping width.
        franka::GripperState gripper_state = gripper.readOnce();
        if (gripper_state.max_width < grasping_width) {
            std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
            return -1;
        }
        // Grasp the object.
        if (!gripper.grasp(grasping_width, 0.1, 60)) {
            std::cout << "Failed to grasp object." << std::endl;
            return -1;
        }
    }catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
