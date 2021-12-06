#include<iostream>
#include<threads.hpp>
#include<thread>
#include<array>
#include<parameters.hpp>
#include<franka/robot.h>

using namespace std;
using namespace CSIR;

int main(int argc, char** argv){
    MessageQue<array<double, DOF> > q;

    thread t1(thread_robot_control, robot_ip, ref(q));
    thread t2(thread_upd_recieve, ref(q));

    t1.join();
    t2.join();

    return 1;
}
