#include<threads.hpp>
#include<udp.hpp>
#include<franka/robot.h>
#include<robot.hpp>
#include<parameters.hpp>

#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>

#include<chrono>
#include<thread>
#include<array>

using namespace std;

void CSIR::thread_robot_control(const char* robot_ip_, MessageQue<array<double, DOF> >& message_queue){
    franka::Robot robot(robot_ip_);
    CSIR::Robot::initialize(robot);
    CSIR::Robot::robot_control(robot, message_queue);
}


[[noreturn]] void CSIR::thread_upd_recieve(MessageQue<array<double, DOF> >& message_queue){
    int len;

    int sock_fd = CSIR::UDP::create_and_bind(udp_port, len);

    ssize_t recv_num;
    char recv_buf[100];
    struct sockaddr_in addr_client;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    std::array<double, DOF> JointVal = {0};

    while (true)
    {
        //start server
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        if(recv_num > 0){
            recv_buf[recv_num] = '\0';
            std::string joints = recv_buf;
            std::string str0 = joints.substr(1,12);
            std::string str1 = joints.substr(13,12);
            std::string str2 = joints.substr(25,12);
            std::string str3 = joints.substr(37,12);
            std::string str4 = joints.substr(49,12);
            std::string str5 = joints.substr(61,12);
            std::string str6 = joints.substr(73,6);
            double joint0 = std::stod(str0);
            double joint1 = std::stod(str1);
            double joint2 = std::stod(str2);
            double joint3 = std::stod(str3);
            double joint4 = std::stod(str4);
            double joint5 = std::stod(str5);
            double joint6 = std::stod(str6);
            JointVal[0] = joint0;
            JointVal[1] = joint1;
            JointVal[2] = joint2;
            JointVal[3] = joint3;
            JointVal[4] = joint4;
            JointVal[5] = joint5;
            JointVal[6] = joint6;

            //std::cout << JointVal[0]<<' ' <<JointVal[1]<<' '<<JointVal[2]<<' '<<JointVal[3]<<' '<<JointVal[4]<<' '<<JointVal[5]<<' '<<JointVal[6]<< std::endl;
            message_queue.put(JointVal);
        }

        this_thread::sleep_for(1ms);
    }
}