#include<threads.hpp>
#include<udp.hpp>
#include<franka/robot.h>
#include<franka/gripper.h>
#include<robot.hpp>
#include<parameters.hpp>

#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>

#include<chrono>
#include<thread>
#include<array>
#include<mutex>

/****/
#include <arpa/inet.h>
#include <unistd.h>      // for close()
/****/

#include<json.hpp>

using namespace std;
using namespace nlohmann;


void CSIR::thread_robot_control(const char* robot_ip_, MessageQue<array<double, DOF> >& message_queue){
    franka::Robot robot(robot_ip_);

/*
    // UDP sender
    int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        perror("Failed to create UDP socket");
        return;
    }
    struct sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(2233);  // target udp port
    inet_pton(AF_INET, "192.168.1.50", &dest_addr.sin_addr);  // target IP
*/


    CSIR::Robot::initialize(robot);
    CSIR::Robot::robot_control(robot, message_queue);

    close(udp_sock);
}

int CSIR::thread_gripper_control(condition_variable& condition, bool& if_grasp){

    mutex m;
    unique_lock<mutex> lk(m);

    try{
       //initialize
       double grasping_width = 0.02;
       franka::Gripper gripper(robot_ip);

       while(true) {
            gripper.homing();
            condition.wait(lk, [&]{return if_grasp;});

           // Check for the maximum grasping width.
           franka::GripperState gripper_state = gripper.readOnce();
           cout<<gripper_state.max_width<<endl;
           if (gripper_state.max_width < grasping_width) {
               std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
               continue;
           }
           // Grasp the object.
           if (!gripper.grasp(grasping_width, 0.1, 100, 0.05, 0.06)) {
               std::cout << "Failed to grasp object." << std::endl;
               continue;
           }

           // wait until if the object is still grasped
           while(true){
               gripper_state = gripper.readOnce();
               if(!gripper_state.is_grasped){
                   cout<<"Object lost"<<endl;
                   break;
               }

               if(!if_grasp){
                   break;
               }

               this_thread::sleep_for(chrono::milliseconds(300));
           }

           gripper.stop();

       }

   }catch (franka::Exception const& e) {
       std::cout << e.what() << std::endl;
       return -1;
   }
}




[[noreturn]] void CSIR::thread_upd_recieve(MessageQue<array<double, DOF> >& message_queue, condition_variable& condition, bool& if_grasp){
    int len;

    int sock_fd = CSIR::UDP::create_and_bind(udp_port, len);

    ssize_t recv_num;
    char recv_buf[200];
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
            std::string data = recv_buf;

            auto j = json::parse(data);
            auto joints = j["joints"];
            bool _if_grasp = j["gripper"];

            double joint0 = joints[0];
            double joint1 = joints[1];
            double joint2 = joints[2];
            double joint3 = joints[3];
            double joint4 = joints[4];
            double joint5 = joints[5];
            double joint6 = joints[6];
            JointVal[0] = joint0;
            JointVal[1] = joint1;
            JointVal[2] = joint2;
            JointVal[3] = joint3;
            JointVal[4] = joint4;
            JointVal[5] = joint5;
            JointVal[6] = joint6;

            //send information
            //std::cout << JointVal[0]<<' ' <<JointVal[1]<<' '<<JointVal[2]<<' '<<JointVal[3]<<' '<<JointVal[4]<<' '<<JointVal[5]<<' '<<JointVal[6]<< std::endl;
            message_queue.put(JointVal);

            //send grasp
            if_grasp = bool(_if_grasp);
            condition.notify_all();
        }

        this_thread::sleep_for(1ms);
    }
}