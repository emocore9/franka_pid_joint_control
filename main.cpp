#include<iostream>
#include<threads.hpp>
#include<thread>
#include<array>
#include<parameters.hpp>
#include<franka/robot.h>

/****/
#include <mutex>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>

// global joint angle state
std::array<double, 7> g_latest_q{};
std::mutex             g_q_mutex;
std::atomic<bool>      g_running{true};
/****/


using namespace std;
using namespace CSIR;

int main(int argc, char** argv){
    MessageQue<array<double, DOF> > q;
    condition_variable cv;
    bool if_grasp = false;

    thread t1(thread_robot_control, robot_ip, ref(q));
    thread t2(thread_upd_recieve, ref(q), ref(cv), ref(if_grasp));
    thread t3(thread_gripper_control, ref(cv), ref(if_grasp));

    /****/
    std::thread sender([&]() {
    //  UDP socket
        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(2233); // target port
        inet_pton(AF_INET, "192.168.1.50", &addr.sin_addr);  //target IP

        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 Hz

            std::array<double,7> q_copy;
            {
                std::lock_guard<std::mutex> lk(g_q_mutex);
                q_copy = g_latest_q;
            }
            // JSON seq
            nlohmann::json j;
            j["joints"] = std::vector<double>(q_copy.begin(), q_copy.end()); // real gripper state
            j["gripper"] = 0;  // fake gripper state
            auto msg = j.dump();
            sendto(sock,
                    msg.c_str(),
                    static_cast<int>(msg.size()),
                    0,
                    (struct sockaddr*)&addr,
                    sizeof(addr));
                }
            close(sock);
        });


    /****/

    t1.join();
    t2.join();
    t3.join();
    
    /****/
    g_running = false;
    sender.join();
    /****/

    return 1;
}
