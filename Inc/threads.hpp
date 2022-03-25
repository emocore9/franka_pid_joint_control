#ifndef _THREADS_HPP_
#define _THREADS_HPP_

#include<typeinfo>
#include<queue>
#include<array>
#include<mutex>

#include<parameters.hpp>
#include<condition_variable>
namespace CSIR{

/**
 * @brief simple implementation of a thread-safe message que
 * 
 */

template <class T>
class MessageQue{
    private:
    std::queue<T> _queue;
    std::mutex _q_mutex;

    public:
    MessageQue():
    _queue(),
    _q_mutex()
    {}

    /**
     * @brief put a content in que
     * 
     * @param array 
     */
    void put(T &array){
        std::lock_guard<std::mutex> guard(_q_mutex);
        _queue.push(array);
    }

    /**
     * @brief Get the Array objects 
     * 
     * @param ret the return content
     * @return true get the content successfully
     * @return false if the que is empty
     */
    bool get(T &ret){
        std::lock_guard<std::mutex> guard(_q_mutex);
        if(_queue.empty()) return false;
        ret = _queue.front();
        _queue.pop();
        return true;
    }

    int size(){
        return _queue.size();
    }
};

void thread_robot_control(const char* arg_robot, MessageQue<std::array<double, DOF> >& message_queue);
[[noreturn]] void thread_upd_recieve(MessageQue<std::array<double, DOF> >& message_queue, std::condition_variable& condition, bool& if_grasp);
int thread_gripper_control(std::condition_variable& condition, bool& if_grasp);

};

#endif