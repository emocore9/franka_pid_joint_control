#include<robot.hpp>
#include<pid.hpp>
#include<parameters.hpp>

#include<franka/exception.h>
#include<franka/robot.h>

#include<queue>
#include<cmath>

using namespace std;


void CSIR::Robot::initialize(franka::Robot& robot){

    //set default behaviour
    robot.setCollisionBehavior(
    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
    {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
    {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

void CSIR::Robot::robot_control(franka::Robot& robot, 
CSIR::Robot::messageQ& recieved_angle_queue){
    
    //desired angle queue for all 7 joint
    queue<double> desire_angle_queues[DOF];

    //pid optimizers
    CSIR::PID pid_optimizers[DOF];
    for(int i = 0; i < DOF; i++){
        pid_optimizers[i] = CSIR::PID(CSIR::time_control_interval, CSIR::pid_max, CSIR::pid_min, CSIR::Kp, CSIR::Kd, CSIR::Ki);
    }

    int time_update_previous = 0;
    arrayDOF previous_setting_angles = INITIAL_ANGLE_STATE;

    robot.control([&](const franka::RobotState&  robot_state, franka::Duration period) -> franka::JointVelocities {
        
        int time_current = period.toMSec(); 

        //update the queues which stored the next   
        if(time_update_previous - time_current >= CSIR::time_update_direvalue){
            update_desire_velocity(desire_angle_queues, previous_setting_angles, recieved_angle_queue);
            time_update_previous = time_current;
        }

        //generate optimal velocities
        franka::JointVelocities ret = operate_pid(robot_state,
        desire_angle_queues,
        pid_optimizers,
        previous_setting_angles);
        

        return ret;
    });


}

void CSIR::Robot::update_desire_velocity(queue<double> desire_angle_queues[], 
const arrayDOF& previous_setting_angles, 
CSIR::Robot::messageQ& recieved_angle_queue){

    //get recieved value from message queue from UDP protocol
    arrayDOF recieved;
    if(!recieved_angle_queue.get(recieved)){
        return;
    }

    //add segmeented postions to the queue
    for(int i = 0; i < DOF; i++){
        //clean the operating queue for coresponding joint
        desire_angle_queues[i] = queue<double>();
        
        //devided the desired rout into group of positions
        double control_update = ceil(control_update_max*fabs((recieved[i] - previous_setting_angles[i])/(max_q[i] - min_q[i])));
        if (control_update < control_update_min)
            control_update =control_update_min;

        for(int j = 0; j < control_update; j++) 
            desire_angle_queues[i].push(previous_setting_angles[i] + (recieved[i] - previous_setting_angles[i]) * (j+1)/control_update);
    }

}

franka::JointVelocities CSIR::Robot::operate_pid(const franka::RobotState& robot_state,
std::queue<double> desire_angle_queues[], 
CSIR::PID pid_optimizers[], 
CSIR::Robot::arrayDOF& previous_setting_angles){
    
    static arrayDOF previous_execution_velocities = {0.0};
    static arrayDOF previous_ddq = {0.0};
    static arrayDOF previous_dddq = {0.0};

    arrayDOF current_realtime_angles = robot_state.q;

    arrayDOF velocities_excution = {0.0};
    arrayDOF V_optimal_realtime = {0.0};
    arrayDOF A_optimal_realtime = {0.0};
    arrayDOF N_optimal_realtime = {0.0};
    arrayDOF N_d_optimal_realtime = {0.0};

    for(int i = 0; i < DOF; i++){

        if(desire_angle_queues[i].empty()){
             continue;
        }

        double desire_angle = desire_angle_queues[i].front();
        desire_angle_queues[i].pop();
        previous_setting_angles[i] = desire_angle;

        V_optimal_realtime[i] = pid_optimizers[i].calculate(desire_angle, current_realtime_angles[i]);
        
        //add constraints
        /* @todo add constraints*/

        if (fabs(V_optimal_realtime[i])>max_q_d[i])
            V_optimal_realtime[i]=max_q_d[i]*(2*(V_optimal_realtime[i]>0)-1);

        A_optimal_realtime[i]= ( V_optimal_realtime[i]-previous_execution_velocities[i])/time_control_interval;
        if (fabs(A_optimal_realtime[i])>max_q_dd[i])
            A_optimal_realtime[i]=max_q_dd[i]*(2*(A_optimal_realtime[i]>0)-1);

        N_optimal_realtime[i]=( A_optimal_realtime[i]-previous_ddq[i])/time_control_interval;
        if (fabs(N_optimal_realtime[i])>max_q_ddd[i])
            N_optimal_realtime[i]=max_q_ddd[i]*(2*(N_optimal_realtime[i]>0)-1);

        N_d_optimal_realtime[i]=( N_optimal_realtime[i]-previous_dddq[i])/time_control_interval;
        if (fabs(N_d_optimal_realtime[i])>max_q_dddd[i])
            N_d_optimal_realtime[i]=max_q_dddd[i]*(2*(N_d_optimal_realtime[i]>0)-1);

        
        double current_dddq = previous_dddq[i] + N_d_optimal_realtime[i] * time_control_interval;
        double current_ddq = previous_ddq[i] + current_dddq * time_control_interval;
        velocities_excution[i] = previous_execution_velocities[i] + current_ddq * time_control_interval;

        //record the data that last excuted
        previous_dddq[i] = current_dddq;
        previous_ddq[i] = current_ddq;
        previous_execution_velocities[i] = velocities_excution[i];
    }

    return franka::JointVelocities(velocities_excution);

}