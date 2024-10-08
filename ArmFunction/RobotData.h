#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#define DOF_ARM 7
#define DOF_LEG 6
#define DOF_WAIST 3
#define DOF_HEAD 2
#define DOF_HAND 6

#include "chrono"
#include "thread"

class RobotData
{
public:
    struct Robot_Feedback_Basic_Info
    {      
        float joint_q_head[DOF_HEAD];//rad
        float arm_cartesion[2][DOF_ARM];
        float joint_qd_head[DOF_HEAD];//rad/s
        float joint_tau_head[DOF_HEAD];//Nm
        float joint_power_head[DOF_HEAD];//W
        float tau_exp_head[DOF_HEAD];//Nm
        float current_exp_head[DOF_HEAD];//A
        char joint_state_head[DOF_HEAD];
        //uint16_t joint_state_head[DOF_HEAD];
        //------------waist
        float joint_q_waist[DOF_WAIST];
        float joint_qd_waist[DOF_WAIST];
        float joint_tau_waist[DOF_WAIST];
        float joint_power_waist[DOF_WAIST];
        float tau_exp_waist[DOF_WAIST];
        float current_exp_waist[DOF_WAIST];
        char joint_state_waist[DOF_WAIST];
        //uint16_t joint_state_waist[DOF_WAIST];
        //------------arm
        float joint_q_arm[2][DOF_ARM];
        float joint_qd_arm[2][DOF_ARM];
        float joint_tau_arm[2][DOF_ARM];
        float joint_power_arm[2][DOF_ARM];
        float tau_exp_arm[2][DOF_ARM];
        float current_exp_arm[2][DOF_ARM];
        char joint_state_arm[2][DOF_ARM];
        //uint16_t joint_state_arm[2][DOF_ARM];
        //------------leg
        float joint_q_leg[2][DOF_LEG];
        float joint_qd_leg[2][DOF_LEG];
        float joint_tau_leg[2][DOF_LEG];
        float joint_power_leg[2][DOF_LEG];
        float tau_exp_leg[2][DOF_LEG];
        float current_exp_leg[2][DOF_LEG];
        char joint_state_leg[2][DOF_LEG];
        //uint16_t joint_state_leg[2][DOF_LEG];
        //------------hand
        float joint_q_hand[2][DOF_HAND];
        float joint_tau_hand[2][DOF_HAND];
        float vcap_hand[2];
        float eforce_hand[2][5];//5-fingers

        float joint_q_head_exp[DOF_HEAD];//rad
        float joint_qd_head_exp[DOF_HEAD];//rad/s
        float joint_q_waist_exp[DOF_WAIST];
        float joint_qd_waist_exp[DOF_WAIST];
        float joint_q_arm_exp[2][DOF_ARM];
        float joint_qd_arm_exp[2][DOF_ARM];
        float joint_q_leg_exp[2][DOF_LEG];
        float joint_qd_leg_exp[2][DOF_LEG];
        float joint_q_hand_exp[2][DOF_HAND];
        float joint_tau_hand_exp[2][DOF_HAND];


        uint16_t sensor_thumb_l [4];
        uint16_t sensor_index_finger_l [4];
        uint16_t sensor_middle_finger_l [4];
        uint16_t sensor_ring_finger_l [4];
        uint16_t sensor_little_finger_l [4];
        //uint16_t sensor_palm_l [63];

        uint16_t sensor_thumb_r [4];
        uint16_t sensor_index_finger_r [4];
        uint16_t sensor_middle_finger_r [4];
        uint16_t sensor_ring_finger_r [4];
        uint16_t sensor_little_finger_r [4];
        //uint16_t sensor_palm_r [63];


    };

    struct Robot_Feedback_Info
    {
        Robot_Feedback_Basic_Info  basic_info;
    };



    struct Servo_Cmd_Basic_Info
    {
        float ee_motion[2][DOF_ARM];
        float arm_cartesion[2][DOF_ARM];
        float dq_exp[2][DOF_ARM];
        float tau_exp[2][DOF_ARM];
        int q_enable[2][DOF_ARM];

        float q_exp_waist[DOF_WAIST];
        float dq_exp_waist[DOF_WAIST];
        float tau_exp_waist[DOF_WAIST];
        int q_enable_waist[DOF_WAIST];

        float q_exp_head[DOF_HEAD];
        float dq_exp_head[DOF_HEAD];
        float tau_exp_head[DOF_HEAD];
        int q_enable_head[DOF_HEAD];

        float q_exp_hand[2][DOF_HAND];
        float dq_exp_hand[2][DOF_HAND];
        float tau_exp_hand[2][DOF_HAND];
        float q_vcap_hand[2];
        int q_enable_hand[2][DOF_HAND];
    };
    struct RobotState
    {
        int motion_mode;
        int running_mode;
        int error;
        int reset_feedback;

    };

    struct Motion_Data_Recieve
    {
        float left_arm_rz;/* data */
        float left_arm_ry;
        float left_arm_rx;
        float left_arm_px;/* data */
        float left_arm_py;
        float left_arm_pz;
        float left_arm_belt;

        float right_arm_rz;/* data */
        float right_arm_ry;
        float right_arm_rx;
        float right_arm_px;/* data */
        float right_arm_py;
        float right_arm_pz;
        float right_arm_belt;

        float hand_data[2][DOF_HAND];
        float vcap_data[2];

    };

    struct MotionData
    {
        Servo_Cmd_Basic_Info  basic_cmd_info;

    };


    struct RemoteComputerData
    {

    };

    struct RobotCmd
    {
        int motion_mode;
        int running_mode;
        int enable = 0;
        int filter_enable;
        int joint_mode;  /////  0:POSITION, 1:VELOCITY, 2:CURRENT;
        int reset_error = 0;

    };
    struct JointCmd
    {
        Servo_Cmd_Basic_Info  basic_cmd_info;
    };



    struct RobotInfo
    {
        Robot_Feedback_Info  robot_feedback_info_;//机器人反馈数据
        MotionData    motion_data_;//下发给机器人动捕数据(weizi
        RobotState    robot_state_;
        RemoteComputerData  remote_computer_data_;
        RobotCmd      robot_cmd_, robot_cmd_send_;//下发给机器人指令
        JointCmd    joint_cmd_;//下发给机器人关节数据(angle
        Motion_Data_Recieve  motion_data_recieve_;//来自其他远程电脑的数据


    };






public:
    RobotData();
    ~RobotData();

    RobotInfo  robot_info_;
};

#endif // ROBOTSYSTEM_H
