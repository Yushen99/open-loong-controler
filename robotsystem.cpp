#include "robotsystem.h"








int RobotSystem::SetMotionCaptureComm(const char * robot_ip, int port){
    udp_client_robot_ee = new UdpClient(port, robot_ip);
    //float k_arc2angle = 1.0/(2*3.1415926) *360000;
    float test_ee[2][DOF_ARM] = {{-1.5708, 1.5708, 0, -270.0, 225.28, 82.69, 0.7584},
                                 {1.5708, 1.5708, 0, -270.0, -225.28, 82.69, -0.7584}};
    for(int i=0; i<2; i++){
        for(int j=0; j<3; j++){
            test_ee[i][j] = test_ee[i][j] ;
        }
    }
    test_ee[0][6] = test_ee[0][6] ;
    test_ee[1][6] = test_ee[1][6] ;

    memcpy(&robot_data->robot_info_.motion_data_recieve_, test_ee, sizeof(test_ee));
    return 0;
}
int RobotSystem::SetManualComm(const char * robot_ip, int port){
    udp_client_robot_manual = new UdpClient(port, robot_ip);
    return 0;
}
int RobotSystem::SetCmdComm(const char * robot_ip, int port){
    udp_client_robot_command = new UdpClient(port, robot_ip);
    return 0;
}

int RobotSystem::CommunicationStart(){
    udp_server->recv_data();
    return 0;
}

void RobotSystem::UdpClientsRun(){

    ModeUpdate();
    sendto(udp_client_robot_command->sock_fd,
        (void *)&robot_data->robot_info_.robot_cmd_send_, sizeof(RobotData::RobotCmd), 0,
        (struct sockaddr *)&udp_client_robot_command->addr_serv, udp_client_robot_command->len);
//qInfo()<<robot_data->robot_info_.robot_cmd_send_.enable;

    switch (motion_mode) {
        case IDLE:
            from_rcp=true;
            break;
        case MOTION_CAPTURE:
            if(from_rcp){
                float lx = robot_data->robot_info_.motion_data_recieve_.left_arm_px;
                float ly = robot_data->robot_info_.motion_data_recieve_.left_arm_py;
                float lz = robot_data->robot_info_.motion_data_recieve_.left_arm_pz;
                float rx = robot_data->robot_info_.motion_data_recieve_.right_arm_px;
                float ry = robot_data->robot_info_.motion_data_recieve_.right_arm_py;
                float rz = robot_data->robot_info_.motion_data_recieve_.right_arm_pz;
                if(std::sqrt(lx*lx+ly*ly+lz*lz)<590&&std::sqrt(rx*rx+ry*ry+rz*rz)<590){
                    memcpy(&robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion,
                           &robot_data->robot_info_.motion_data_recieve_, sizeof(RobotData::Motion_Data_Recieve));
                    memcpy(&robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand,
                           &robot_data->robot_info_.motion_data_recieve_.hand_data,
                           sizeof(robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand));
                    robot_data->robot_info_.motion_data_.basic_cmd_info.q_vcap_hand[0] =
                            robot_data->robot_info_.motion_data_recieve_.vcap_data[0];
                }

            }
            sendto(udp_client_robot_ee->sock_fd,
                (void *)&robot_data->robot_info_.motion_data_, sizeof(RobotData::MotionData), 0,
                (struct sockaddr *)&udp_client_robot_ee->addr_serv, udp_client_robot_ee->len);
            break;
        case MANUAL:
//            memcpy(&robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion,
//                   &robot_data->robot_info_.motion_data_recieve_.left_arm_rz,
//                   sizeof(robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion));
//            memcpy(&robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand,
//                   &robot_data->robot_info_.motion_data_recieve_.hand_data,
//                   sizeof(robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand));
//            robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_vcap_hand[0] =
//                    robot_data->robot_info_.motion_data_recieve_.vcap_data[0];
//            memcpy(&robot_data->robot_info_.joint_cmd_.basic_cmd_info.arm_cartesion,
//                   &robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion,
//        sizeof(robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion));
            sendto(udp_client_robot_manual->sock_fd,
                (void *)&robot_data->robot_info_.joint_cmd_, sizeof(RobotData::JointCmd), 0,
                (struct sockaddr *)&udp_client_robot_manual->addr_serv, udp_client_robot_manual->len);
            break;
//        case MOTION_CAPTURE:
//            sendto(udp_client_robot_ee->sock_fd,
//                (void *)&robot_data->robot_info_.motion_data_, sizeof(RobotData::MotionData), 0,
//                (struct sockaddr *)&udp_client_robot_ee->addr_serv, udp_client_robot_ee->len);
//            break;
        case MOVEL:
            sendto(udp_client_robot_manual->sock_fd,
                (void *)&robot_data->robot_info_.joint_cmd_, sizeof(RobotData::JointCmd), 0,
                (struct sockaddr *)&udp_client_robot_manual->addr_serv, udp_client_robot_manual->len);
            break;

        default:
            break;
    }

}


int RobotSystem::ModeUpdate(){

    switch (motion_mode) {
        case IDLE:
            robot_data->robot_info_.robot_cmd_send_.motion_mode = 0;  break;
        case DYNAMIC:
            robot_data->robot_info_.robot_cmd_send_.motion_mode = 1; break;
        case MANUAL:
            robot_data->robot_info_.robot_cmd_send_.motion_mode = 2; break;
        case AUTO:
            robot_data->robot_info_.robot_cmd_send_.motion_mode = 3; break;
        case MOVEL:
            robot_data->robot_info_.robot_cmd_send_.motion_mode = 4; break;
        case MOTION_CAPTURE:
            robot_data->robot_info_.robot_cmd_send_.motion_mode = 5; break;
        default:
            break;
    }
    switch (running_mode) {

        case INIT:
            robot_data->robot_info_.robot_cmd_send_.running_mode = 0;break;
        case INIT_OK:
            robot_data->robot_info_.robot_cmd_send_.running_mode = 1; break;
        case READY:
            robot_data->robot_info_.robot_cmd_send_.running_mode = 2; break;
        case READY_OK:
            robot_data->robot_info_.robot_cmd_send_.running_mode = 4;break;
        case RUN:
            running_mode = RUN; break;
        case STOP:
            running_mode = STOP; break;
        case DISABLE:
            running_mode = DISABLE; break;
        case ERROR:
            running_mode = ERROR; break;
        default:
            break;
    }
    return 0;
}


int RobotSystem::Enable(){
    robot_data->robot_info_.robot_cmd_send_.enable = 1;

    for(int i=0; i<2; i++){
        for(int j=0; j<DOF_ARM; j++){
            robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_enable[i][j] = 1.0;
            robot_data->robot_info_.motion_data_.basic_cmd_info.q_enable[i][j] = 1.0;
        }
    }
    for(int j=0; j<DOF_WAIST; j++){
        robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_enable_waist[j] = 1.0;
        robot_data->robot_info_.motion_data_.basic_cmd_info.q_enable_waist[j] = 1.0;
    }
    for(int j=0; j<DOF_HEAD; j++){
        robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_enable_head[j] = 1.0;
        robot_data->robot_info_.motion_data_.basic_cmd_info.q_enable_head[j] = 1.0;
    }
    return 0;
}


int RobotSystem::Disable(){
    robot_data->robot_info_.robot_cmd_send_.enable = 0;

    for(int i=0; i<2; i++){
        for(int j=0; j<DOF_ARM; j++){
            robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_enable[i][j] = 0.0;
            robot_data->robot_info_.motion_data_.basic_cmd_info.q_enable[i][j] = 0.0;
        }
    }
    for(int j=0; j<DOF_WAIST; j++){
        robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_enable_waist[j] = 0.0;
        robot_data->robot_info_.motion_data_.basic_cmd_info.q_enable_waist[j] = 0.0;
    }
    for(int j=0; j<DOF_HEAD; j++){
        robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_enable_head[j] = 0.0;
        robot_data->robot_info_.motion_data_.basic_cmd_info.q_enable_head[j] = 0.0;
    }
    return 0;
}



