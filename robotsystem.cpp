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
int RobotSystem::SetFk(const char * robot_ip, int port){
    udp_client_fk = new UdpClient(port, robot_ip);
    return 0;
}
int RobotSystem::SetRemoteDevice(const char *robot_ip, int port){
    udp_client_remote_device = new UdpClient(port, robot_ip);
    return 0;
}
int RobotSystem::SetAgv(const char *robot_ip, int port){
    udp_client_agv = new UdpClient(port, robot_ip);
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


    switch (motion_mode) {
        case IDLE:
            from_rcp=true;
            break;
        case MOTION_CAPTURE:
            if(from_rcp){
                float tx_left,ty_left,tz_left,rx_left,ry_left,rz_left;
                float tx_right,ty_right,tz_right,rx_right,ry_right,rz_right;
                if(!rcp_lock){
                    tx_left =  robot_data->robot_info_.motion_data_recieve_.left_arm_px;
                    ty_left =  robot_data->robot_info_.motion_data_recieve_.left_arm_py;
                    tz_left =  robot_data->robot_info_.motion_data_recieve_.left_arm_pz;
                    tx_right = robot_data->robot_info_.motion_data_recieve_.right_arm_px;
                    ty_right = robot_data->robot_info_.motion_data_recieve_.right_arm_py;
                    tz_right = robot_data->robot_info_.motion_data_recieve_.right_arm_pz;
                    if(std::sqrt(tx_left*tx_left+ty_left*ty_left+tz_left*tz_left)<=595&&std::sqrt(tx_right*tx_right+ty_right*ty_right+tz_right*tz_right)<=595){
                        memcpy(&robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion,
                               &robot_data->robot_info_.motion_data_recieve_, sizeof(RobotData::Motion_Data_Recieve));
                        memcpy(&robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand,
                               &robot_data->robot_info_.motion_data_recieve_.hand_data,
                               sizeof(robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand));
                        robot_data->robot_info_.motion_data_.basic_cmd_info.q_vcap_hand[0] = robot_data->robot_info_.motion_data_recieve_.vcap_data[0];
                        robot_data->robot_info_.motion_data_.basic_cmd_info.q_vcap_hand[1] = robot_data->robot_info_.motion_data_recieve_.vcap_data[1];
                        x_v=robot_data->robot_info_.motion_data_recieve_.agv_speed[0]*0.5;
                        y_v=robot_data->robot_info_.motion_data_recieve_.agv_speed[1]*0.5;
                        z_angle=robot_data->robot_info_.motion_data_recieve_.agv_speed[2]*0.5;
                        //std::cout<<"x: "<<x_v<<", y: "<<y_v<<", z: "<<z_angle<<std::endl;
                    }else{
                        qInfo()<<"RCP OUT OF RANGE";
                    }
                }else{
//                    tx_right = -311;//robot_data->robot_info_.motion_data_recieve_.right_arm_px;
//                    ty_right = -250;//robot_data->robot_info_.motion_data_recieve_.right_arm_py;
//                    tz_right = 34;//robot_data->robot_info_.motion_data_recieve_.right_arm_pz;
//                    rx_right = -2.8543;//robot_data->robot_info_.motion_data_recieve_.right_arm_rx;
//                    ry_right = 1.2392;//robot_data->robot_info_.motion_data_recieve_.right_arm_ry;
//                    rz_right = -1.8774;//robot_data->robot_info_.motion_data_recieve_.right_arm_rz;
                    tx_right = robot_data->robot_info_.motion_data_recieve_.right_arm_px;
                    ty_right = robot_data->robot_info_.motion_data_recieve_.right_arm_py;
                    tz_right = robot_data->robot_info_.motion_data_recieve_.right_arm_pz;
                    rx_right = robot_data->robot_info_.motion_data_recieve_.right_arm_rx;
                    ry_right = robot_data->robot_info_.motion_data_recieve_.right_arm_ry;
                    rz_right = robot_data->robot_info_.motion_data_recieve_.right_arm_rz;
                    Eigen::Matrix4f matrix_right;
                    matrix_right= Eigen::Matrix4f::Identity();
                    matrix_right(0, 3) = tx_right;      matrix_right(1, 3) = ty_right;      matrix_right(2, 3) = tz_right;
                    Eigen::Matrix3f rZ_right = Eigen::AngleAxisf(rz_right, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                    Eigen::Matrix3f rY_right = Eigen::AngleAxisf(ry_right, Eigen::Vector3f::UnitY()).toRotationMatrix();
                    Eigen::Matrix3f rX_right = Eigen::AngleAxisf(rx_right, Eigen::Vector3f::UnitX()).toRotationMatrix();
                    Eigen::Matrix3f rotation_matrix_right = rX_right * rY_right * rZ_right;
                    Eigen::Matrix3f rotation_matrix_right_new;
                    rotation_matrix_right_new << cos(ry_right)*cos(rz_right),-cos(ry_right)*sin(rz_right),sin(ry_right),
                              cos(rx_right)*sin(rz_right) + cos(rz_right)*sin(rx_right)*sin(ry_right), cos(rx_right)*cos(rz_right) - sin(rx_right)*sin(ry_right)*sin(rz_right), -cos(ry_right)*sin(rx_right),
                              sin(rx_right)*sin(rz_right) - cos(rx_right)*cos(rz_right)*sin(ry_right), cos(rz_right)*sin(rx_right) + cos(rx_right)*sin(ry_right)*sin(rz_right),  cos(rx_right)*cos(ry_right);
                    std::cout<<"rotation matrix old:  "<<rotation_matrix_right<<std::endl;
                    std::cout<<"rotation matrix new:  "<<rotation_matrix_right_new<<std::endl;
                    matrix_right.block<3, 3>(0, 0) = rotation_matrix_right;
//                    matrix_right<<  0.977336,   0.149192,   0.150186,   -592.195,
//                                    0.159035, -0.0492045,  -0.986046,    10.2946,
//                                    -0.13972,   0.987583, -0.0718161,     68.216,
//                                           0,          0,          0,          1;
                    std::cout<<"matrix right:  "<<matrix_right<<std::endl;
                    Eigen::Matrix4f matrix_BL_HL,matrix_rtl;;
                    matrix_rtl << 1,0,0,0,
                                0,-1,0,0,
                                0,0,-1,-405.2,
                                0.0, 0.0, 0.0, 1.0;
//                    matrix_HR_HL<<0.990951 ,-0.0311454 , -0.130563 ,   15.9598 ,
//                                  0.0347736,   0.999067,  0.0256014,   -495.646,
//                                   0.129644, -0.0299099,   0.991109,    48.4371,
//                                          0,          0,          0,          1;
                    std::cout<<"matrix_HR_HL:  "<<matrix_HR_HL<<std::endl;
                    matrix_BL_HL = matrix_rtl.inverse()*matrix_right*matrix_HR_HL;
                    std::cout<<matrix_BL_HL<<std::endl;
                    ry_left = atan2(matrix_BL_HL(0,2),std::sqrt(matrix_BL_HL(0,0)*matrix_BL_HL(0,0)+matrix_BL_HL(0,1)*matrix_BL_HL(0,1)));
                    if(std::abs(ry_left-M_PI/2.0)<1e-6){
                        rx_left=0; rz_left=atan2(matrix_BL_HL(1,0),matrix_BL_HL(1,1));
                    }else if(std::abs(ry_left-(-M_PI/2.0))<1e-6){
                        rx_left=0;rz_left=atan2(matrix_BL_HL(1,0),matrix_BL_HL(1,1));
                    }else{
                        rz_left = atan2(-matrix_BL_HL(0,1)/std::cos(ry_left),matrix_BL_HL(0,0)/std::cos(ry_left));
                        rx_left = atan2(-matrix_BL_HL(1,2)/std::cos(ry_left),matrix_BL_HL(2,2)/std::cos(ry_left));
                    }
                    tx_left = matrix_BL_HL(0,3); ty_left=matrix_BL_HL(1,3); tz_left=matrix_BL_HL(2,3);
                    float length_left = std::sqrt(tx_left*tx_left+ty_left*ty_left+tz_left*tz_left);
                    float length_right = std::sqrt(tx_right*tx_right+ty_right*ty_right+tz_right*tz_right);
                    if(length_left<=595&&length_right<=595){
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][0]=rz_left;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][1]=ry_left;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][2]=rx_left;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][3]=tx_left;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][4]=ty_left;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][5]=tz_left;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][6]=robot_data->robot_info_.motion_data_recieve_.left_arm_belt;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][0]=rz_right;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][1]=ry_right;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][2]=rx_right;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][3]=tx_right;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][4]=ty_right;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][5]=tz_right;
                        robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][6]=robot_data->robot_info_.motion_data_recieve_.right_arm_belt;
                        std::cout << "left: rz: " << rz_left << ", ry: " << ry_left << ", rx: " << rx_left << ", tx: " << tx_left << ", ty: " << ty_left << ", tz: " << tz_left << std::endl;

                    }else{
                        qInfo()<<"RCP LOCK OUT OF RANGE";
                        std::cout << "left: rz: " << rz_left << ", ry: " << ry_left << ", rx: " << rx_left << ", tx: " << tx_left << ", ty: " << ty_left << ", tz: " << tz_left << std::endl;

                    }
                    memcpy(&robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand,
                           &robot_data->robot_info_.motion_data_recieve_.hand_data,
                           sizeof(robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand));
                }
            }
            sendto(udp_client_robot_ee->sock_fd,
                (void *)&robot_data->robot_info_.motion_data_, sizeof(RobotData::MotionData), 0,
                (struct sockaddr *)&udp_client_robot_ee->addr_serv, udp_client_robot_ee->len);

            float simulate[16];
            simulate[0]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][0];
            simulate[1]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][1];
            simulate[2]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][2];
            simulate[3]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][3];
            simulate[4]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][4];
            simulate[5]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][5];
            simulate[6]=1.0;
            simulate[7]=1.0;
            simulate[8]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][0];
            simulate[9]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][1];
            simulate[10]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][2];
            simulate[11]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][3];
            simulate[12]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][4];
            simulate[13]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][5];
            simulate[14]=-1.0;
            simulate[15]=1.0;
            sendto(udp_client_fk->sock_fd,
                (void *)simulate, sizeof(RobotData::Test), 0,
                (struct sockaddr *)&udp_client_fk->addr_serv, udp_client_fk->len);

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

            sendto(udp_client_robot_manual->sock_fd,
                (void *)&robot_data->robot_info_.joint_cmd_, sizeof(RobotData::JointCmd), 0,
                (struct sockaddr *)&udp_client_robot_manual->addr_serv, udp_client_robot_manual->len);
            break;
//        case MOTION_CAPTURE:
//            sendto(udp_client_robot_ee->sock_fd,
//                (void *)&robot_data->robot_info_.motion_data_, sizeof(RobotData::MotionData), 0,
//                (struct sockaddr *)&udp_client_robot_ee->addr_serv, udp_client_robot_ee->len);
//            break;
        case DEMONSTRATOR:
//            double dv3_l[7],dv3_r[7];
//            double weizi_l[7],weizi_r[7];
//            double weizi_combined[14];
//            dv3_l[0]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][0]/57300;
//            dv3_l[1]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][1]/57300;
//            dv3_l[2]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][2]/57300;
//            dv3_l[3]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][3]/57300;
//            dv3_l[4]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][4]/57300;
//            dv3_l[5]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][5]/57300;
//            dv3_l[6]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][6]/57300;
//            dv3_r[0]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][0]/57300;
//            dv3_r[1]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][1]/57300;
//            dv3_r[2]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][2]/57300;
//            dv3_r[3]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][3]/57300;
//            dv3_r[4]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][4]/57300;
//            dv3_r[5]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][5]/57300;
//            dv3_r[6]=robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][6]/57300;

//            k_OLR2(dv3_l, weizi_l);
//            k_OLR2(dv3_r, weizi_r);

//            std::copy(std::begin(weizi_l), std::end(weizi_l), weizi_combined);
//            std::copy(std::begin(weizi_r), std::end(weizi_r), weizi_combined + 7);
            //float test[28];
            simulate[0]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][0];
            simulate[1]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][1];
            simulate[2]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][2];
            simulate[3]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][3];
            simulate[4]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][4];
            simulate[5]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][5];
            simulate[6]=1.0;
            simulate[7]=1.0;
            simulate[8]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][0];
            simulate[9]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][1];
            simulate[10]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][2];
            simulate[11]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][3];
            simulate[12]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][4];
            simulate[13]=robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][5];
            simulate[14]=-1.0;
            simulate[15]=1.0;
            sendto(udp_client_fk->sock_fd,
                (void *)simulate, sizeof(RobotData::Test), 0,
                (struct sockaddr *)&udp_client_fk->addr_serv, udp_client_fk->len);
            break;

        default:
            break;
    }

}

void RobotSystem::UdpClientAgv(){
    unsigned char send_data[] = {0x30, 0x14, 0x00, 0x0D, 0x90, 0x38,  // pin: 0x00000000
        0x01,                                // safemode
        0x00, 0x00, 0x00, 0x00,              // x_v (float: 0.0)
        0x00, 0x00, 0x00, 0x00,              // y_v (float: 0.0)
        0x3D, 0xCC, 0xCC, 0xCD,              // z_angle (float: 0.0)
        0x01,                                // speed_stage
        0x00,                                // navigation
        0x00,                                // map
        0x14, 0x3F };                        // CRC placeholder
//    if(running_mode==RobotSystem::INIT_OK){
//        x_v=0;y_v=0;z_angle=0;
//    }
    if((x_v<1&&x_v>-1)&&(y_v<1&&y_v>-1)&&(z_angle<1&&z_angle>-1)){
        floatToBytes(x_v, send_data + 7);
        floatToBytes(y_v, send_data + 11);
        floatToBytes(z_angle, send_data + 15);
    }else{
        std::cout<<"Danger!!!"<<std::endl;
        floatToBytes(0.0, send_data + 7);
        floatToBytes(0.0, send_data + 11);
        floatToBytes(0.0, send_data + 15);
    }


    // Calculate CRC
    unsigned short crc = calculateCRC(send_data, sizeof(send_data) - 2);
    send_data[23] = crc & 0xFF;
    send_data[22] = (crc >> 8) & 0xFF;

    char buffer[sizeof(send_data)];
    for (size_t i = 0; i < 24; ++i) {
        std::cout << std::hex << static_cast<int>(send_data[i]) << " ";
    }
    std::cout<<std::endl;
    std::cout<<"x: "<<x_v<<", y: "<<y_v<<", z: "<<z_angle<<std::endl;
    memcpy(buffer, &send_data, sizeof(send_data));
    sendto(udp_client_agv->sock_fd, buffer, sizeof(send_data), 0, (const struct sockaddr*)&udp_client_agv->addr_serv, udp_client_agv->len);
}

void RobotSystem::UdpClientRemoteRun(){

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
        case DEMONSTRATOR:
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

//agv
unsigned short RobotSystem::calculateCRC(const unsigned char* data, unsigned int length) {
    unsigned short crc = 0xFFFF;
    for (unsigned int i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void RobotSystem::floatToBytes(float value, unsigned char* bytes) {
    unsigned char* floatBytes = reinterpret_cast<unsigned char*>(&value);
    // 将float值按大端序写入bytes
    bytes[0] = floatBytes[3];
    bytes[1] = floatBytes[2];
    bytes[2] = floatBytes[1];
    bytes[3] = floatBytes[0];
}

