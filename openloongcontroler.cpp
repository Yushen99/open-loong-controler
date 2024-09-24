#include "openloongcontroler.h"
#include "ui_openloongcontroler.h"
#include <iostream>
#include <QDateTime>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

//#include "k_ORL2/k_OLR2.c"
//#include "k_ORL2/k_OLR2_initialize.c"
//#include "k_ORL2/k_OLR2_terminate.c"
//#include "k_ORL2/rt_nonfinite.c"
//#include "k_ORL2/rtGetInf.c"
//#include "k_ORL2/rtGetNaN.c"
//#include "k_ORL2/norm.c"
//#include "k_ORL2/CoordinateTrans.c"

OpenLoongControler::OpenLoongControler(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::OpenLoongControler)
{
    ui->setupUi(this);
    setFocusPolicy(Qt::ClickFocus);
    initLabel();
    robot_system = new RobotSystem();
    robot_system->CommunicationStart();

    //UI update timer
    show_timer = new QTimer(this);
    connect(show_timer,&QTimer::timeout, this, &OpenLoongControler::ShowInfomation);
    show_timer->start(10);

    //keyboard timer
    keyRespondTimer = new QTimer(this);
    connect(keyRespondTimer, &QTimer::timeout, this, &OpenLoongControler::handleKeyPress);

    //demonstrator timer
    demonstrator_timer = new QTimer(this);
    connect(demonstrator_timer, &QTimer::timeout, this, &OpenLoongControler::handleDemonstrator);
    demonstrator_run_timer = new QTimer(this);
    connect(demonstrator_run_timer, &QTimer::timeout, this, &OpenLoongControler::handleDemonstrator_run);

    //rcp timer
    rcp_timer = new QTimer(this);
    connect(rcp_timer, &QTimer::timeout, this, &OpenLoongControler::handleRCP);
    rcp_run_timer = new QTimer(this);
    connect(rcp_run_timer, &QTimer::timeout, this, &OpenLoongControler::handleRCP_run);

    //signal to demonstrator thread
    connect(this,&OpenLoongControler::startRecording,&thread_demonstrator,&ThreadDemonstrator::startRecording);
    connect(this,&OpenLoongControler::stopRecording,&thread_demonstrator,&ThreadDemonstrator::stopRecording);
    connect(this,&OpenLoongControler::clearRecording,&thread_demonstrator,&ThreadDemonstrator::clearRecording);
    connect(this,&OpenLoongControler::saveRecording,&thread_demonstrator,&ThreadDemonstrator::saveRecording);

    //draw timer
    draw_timer = new QTimer(this);
    connect(draw_timer,&QTimer::timeout,this,&OpenLoongControler::drawHand_update);
    QLinearGradient linear=QLinearGradient(QPoint(0,0),QPoint(255,0));
    linear.setColorAt(0, Qt::blue);
    linear.setColorAt(0.4, Qt::blue);
    linear.setColorAt(0.5, Qt::cyan);
    linear.setColorAt(0.6, Qt::green);
    linear.setColorAt(0.8, Qt::yellow);
    linear.setColorAt(0.95, Qt::red);
    QImage img(256,1,QImage::Format_ARGB32);
    QPainter painterImg(&img);
    painterImg.fillRect(img.rect(),linear);
    quint32 alpha=0;
    for(quint32 i=0;i<256;i++){
        alpha=255/255.0*i;
        colorList[i]=(img.pixel(i,0)&0x00FFFFFF)|(alpha<<24);
    }

    robot_system->robot_data->robot_info_.robot_cmd_send_.filter_enable = true;

    ui->widget_hand->installEventFilter(this);
}


OpenLoongControler::~OpenLoongControler()
{
    delete ui;
    delete robot_system;
}

//Main UI
void OpenLoongControler::on_pushButton_oneshot_demonstrator_clicked()
{
    //robot_system->robot_data->robot_info_.robot_state_.motion_mode = 4;
    robot_system->motion_mode = RobotSystem::DEMONSTRATOR;
}

void OpenLoongControler::on_pushButton_oneshot_idle_clicked()
{
    robot_system->motion_mode = RobotSystem::IDLE;
//    robot_system->robot_data->robot_info_.robot_state_.motion_mode = 0;
    bool_manualEnabled = false;
    ui->label_manual_keyboard->setText("Off");
    ui->label_rcp_mora->setText("Auto");
    robot_system->x_v=0.0;
    robot_system->y_v=0.0;
    robot_system->z_angle=0.0;
}

void OpenLoongControler::on_pushButton_oneshot_manual_clicked()
{
    //enable or disable manual function
    robot_system->motion_mode = RobotSystem::MANUAL;
    bool_manualEnabled = true;
    if(bool_manualEnabled){
//        robot_system->robot_data->robot_info_.robot_state_.motion_mode = 2;
        ui->label_manual_keyboard->setText("On");
    }else{
        ui->label_manual_keyboard->setText("Off");
    }

    memcpy(&robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion,
           &robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm,
           sizeof(robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm));
    memcpy(&robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_waist,
           &robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist,
           sizeof(robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist));
    memcpy(&robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_head,
           &robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head,
           sizeof(robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head));
    qInfo()<<robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][0]<<" "<<
             robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][1]<<" "<<
             robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][2]<<" "<<
             robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][3]<<" "<<
             robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][4]<<" "<<
             robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][5]<<" "<<
             robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][6]<<" ";
}

void OpenLoongControler::on_pushButton_oneshot_rcp_clicked()
{
    robot_system->motion_mode = RobotSystem::MOTION_CAPTURE;
}

void OpenLoongControler::on_pushButton_stop_clicked()
{
    robot_system->running_mode = RobotSystem::INIT_OK;
}

void OpenLoongControler::on_pushButton_reset_clicked()
{
    robot_system->running_mode = RobotSystem::READY;
}

//connect
void OpenLoongControler::on_pushButton_init_connect_clicked()
{
    std::string robot_ip = ui->lineEdit_robot_ip->text().toStdString();
    std::string server_ip = ui->lineEdit_server_ip->text().toStdString();
    std::string device_ip = ui->lineEdit_device_ip->text().toStdString();
    robot_system->SetCmdComm(robot_ip.data(), PORT_MOTION_CMD);
    robot_system->SetManualComm(robot_ip.data(), PORT_MOTION_JOINT);
    robot_system->SetMotionCaptureComm(robot_ip.data(), PORT_MOTION_EE);
    robot_system->SetFk(server_ip.data(),5005);
    robot_system->SetRemoteDevice(device_ip.data(),0x5000);
    robot_system->SetAgv("192.168.1.41",3838);

    clients_com_timer = new QTimer(this);
    connect(clients_com_timer,&QTimer::timeout, robot_system, &RobotSystem::UdpClientsRun);
    clients_com_timer->start(2);

    clients_agv = new QTimer(this);
    connect(clients_agv,&QTimer::timeout, robot_system, &RobotSystem::UdpClientAgv);
    clients_agv->start(100);

//    clients_remote_timer = new QTimer(this);
//    connect(clients_remote_timer,&QTimer::timeout, robot_system, &RobotSystem::UdpClientRemoteRun);
//    clients_remote_timer->start(10);

}

void OpenLoongControler::on_pushButton_enable_clicked()
{
    robot_system->Enable();
}

void OpenLoongControler::on_pushButton_disable_clicked()
{
    robot_system->Disable();
}

void OpenLoongControler::on_radioButton_filter_on_clicked()
{
    robot_system->robot_data->robot_info_.robot_cmd_send_.filter_enable = 1;
}

void OpenLoongControler::on_radioButton_filter_off_clicked()
{
    robot_system->robot_data->robot_info_.robot_cmd_send_.filter_enable = 0;
}

//manual
void OpenLoongControler::initLabel(){
    map_pos[Qt::Key_1] = LabelInfo(ui->label_feedback_l_A_1,ADD,ARM,0,0);
    map_pos[Qt::Key_2] = LabelInfo(ui->label_feedback_l_A_2,ADD,ARM,0,1);
    map_pos[Qt::Key_3] = LabelInfo(ui->label_feedback_l_A_3,ADD,ARM,0,2);
    map_pos[Qt::Key_4] = LabelInfo(ui->label_feedback_l_A_4,ADD,ARM,0,3);
    map_pos[Qt::Key_5] = LabelInfo(ui->label_feedback_l_A_5,ADD,ARM,0,4);
    map_pos[Qt::Key_6] = LabelInfo(ui->label_feedback_l_A_6,ADD,ARM,0,5);
    map_pos[Qt::Key_7] = LabelInfo(ui->label_feedback_l_A_7,ADD,ARM,0,6);
    map_pos[Qt::Key_Q] = LabelInfo(ui->label_feedback_l_A_1,MINUS,ARM,0,0);
    map_pos[Qt::Key_W] = LabelInfo(ui->label_feedback_l_A_2,MINUS,ARM,0,1);
    map_pos[Qt::Key_E] = LabelInfo(ui->label_feedback_l_A_3,MINUS,ARM,0,2);
    map_pos[Qt::Key_R] = LabelInfo(ui->label_feedback_l_A_4,MINUS,ARM,0,3);
    map_pos[Qt::Key_T] = LabelInfo(ui->label_feedback_l_A_5,MINUS,ARM,0,4);
    map_pos[Qt::Key_Y] = LabelInfo(ui->label_feedback_l_A_6,MINUS,ARM,0,5);
    map_pos[Qt::Key_U] = LabelInfo(ui->label_feedback_l_A_7,MINUS,ARM,0,6);

    map_pos[Qt::Key_A] = LabelInfo(ui->label_feedback_l_L_1,ADD,ARM,1,0);
    map_pos[Qt::Key_S] = LabelInfo(ui->label_feedback_l_L_2,ADD,ARM,1,1);
    map_pos[Qt::Key_D] = LabelInfo(ui->label_feedback_l_L_3,ADD,ARM,1,2);
    map_pos[Qt::Key_F] = LabelInfo(ui->label_feedback_l_L_4,ADD,ARM,1,3);
    map_pos[Qt::Key_G] = LabelInfo(ui->label_feedback_l_L_5,ADD,ARM,1,4);
    map_pos[Qt::Key_H] = LabelInfo(ui->label_feedback_l_L_6,ADD,ARM,1,5);
    map_pos[Qt::Key_J] = LabelInfo(ui->label_feedback_l_L_7,ADD,ARM,1,6);
    map_pos[Qt::Key_Z] = LabelInfo(ui->label_feedback_l_L_1,MINUS,ARM,1,0);
    map_pos[Qt::Key_X] = LabelInfo(ui->label_feedback_l_L_2,MINUS,ARM,1,1);
    map_pos[Qt::Key_C] = LabelInfo(ui->label_feedback_l_L_3,MINUS,ARM,1,2);
    map_pos[Qt::Key_V] = LabelInfo(ui->label_feedback_l_L_4,MINUS,ARM,1,3);
    map_pos[Qt::Key_B] = LabelInfo(ui->label_feedback_l_L_5,MINUS,ARM,1,4);
    map_pos[Qt::Key_N] = LabelInfo(ui->label_feedback_l_L_6,MINUS,ARM,1,5);
    map_pos[Qt::Key_M] = LabelInfo(ui->label_feedback_l_L_7,MINUS,ARM,1,6);

    map_pos_ctrl[Qt::Key_1] = LabelInfo(ui->label_feedback_waist1_pos,ADD,WAIST,0,0);
    map_pos_ctrl[Qt::Key_2] = LabelInfo(ui->label_feedback_waist2_pos,ADD,WAIST,0,1);
    map_pos_ctrl[Qt::Key_3] = LabelInfo(ui->label_feedback_waist3_pos,ADD,WAIST,0,2);
    map_pos_ctrl[Qt::Key_4] = LabelInfo(ui->label_feedback_head1_pos,ADD,HEAD,0,0);
    map_pos_ctrl[Qt::Key_5] = LabelInfo(ui->label_feedback_head2_pos,ADD,HEAD,0,1);
    map_pos_ctrl[Qt::Key_Q] = LabelInfo(ui->label_feedback_waist1_pos,MINUS,WAIST,0,0);
    map_pos_ctrl[Qt::Key_W] = LabelInfo(ui->label_feedback_waist2_pos,MINUS,WAIST,0,1);
    map_pos_ctrl[Qt::Key_E] = LabelInfo(ui->label_feedback_waist3_pos,MINUS,WAIST,0,2);
    map_pos_ctrl[Qt::Key_R] = LabelInfo(ui->label_feedback_head1_pos,MINUS,HEAD,0,0);
    map_pos_ctrl[Qt::Key_T] = LabelInfo(ui->label_feedback_head2_pos,MINUS,HEAD,0,1);

    map_curr[Qt::Key_1] = LabelInfo(ui->label_feedback_cur_l_A_1,ADD,ARM,0,0);
    map_curr[Qt::Key_2] = LabelInfo(ui->label_feedback_cur_l_A_2,ADD,ARM,0,1);
    map_curr[Qt::Key_3] = LabelInfo(ui->label_feedback_cur_l_A_3,ADD,ARM,0,2);
    map_curr[Qt::Key_4] = LabelInfo(ui->label_feedback_cur_l_A_4,ADD,ARM,0,3);
    map_curr[Qt::Key_5] = LabelInfo(ui->label_feedback_cur_l_A_5,ADD,ARM,0,4);
    map_curr[Qt::Key_6] = LabelInfo(ui->label_feedback_cur_l_A_6,ADD,ARM,0,5);
    map_curr[Qt::Key_7] = LabelInfo(ui->label_feedback_cur_l_A_7,ADD,ARM,0,6);
    map_curr[Qt::Key_Q] = LabelInfo(ui->label_feedback_cur_l_A_1,MINUS,ARM,0,0);
    map_curr[Qt::Key_W] = LabelInfo(ui->label_feedback_cur_l_A_2,MINUS,ARM,0,1);
    map_curr[Qt::Key_E] = LabelInfo(ui->label_feedback_cur_l_A_3,MINUS,ARM,0,2);
    map_curr[Qt::Key_R] = LabelInfo(ui->label_feedback_cur_l_A_4,MINUS,ARM,0,3);
    map_curr[Qt::Key_T] = LabelInfo(ui->label_feedback_cur_l_A_5,MINUS,ARM,0,4);
    map_curr[Qt::Key_Y] = LabelInfo(ui->label_feedback_cur_l_A_6,MINUS,ARM,0,5);
    map_curr[Qt::Key_U] = LabelInfo(ui->label_feedback_cur_l_A_7,MINUS,ARM,0,6);

    map_curr[Qt::Key_A] = LabelInfo(ui->label_feedback_cur_l_L_1,ADD,ARM,1,0);
    map_curr[Qt::Key_S] = LabelInfo(ui->label_feedback_cur_l_L_2,ADD,ARM,1,1);
    map_curr[Qt::Key_D] = LabelInfo(ui->label_feedback_cur_l_L_3,ADD,ARM,1,2);
    map_curr[Qt::Key_F] = LabelInfo(ui->label_feedback_cur_l_L_4,ADD,ARM,1,3);
    map_curr[Qt::Key_G] = LabelInfo(ui->label_feedback_cur_l_L_5,ADD,ARM,1,4);
    map_curr[Qt::Key_H] = LabelInfo(ui->label_feedback_cur_l_L_6,ADD,ARM,1,5);
    map_curr[Qt::Key_J] = LabelInfo(ui->label_feedback_cur_l_L_7,ADD,ARM,1,6);
    map_curr[Qt::Key_Z] = LabelInfo(ui->label_feedback_cur_l_L_1,MINUS,ARM,1,0);
    map_curr[Qt::Key_X] = LabelInfo(ui->label_feedback_cur_l_L_2,MINUS,ARM,1,1);
    map_curr[Qt::Key_C] = LabelInfo(ui->label_feedback_cur_l_L_3,MINUS,ARM,1,2);
    map_curr[Qt::Key_V] = LabelInfo(ui->label_feedback_cur_l_L_4,MINUS,ARM,1,3);
    map_curr[Qt::Key_B] = LabelInfo(ui->label_feedback_cur_l_L_5,MINUS,ARM,1,4);
    map_curr[Qt::Key_N] = LabelInfo(ui->label_feedback_cur_l_L_6,MINUS,ARM,1,5);
    map_curr[Qt::Key_M] = LabelInfo(ui->label_feedback_cur_l_L_7,MINUS,ARM,1,6);

    map_curr_ctrl[Qt::Key_1] = LabelInfo(ui->label_feedback_waist1_curr,ADD,WAIST,0,0);
    map_curr_ctrl[Qt::Key_2] = LabelInfo(ui->label_feedback_waist2_curr,ADD,WAIST,0,1);
    map_curr_ctrl[Qt::Key_3] = LabelInfo(ui->label_feedback_waist3_curr,ADD,WAIST,0,2);
    map_curr_ctrl[Qt::Key_4] = LabelInfo(ui->label_feedback_head1_curr,ADD,HEAD,0,0);
    map_curr_ctrl[Qt::Key_5] = LabelInfo(ui->label_feedback_head2_curr,ADD,HEAD,0,1);
    map_curr_ctrl[Qt::Key_Q] = LabelInfo(ui->label_feedback_waist1_curr,MINUS,WAIST,0,0);
    map_curr_ctrl[Qt::Key_W] = LabelInfo(ui->label_feedback_waist2_curr,MINUS,WAIST,0,1);
    map_curr_ctrl[Qt::Key_E] = LabelInfo(ui->label_feedback_waist3_curr,MINUS,WAIST,0,2);
    map_curr_ctrl[Qt::Key_R] = LabelInfo(ui->label_feedback_head1_curr,MINUS,HEAD,0,0);
    map_curr_ctrl[Qt::Key_T] = LabelInfo(ui->label_feedback_head2_curr,MINUS,HEAD,0,1);

    map_pos_alt[Qt::Key_1] = LabelInfo(ui->label_feedback_l_H_1,ADD,HAND,0,0);
    map_pos_alt[Qt::Key_2] = LabelInfo(ui->label_feedback_l_H_2,ADD,HAND,0,1);
    map_pos_alt[Qt::Key_3] = LabelInfo(ui->label_feedback_l_H_3,ADD,HAND,0,2);
    map_pos_alt[Qt::Key_4] = LabelInfo(ui->label_feedback_l_H_4,ADD,HAND,0,3);
    map_pos_alt[Qt::Key_5] = LabelInfo(ui->label_feedback_l_H_5,ADD,HAND,0,4);
    map_pos_alt[Qt::Key_6] = LabelInfo(ui->label_feedback_l_H_6,ADD,HAND,0,5);
    map_pos_alt[Qt::Key_Q] = LabelInfo(ui->label_feedback_l_H_1,MINUS,HAND,0,0);
    map_pos_alt[Qt::Key_W] = LabelInfo(ui->label_feedback_l_H_2,MINUS,HAND,0,1);
    map_pos_alt[Qt::Key_E] = LabelInfo(ui->label_feedback_l_H_3,MINUS,HAND,0,2);
    map_pos_alt[Qt::Key_R] = LabelInfo(ui->label_feedback_l_H_4,MINUS,HAND,0,3);
    map_pos_alt[Qt::Key_T] = LabelInfo(ui->label_feedback_l_H_5,MINUS,HAND,0,4);
    map_pos_alt[Qt::Key_Y] = LabelInfo(ui->label_feedback_l_H_6,MINUS,HAND,0,5);

    map_pos_alt[Qt::Key_A] = LabelInfo(ui->label_feedback_r_H_1,ADD,HAND,1,0);
    map_pos_alt[Qt::Key_S] = LabelInfo(ui->label_feedback_r_H_2,ADD,HAND,1,1);
    map_pos_alt[Qt::Key_D] = LabelInfo(ui->label_feedback_r_H_3,ADD,HAND,1,2);
    map_pos_alt[Qt::Key_F] = LabelInfo(ui->label_feedback_r_H_4,ADD,HAND,1,3);
    map_pos_alt[Qt::Key_G] = LabelInfo(ui->label_feedback_r_H_5,ADD,HAND,1,4);
    map_pos_alt[Qt::Key_H] = LabelInfo(ui->label_feedback_r_H_6,ADD,HAND,1,5);
    map_pos_alt[Qt::Key_Z] = LabelInfo(ui->label_feedback_r_H_1,MINUS,HAND,1,0);
    map_pos_alt[Qt::Key_X] = LabelInfo(ui->label_feedback_r_H_2,MINUS,HAND,1,1);
    map_pos_alt[Qt::Key_C] = LabelInfo(ui->label_feedback_r_H_3,MINUS,HAND,1,2);
    map_pos_alt[Qt::Key_V] = LabelInfo(ui->label_feedback_r_H_4,MINUS,HAND,1,3);
    map_pos_alt[Qt::Key_B] = LabelInfo(ui->label_feedback_r_H_5,MINUS,HAND,1,4);
    map_pos_alt[Qt::Key_N] = LabelInfo(ui->label_feedback_r_H_6,MINUS,HAND,1,5);

    map_curr_alt[Qt::Key_1] = LabelInfo(ui->label_feedback_curr_l_H_1,ADD,HAND,0,0);
    map_curr_alt[Qt::Key_2] = LabelInfo(ui->label_feedback_curr_l_H_2,ADD,HAND,0,1);
    map_curr_alt[Qt::Key_3] = LabelInfo(ui->label_feedback_curr_l_H_3,ADD,HAND,0,2);
    map_curr_alt[Qt::Key_4] = LabelInfo(ui->label_feedback_curr_l_H_4,ADD,HAND,0,3);
    map_curr_alt[Qt::Key_5] = LabelInfo(ui->label_feedback_curr_l_H_5,ADD,HAND,0,4);
    map_curr_alt[Qt::Key_6] = LabelInfo(ui->label_feedback_curr_l_H_6,ADD,HAND,0,5);
    map_curr_alt[Qt::Key_Q] = LabelInfo(ui->label_feedback_curr_l_H_1,MINUS,HAND,0,0);
    map_curr_alt[Qt::Key_W] = LabelInfo(ui->label_feedback_curr_l_H_2,MINUS,HAND,0,1);
    map_curr_alt[Qt::Key_E] = LabelInfo(ui->label_feedback_curr_l_H_3,MINUS,HAND,0,2);
    map_curr_alt[Qt::Key_R] = LabelInfo(ui->label_feedback_curr_l_H_4,MINUS,HAND,0,3);
    map_curr_alt[Qt::Key_T] = LabelInfo(ui->label_feedback_curr_l_H_5,MINUS,HAND,0,4);
    map_curr_alt[Qt::Key_Y] = LabelInfo(ui->label_feedback_curr_l_H_6,MINUS,HAND,0,5);

    map_curr_alt[Qt::Key_A] = LabelInfo(ui->label_feedback_curr_r_H_1,ADD,HAND,1,0);
    map_curr_alt[Qt::Key_S] = LabelInfo(ui->label_feedback_curr_r_H_2,ADD,HAND,1,1);
    map_curr_alt[Qt::Key_D] = LabelInfo(ui->label_feedback_curr_r_H_3,ADD,HAND,1,2);
    map_curr_alt[Qt::Key_F] = LabelInfo(ui->label_feedback_curr_r_H_4,ADD,HAND,1,3);
    map_curr_alt[Qt::Key_G] = LabelInfo(ui->label_feedback_curr_r_H_5,ADD,HAND,1,4);
    map_curr_alt[Qt::Key_H] = LabelInfo(ui->label_feedback_curr_r_H_6,ADD,HAND,1,5);
    map_curr_alt[Qt::Key_Z] = LabelInfo(ui->label_feedback_curr_r_H_1,MINUS,HAND,1,0);
    map_curr_alt[Qt::Key_X] = LabelInfo(ui->label_feedback_curr_r_H_2,MINUS,HAND,1,1);
    map_curr_alt[Qt::Key_C] = LabelInfo(ui->label_feedback_curr_r_H_3,MINUS,HAND,1,2);
    map_curr_alt[Qt::Key_V] = LabelInfo(ui->label_feedback_curr_r_H_4,MINUS,HAND,1,3);
    map_curr_alt[Qt::Key_B] = LabelInfo(ui->label_feedback_curr_r_H_5,MINUS,HAND,1,4);
    map_curr_alt[Qt::Key_N] = LabelInfo(ui->label_feedback_curr_r_H_6,MINUS,HAND,1,5);

    map_rcp[Qt::Key_1] = LabelInfo(ui->label_rcp_motion_l_1,ADD,ARM,0,0);
    map_rcp[Qt::Key_2] = LabelInfo(ui->label_rcp_motion_l_2,ADD,ARM,0,1);
    map_rcp[Qt::Key_3] = LabelInfo(ui->label_rcp_motion_l_3,ADD,ARM,0,2);
    map_rcp[Qt::Key_4] = LabelInfo(ui->label_rcp_motion_l_4,ADD,ARM,0,3);
    map_rcp[Qt::Key_5] = LabelInfo(ui->label_rcp_motion_l_5,ADD,ARM,0,4);
    map_rcp[Qt::Key_6] = LabelInfo(ui->label_rcp_motion_l_6,ADD,ARM,0,5);
    map_rcp[Qt::Key_7] = LabelInfo(ui->label_rcp_motion_l_7,ADD,ARM,0,6);
    map_rcp[Qt::Key_Q] = LabelInfo(ui->label_rcp_motion_l_1,MINUS,ARM,0,0);
    map_rcp[Qt::Key_W] = LabelInfo(ui->label_rcp_motion_l_2,MINUS,ARM,0,1);
    map_rcp[Qt::Key_E] = LabelInfo(ui->label_rcp_motion_l_3,MINUS,ARM,0,2);
    map_rcp[Qt::Key_R] = LabelInfo(ui->label_rcp_motion_l_4,MINUS,ARM,0,3);
    map_rcp[Qt::Key_T] = LabelInfo(ui->label_rcp_motion_l_5,MINUS,ARM,0,4);
    map_rcp[Qt::Key_Y] = LabelInfo(ui->label_rcp_motion_l_6,MINUS,ARM,0,5);
    map_rcp[Qt::Key_U] = LabelInfo(ui->label_rcp_motion_l_7,MINUS,ARM,0,6);

    map_rcp[Qt::Key_A] = LabelInfo(ui->label_rcp_motion_r_1,ADD,ARM,1,0);
    map_rcp[Qt::Key_S] = LabelInfo(ui->label_rcp_motion_r_2,ADD,ARM,1,1);
    map_rcp[Qt::Key_D] = LabelInfo(ui->label_rcp_motion_r_3,ADD,ARM,1,2);
    map_rcp[Qt::Key_F] = LabelInfo(ui->label_rcp_motion_r_4,ADD,ARM,1,3);
    map_rcp[Qt::Key_G] = LabelInfo(ui->label_rcp_motion_r_5,ADD,ARM,1,4);
    map_rcp[Qt::Key_H] = LabelInfo(ui->label_rcp_motion_r_6,ADD,ARM,1,5);
    map_rcp[Qt::Key_J] = LabelInfo(ui->label_rcp_motion_r_7,ADD,ARM,1,6);
    map_rcp[Qt::Key_Z] = LabelInfo(ui->label_rcp_motion_r_1,MINUS,ARM,1,0);
    map_rcp[Qt::Key_X] = LabelInfo(ui->label_rcp_motion_r_2,MINUS,ARM,1,1);
    map_rcp[Qt::Key_C] = LabelInfo(ui->label_rcp_motion_r_3,MINUS,ARM,1,2);
    map_rcp[Qt::Key_V] = LabelInfo(ui->label_rcp_motion_r_4,MINUS,ARM,1,3);
    map_rcp[Qt::Key_B] = LabelInfo(ui->label_rcp_motion_r_5,MINUS,ARM,1,4);
    map_rcp[Qt::Key_N] = LabelInfo(ui->label_rcp_motion_r_6,MINUS,ARM,1,5);
    map_rcp[Qt::Key_M] = LabelInfo(ui->label_rcp_motion_r_7,MINUS,ARM,1,6);
}

void OpenLoongControler::keyPressEvent(QKeyEvent *event)
{
//    cout<<"keypress: "<<event->key()<<endl;
//    QMainWindow::keyPressEvent(event);
//    if (bool_manualEnabled) {
//        cout<<"keypress: "<<event->key()<<endl;
//        QString KJtext = QKeySequence(event->key() | event->modifiers()).toString();
//        std::cout << KJtext.toStdString() << std::endl;
//        keyPressedMap[event->key()] = true;
//        int modifier = event->modifiers() == Qt::ControlModifier ? 1 : 0;
//        //pressedKeys.insert(event->key());
//        handleKeyPresses(modifier);
//    }
    if(bool_manualEnabled){
        if(!event->isAutoRepeat())
            pressedKeys.append(event->key());
        if(!keyRespondTimer->isActive())
            keyRespondTimer->start(80);
    }
}

void OpenLoongControler::keyReleaseEvent(QKeyEvent *event)
{
//    cout<<"key release: "<<event->key()<<endl;
//    QMainWindow::keyReleaseEvent(event);
//    //pressedKeys.remove(event->key());
//    keyPressedMap[event->key()] = false;
    if(!event->isAutoRepeat())
        pressedKeys.removeAll(event->key());
    if(pressedKeys.isEmpty()){
        keyRespondTimer->stop();
        if (event->key() == Qt::Key_Left || event->key() == Qt::Key_Right) {
            robot_system->z_angle = 0.0;
//            qInfo() << "Stopped turning";
        }
        if (event->key() == Qt::Key_Up || event->key() == Qt::Key_Down) {
            robot_system->x_v = 0.0;
//            qInfo() << "Stopped moving forward/backward";
        }
    }
}

void OpenLoongControler::handleKeyPress(){
    if(robot_system->motion_mode==RobotSystem::MANUAL){
        for (int key : pressedKeys) {
            float currentValue = 0.0f;
            if(posCurr==POSITION){
                if(!pressedKeys.contains(Qt::Key_Control)&&!pressedKeys.contains(Qt::Key_Alt)){
                    if (map_pos.find(key) != map_pos.end()) {
                        currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[map_pos[key].row][map_pos[key].col];
                        cout << "Key: " << key << endl;
                        cout << "Value: " << currentValue << endl;
                        currentValue = map_pos[key].add_minus == ADD ? currentValue + manual_step : currentValue - manual_step;
                        //map_pos[key].label->setText(QString::number(currentValue));
                        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[map_pos[key].row][map_pos[key].col] = currentValue;
                    }
                }else if(pressedKeys.contains(Qt::Key_Control)){
                    if (map_pos_ctrl.find(key) != map_pos_ctrl.end()) {
                        if(map_pos_ctrl[key].joint_info==WAIST){
                            currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist[map_pos_ctrl[key].col];
                            cout << "Key: " << key << endl;
                            cout << "Value: " << currentValue << endl;
                            currentValue = map_pos_ctrl[key].add_minus == ADD ? currentValue + manual_step : currentValue - manual_step;
                            //map_pos_ctrl[key].label->setText(QString::number(currentValue));
                            robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_waist[map_pos_ctrl[key].col] = currentValue;
                        }else if(map_pos_ctrl[key].joint_info==HEAD){
                            currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head[map_pos_ctrl[key].col];
                            cout << "Key: " << key << endl;
                            cout << "Value: " << currentValue << endl;
                            currentValue = map_pos_ctrl[key].add_minus == ADD ? currentValue + manual_step : currentValue - manual_step;

                            //map_pos_ctrl[key].label->setText(QString::number(currentValue));
                            robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_head[map_pos_ctrl[key].col] = currentValue;
                        }
                    }
                }else if(pressedKeys.contains(Qt::Key_Alt)){
                    if (map_pos_alt.find(key) != map_pos_alt.end()) {
                        currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[map_pos_alt[key].row][map_pos_alt[key].col];
                        cout << "Key: " << key << endl;
                        cout << "Value: " << currentValue << endl;

                        currentValue = map_pos_alt[key].add_minus == ADD ? currentValue + manual_step/1000 : currentValue - manual_step/1000;
                        //currentValue = manual_step;
                        if(currentValue<0)currentValue=0;
                        qInfo()<<currentValue;
                        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[map_pos_alt[key].row][map_pos_alt[key].col] = currentValue;
                    }
                }
            }else if(posCurr==CURRENT){
                if(!pressedKeys.contains(Qt::Key_Control)&&!pressedKeys.contains(Qt::Key_Alt)){
                    if (map_curr.find(key) != map_curr.end()) {
                        currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[map_curr[key].row][map_curr[key].col];
                        cout << "Key: " << key << endl;
                        cout << "Value: " << currentValue << endl;
                        currentValue = map_curr[key].add_minus == ADD ? currentValue + manual_step : currentValue - manual_step;
                        //map_curr[key].label->setText(QString::number(currentValue));
                        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.tau_exp[map_curr[key].row][map_curr[key].col] = currentValue;
                    }
                }else if(pressedKeys.contains(Qt::Key_Control)){
                    if (map_curr_ctrl.find(key) != map_curr_ctrl.end()) {
                        if(map_curr_ctrl[key].joint_info==WAIST){
                            currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_waist[map_curr_ctrl[key].col];
                            cout << "Key: " << key << endl;
                            cout << "Value: " << currentValue << endl;
                            currentValue = map_curr_ctrl[key].add_minus == ADD ? currentValue + manual_step : currentValue - manual_step;
                            //map_curr_ctrl[key].label->setText(QString::number(currentValue));
                            robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.tau_exp_waist[map_curr_ctrl[key].col] = currentValue;
                        }else if(map_curr_ctrl[key].joint_info==HEAD){
                            currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_head[map_curr_ctrl[key].col];
                            cout << "Key: " << key << endl;
                            cout << "Value: " << currentValue << endl;
                            currentValue = map_curr_ctrl[key].add_minus == ADD ? currentValue + manual_step : currentValue - manual_step;
                            //map_curr_ctrl[key].label->setText(QString::number(currentValue));
                            robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.tau_exp_head[map_curr_ctrl[key].col] = currentValue;
                        }
                    }
                }else if(pressedKeys.contains(Qt::Key_Alt)){
                    if (map_curr_alt.find(key) != map_curr_alt.end()) {
                        currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[map_curr_alt[key].row][map_curr_alt[key].col];
                        cout << "Key: " << key << endl;
                        cout << "Value: " << currentValue << endl;
                        currentValue = map_curr_alt[key].add_minus == ADD ? currentValue + manual_step/1000 : currentValue - manual_step/1000;
                        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.tau_exp_hand[map_curr_alt[key].row][map_curr_alt[key].col] = currentValue;
                    }
                }
            }
        }
    }else if(robot_system->motion_mode==RobotSystem::MOTION_CAPTURE){
        for (int key : pressedKeys) {
            float currentValue = 0.0f;
            if(!pressedKeys.contains(Qt::Key_Alt)){
                if (map_rcp.find(key) != map_rcp.end()) {
                    currentValue = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[map_rcp[key].row][map_rcp[key].col];
                    cout << "Key: " << key << endl;
                    cout << "Value: " << currentValue << endl;
                    if(map_rcp[key].col<3||map_rcp[key].col>5){
                        currentValue = map_rcp[key].add_minus == ADD ? currentValue + rcp_step_rxyz*rcp_k : currentValue - rcp_step_rxyz*rcp_k;
                    }else if(map_rcp[key].col==3){
                        currentValue = map_rcp[key].add_minus == ADD ? currentValue + rcp_step_x*50*rcp_k : currentValue - rcp_step_x*50*rcp_k;
                    }else if(map_rcp[key].col==4){
                        currentValue = map_rcp[key].add_minus == ADD ? currentValue + rcp_step_y*50*rcp_k : currentValue - rcp_step_y*50*rcp_k;
                    }else if(map_rcp[key].col==5){
                        currentValue = map_rcp[key].add_minus == ADD ? currentValue + rcp_step_z*50*rcp_k : currentValue - rcp_step_z*50*rcp_k;
                    }
                    //map_pos[key].label->setText(QString::number(currentValue));
                    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[map_rcp[key].row][map_rcp[key].col] = currentValue;
                }
            }else{
                if(map_pos_alt.find(key) != map_pos_alt.end()){
                    currentValue = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[map_pos_alt[key].row][map_pos_alt[key].col];
                    cout << "Key: " << key << endl;
                    cout << "Value: " << currentValue << endl;

                    currentValue = map_pos_alt[key].add_minus == ADD ? currentValue + manual_step/1000 : currentValue - manual_step/1000;
                    //currentValue = manual_step;
                    if(currentValue<0)currentValue=0;
                    qInfo()<<currentValue;
                    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[map_pos_alt[key].row][map_pos_alt[key].col] = currentValue;
                }
            }
        }
    }

    if (pressedKeys.contains(Qt::Key_Left)){
        qInfo() << "Left Key: ";
        robot_system->z_angle = 0.1;
    }else if(pressedKeys.contains(Qt::Key_Right)){
        qInfo() << "Right Key: ";
        robot_system->z_angle = -0.1;
    }else {
        robot_system->z_angle = 0.0;  // 停止转向
    }
    if (pressedKeys.contains(Qt::Key_Up)){
        qInfo() << "Up Key: ";
        robot_system->x_v = 0.1;
    }else if(pressedKeys.contains(Qt::Key_Down)){
        qInfo() << "Down Key: ";
        robot_system->x_v = -0.1;
    }else if(!pressedKeys.contains(Qt::Key_Up) && !pressedKeys.contains(Qt::Key_Down)) {
        robot_system->x_v = 0.0;  // 停止转向
    }
}

void OpenLoongControler::ShowInfomation(){
    //dataMutex.lock();
    ui->label_feedback_l_A_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][0]));
    ui->label_feedback_l_A_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][1]));
    ui->label_feedback_l_A_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][2]));
    ui->label_feedback_l_A_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][3]));
    ui->label_feedback_l_A_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][4]));
    ui->label_feedback_l_A_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][5]));
    ui->label_feedback_l_A_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][6]));
    ui->label_feedback_r_A_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[0][0]));
    ui->label_feedback_r_A_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[0][1]));
    ui->label_feedback_r_A_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[0][2]));
    ui->label_feedback_r_A_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[0][3]));
    ui->label_feedback_r_A_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[0][4]));
    ui->label_feedback_r_A_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[0][5]));
    ui->label_feedback_r_A_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[0][6]));

    ui->label_feedback_l_L_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][0]));
    ui->label_feedback_l_L_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][1]));
    ui->label_feedback_l_L_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][2]));
    ui->label_feedback_l_L_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][3]));
    ui->label_feedback_l_L_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][4]));
    ui->label_feedback_l_L_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][5]));
    ui->label_feedback_l_L_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][6]));
    ui->label_feedback_r_L_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[1][0]));
    ui->label_feedback_r_L_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[1][1]));
    ui->label_feedback_r_L_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[1][2]));
    ui->label_feedback_r_L_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[1][3]));
    ui->label_feedback_r_L_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[1][4]));
    ui->label_feedback_r_L_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[1][5]));
    ui->label_feedback_r_L_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm_exp[1][6]));

    ui->label_feedback_waist1_pos->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist[0]));
    ui->label_feedback_waist2_pos->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist[1]));
    ui->label_feedback_waist3_pos->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist[2]));
    ui->label_feedback_head1_pos->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head[0]));
    ui->label_feedback_head2_pos->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head[1]));


    ui->label_feedback_velo_l_A_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[0][0]));
    ui->label_feedback_velo_l_A_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[0][1]));
    ui->label_feedback_velo_l_A_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[0][2]));
    ui->label_feedback_velo_l_A_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[0][3]));
    ui->label_feedback_velo_l_A_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[0][4]));
    ui->label_feedback_velo_l_A_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[0][5]));
    ui->label_feedback_velo_l_A_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[0][6]));
    ui->label_feedback_velo_r_A_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[0][0]));
    ui->label_feedback_velo_r_A_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[0][1]));
    ui->label_feedback_velo_r_A_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[0][2]));
    ui->label_feedback_velo_r_A_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[0][3]));
    ui->label_feedback_velo_r_A_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[0][4]));
    ui->label_feedback_velo_r_A_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[0][5]));
    ui->label_feedback_velo_r_A_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[0][6]));

    ui->label_feedback_velo_l_L_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[1][0]));
    ui->label_feedback_velo_l_L_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[1][1]));
    ui->label_feedback_velo_l_L_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[1][2]));
    ui->label_feedback_velo_l_L_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[1][3]));
    ui->label_feedback_velo_l_L_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[1][4]));
    ui->label_feedback_velo_l_L_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[1][5]));
    ui->label_feedback_velo_l_L_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm[1][6]));
    ui->label_feedback_velo_r_L_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[1][0]));
    ui->label_feedback_velo_r_L_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[1][1]));
    ui->label_feedback_velo_r_L_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[1][2]));
    ui->label_feedback_velo_r_L_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[1][3]));
    ui->label_feedback_velo_r_L_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[1][4]));
    ui->label_feedback_velo_r_L_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[1][5]));
    ui->label_feedback_velo_r_L_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_arm_exp[1][6]));

    ui->label_feedback_waist1_velo->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_waist[0]));
    ui->label_feedback_waist2_velo->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_waist[1]));
    ui->label_feedback_waist3_velo->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_waist[2]));
    ui->label_feedback_head1_velo->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_head[0]));
    ui->label_feedback_head2_velo->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_qd_head[1]));

    ui->label_feedback_cur_l_A_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[0][0]));
    ui->label_feedback_cur_l_A_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[0][1]));
    ui->label_feedback_cur_l_A_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[0][2]));
    ui->label_feedback_cur_l_A_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[0][3]));
    ui->label_feedback_cur_l_A_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[0][4]));
    ui->label_feedback_cur_l_A_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[0][5]));
    ui->label_feedback_cur_l_A_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[0][6]));
    ui->label_feedback_cur_r_A_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[0][0]));
    ui->label_feedback_cur_r_A_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[0][1]));
    ui->label_feedback_cur_r_A_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[0][2]));
    ui->label_feedback_cur_r_A_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[0][3]));
    ui->label_feedback_cur_r_A_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[0][4]));
    ui->label_feedback_cur_r_A_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[0][5]));
    ui->label_feedback_cur_r_A_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[0][6]));

    ui->label_feedback_cur_l_L_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[1][0]));
    ui->label_feedback_cur_l_L_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[1][1]));
    ui->label_feedback_cur_l_L_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[1][2]));
    ui->label_feedback_cur_l_L_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[1][3]));
    ui->label_feedback_cur_l_L_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[1][4]));
    ui->label_feedback_cur_l_L_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[1][5]));
    ui->label_feedback_cur_l_L_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_arm[1][6]));
    ui->label_feedback_cur_r_L_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[1][0]));
    ui->label_feedback_cur_r_L_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[1][1]));
    ui->label_feedback_cur_r_L_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[1][2]));
    ui->label_feedback_cur_r_L_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[1][3]));
    ui->label_feedback_cur_r_L_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[1][4]));
    ui->label_feedback_cur_r_L_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[1][5]));
    ui->label_feedback_cur_r_L_7->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_arm[1][6]));

    ui->label_feedback_waist1_curr->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_waist[0]));
    ui->label_feedback_waist2_curr->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_waist[1]));
    ui->label_feedback_waist3_curr->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_waist[2]));
    ui->label_feedback_head1_curr->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_head[0]));
    ui->label_feedback_head2_curr->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_head[1]));

    ui->label_feedback_waist1_curr_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_waist[0]));
    ui->label_feedback_waist2_curr_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_waist[1]));
    ui->label_feedback_waist3_curr_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_waist[2]));
    ui->label_feedback_head1_curr_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_head[0]));
    ui->label_feedback_head2_curr_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.tau_exp_head[1]));
    //rcp
    ui->label_rcp_motion_l_1->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_rz));
    ui->label_rcp_motion_l_2->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_ry));
    ui->label_rcp_motion_l_3->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_rx));
    ui->label_rcp_motion_l_4->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_px));
    ui->label_rcp_motion_l_5->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_py));
    ui->label_rcp_motion_l_6->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_pz));
    ui->label_rcp_motion_l_7->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_belt));

    ui->label_rcp_motion_r_1->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.right_arm_rz));
    ui->label_rcp_motion_r_2->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.right_arm_ry));
    ui->label_rcp_motion_r_3->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.right_arm_rx));
    ui->label_rcp_motion_r_4->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.right_arm_px));
    ui->label_rcp_motion_r_5->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.right_arm_py));
    ui->label_rcp_motion_r_6->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.right_arm_pz));
    ui->label_rcp_motion_r_7->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.right_arm_belt));

    //hand
    ui->label_feedback_l_H_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][0]));
    ui->label_feedback_l_H_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][1]));
    ui->label_feedback_l_H_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][2]));
    ui->label_feedback_l_H_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][3]));
    ui->label_feedback_l_H_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][4]));
    ui->label_feedback_l_H_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][5]));
    ui->label_feedback_r_H_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][0]));
    ui->label_feedback_r_H_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][1]));
    ui->label_feedback_r_H_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][2]));
    ui->label_feedback_r_H_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][3]));
    ui->label_feedback_r_H_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][4]));
    ui->label_feedback_r_H_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][5]));

    ui->label_feedback_curr_l_H_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[0][0]));
    ui->label_feedback_curr_l_H_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[0][1]));
    ui->label_feedback_curr_l_H_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[0][2]));
    ui->label_feedback_curr_l_H_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[0][3]));
    ui->label_feedback_curr_l_H_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[0][4]));
    ui->label_feedback_curr_l_H_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[0][5]));
    ui->label_feedback_curr_r_H_1->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[1][0]));
    ui->label_feedback_curr_r_H_2->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[1][1]));
    ui->label_feedback_curr_r_H_3->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[1][2]));
    ui->label_feedback_curr_r_H_4->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[1][3]));
    ui->label_feedback_curr_r_H_5->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[1][4]));
    ui->label_feedback_curr_r_H_6->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_tau_hand[1][5]));

    ui->label_ts_hand_motion_l_1->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[0][0]));
    ui->label_ts_hand_motion_l_2->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[0][1]));
    ui->label_ts_hand_motion_l_3->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[0][2]));
    ui->label_ts_hand_motion_l_4->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[0][3]));
    ui->label_ts_hand_motion_l_5->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[0][4]));
    ui->label_ts_hand_motion_l_6->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[0][5]));
    ui->label_ts_hand_motion_r_1->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[1][0]));
    ui->label_ts_hand_motion_r_2->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[1][1]));
    ui->label_ts_hand_motion_r_3->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[1][2]));
    ui->label_ts_hand_motion_r_4->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[1][3]));
    ui->label_ts_hand_motion_r_5->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[1][4]));
    ui->label_ts_hand_motion_r_6->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.hand_data[1][5]));

    ui->label_other_motion_1->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.vcap_data[0]));
    ui->label_other_motion_2->setText(QString::number(
         robot_system->robot_data->robot_info_.motion_data_recieve_.vcap_data[1]));

    ShowModeInfo();
}

void OpenLoongControler::ShowModeInfo(){
    switch (robot_system->robot_data->robot_info_.robot_state_.motion_mode) {
        case RobotSystem::IDLE:
             ui->label_motion_mode->setText("IDLE"); break;
        case RobotSystem::DYNAMIC:
            ui->label_motion_mode->setText("DYNAMIC"); break;
        case RobotSystem::MANUAL:
            ui->label_motion_mode->setText("MANUAL"); break;
        case RobotSystem::AUTO:
            ui->label_motion_mode->setText("AUTO"); break;
        case RobotSystem::DEMONSTRATOR:
            ui->label_motion_mode->setText("DEMONSTRATOR"); break;
        case RobotSystem::MOTION_CAPTURE:
            ui->label_motion_mode->setText("MOTION_CAPTURE"); break;
        default:
            break;
    }
    switch (robot_system->robot_data->robot_info_.robot_state_.running_mode) {

        case RobotSystem::INIT:
            ui->label_running_mode->setText("INIT"); break;
        case RobotSystem::INIT_OK:
            ui->label_running_mode->setText("INIT_OK"); break;
        case RobotSystem::READY:
            ui->label_running_mode->setText("READY"); break;
        case RobotSystem::READY_OK:
            ui->label_running_mode->setText("READY_OK"); break;
        case RobotSystem::RUN:
            ui->label_running_mode->setText("RUNNING..."); break;
        case RobotSystem::STOP:
            ui->label_running_mode->setText("STOP"); break;
        case RobotSystem::DISABLE:
            ui->label_running_mode->setText("DISABLE"); break;
        case RobotSystem::ERROR:
            ui->label_running_mode->setText("ERROR"); break;
        default:
            break;
    }
}

void OpenLoongControler::on_radioButton_manual_pos_clicked()
{
    posCurr = POSITION;
    robot_system->robot_data->robot_info_.robot_cmd_send_.joint_mode = 0;
}

void OpenLoongControler::on_radioButton_manual_curr_clicked()
{
    posCurr = CURRENT;
    robot_system->robot_data->robot_info_.robot_cmd_send_.joint_mode = 2;
}

void OpenLoongControler::on_pushButton_manual_step_set_clicked()
{
    manual_step = ui->lineEdit_manual_step->text().toFloat();
}

//demonstrator
void OpenLoongControler::handleDemonstrator(){
    joint_larm_0 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][0];
    joint_larm_1 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][1];
    joint_larm_2 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][2];
    joint_larm_3 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][3];
    joint_larm_4 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][4];
    joint_larm_5 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][5];
    joint_larm_6 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][6];
    joint_rarm_0 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][0];
    joint_rarm_1 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][1];
    joint_rarm_2 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][2];
    joint_rarm_3 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][3];
    joint_rarm_4 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][4];
    joint_rarm_5 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][5];
    joint_rarm_6 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][6];
    joint_waist_0 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist[0];
    joint_waist_1 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist[1];
    joint_waist_2 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist[2];
    joint_head_0 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head[0];
    joint_head_1 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head[1];

    joint_lhand_0 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][0];
    joint_lhand_1 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][1];
    joint_lhand_2 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][2];
    joint_lhand_3 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][3];
    joint_lhand_4 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][4];
    joint_lhand_5 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[0][5];
    joint_rhand_0 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][0];
    joint_rhand_1 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][1];
    joint_rhand_2 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][2];
    joint_rhand_3 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][3];
    joint_rhand_4 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][4];
    joint_rhand_5 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_hand[1][5];

    vcap_hand_0 = robot_system->robot_data->robot_info_.motion_data_recieve_.vcap_data[0];
    //vcap_hand_1 = robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.vcap_hand[1];

    //std::cout<<joint_larm_0<<" read value "<<QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString()<<std::endl;
}

void OpenLoongControler::on_pushButton_demon_continue_start_clicked()
{
    demonstrator_timer->start(10);
    thread_demonstrator.startThread();
    emit startRecording(0);
}

void OpenLoongControler::on_pushButton_demon_continue_stop_clicked()
{
    emit stopRecording();
    demonstrator_timer->stop();
}

void OpenLoongControler::on_pushButton_demons_continue_clear_clicked()
{
    emit clearRecording();
}

void OpenLoongControler::on_pushButton_demon_continue_save_clicked()
{
    emit saveRecording(ui->lineEdit_demons_file_savename->text());
}

void OpenLoongControler::on_pushButton_demon_continue_load_clicked()
{
    QString fileName = ui->lineEdit_demons_file_loadname->text();
    demonstrate_data.clear();
    //binary
//    std::ifstream file(fileName.toStdString(), std::ios::binary);
//    if (file.is_open()) {
//        file.seekg(0, std::ios::end);
//        int fileSize = file.tellg();
//        file.seekg(0, std::ios::beg);
//        int numFloats = fileSize / sizeof(float);
//        std::vector<float> data(numFloats);
//        file.read(reinterpret_cast<char*>(&data[0]), fileSize);
//        file.close();

//        // print
//        for (int i = 0; i < numFloats; ++i) {
//            std::cout << data[i] << " ";
//            if ((i + 1) % 19 == 0) {
//                std::cout << std::endl;
//            }
//        }
//        std::cout << std::endl;
//    } else {
//        std::cerr << "Failed to open file for reading." << std::endl;
//    }
    //csv
    std::ifstream file(fileName.toStdString());
    if (file.is_open()) {
        //std::vector<std::vector<float>> data;
        std::string line;
        while (std::getline(file, line)) {
            std::vector<float> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                double value_double = std::stod(cell);
                float value = static_cast<float>(value_double);
                row.push_back(value);
            }
            demonstrate_data.push_back(row);
        }
        file.close();

        // print
        for (const auto& row : demonstrate_data) {
            for (float value : row) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Failed to open file for reading." << std::endl;
    }
}

void OpenLoongControler::on_pushButton_demon_continue_load2_clicked()
{
    QString fileName = ui->lineEdit_demons_file_loadname2->text();
    demonstrate_data.clear();
    std::ifstream file(fileName.toStdString());
    if (file.is_open()) {
        //std::vector<std::vector<float>> data;
        std::string line;
        while (std::getline(file, line)) {
            std::vector<float> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                double value_double = std::stod(cell);
                float value = static_cast<float>(value_double);
                row.push_back(value);
            }
            demonstrate_data.push_back(row);
        }
        file.close();

        // print
        for (const auto& row : demonstrate_data) {
            for (float value : row) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Failed to open file for reading." << std::endl;
    }
}


void OpenLoongControler::on_radioButton_demonstrator_loop_clicked()
{
    bool_demonstrator_loop = true;
}

void OpenLoongControler::on_radioButton_demonstrator_notloop_clicked(){
    bool_demonstrator_loop = false;
}

void OpenLoongControler::on_pushButton_demon_continue_replay_clicked()
{
    if(robot_system->motion_mode!=RobotSystem::MANUAL){
        robot_system->motion_mode = RobotSystem::MANUAL;
    }
    if (demonstrate_data.empty()) {
        qWarning() << "Demonstrate data is empty";
        return;
    }
    qInfo()<<demonstrate_data.size()<<" "<<demonstrate_data[0].size();
    curr_demonstrate_index=0;
    accumulated_time=0.0;
    demonstrator_run_timer->start(10);
}

void OpenLoongControler::handleDemonstrator_run(){
    if(curr_demonstrate_index+3 < demonstrate_data.size()){
        accumulated_time += playback_speed;
        while (accumulated_time >= 1.0) {
            accumulated_time -= 1.0;
            curr_demonstrate_index += 1;
        }
        qInfo()<<curr_demonstrate_index<<" "<<accumulated_time<<" "<<playback_speed;
        std::vector<float> interpolated_data = interpolate(demonstrate_data[curr_demonstrate_index], demonstrate_data[curr_demonstrate_index + 1], accumulated_time);
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][0]=interpolated_data[0];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][1]=interpolated_data[1];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][2]=interpolated_data[2];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][3]=interpolated_data[3];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][4]=interpolated_data[4];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][5]=interpolated_data[5];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][6]=interpolated_data[6];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][0]=interpolated_data[7];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][1]=interpolated_data[8];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][2]=interpolated_data[9];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][3]=interpolated_data[10];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][4]=interpolated_data[11];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][5]=interpolated_data[12];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][6]=interpolated_data[13];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[0][0]=interpolated_data[14];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[0][1]=interpolated_data[15];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[0][2]=interpolated_data[16];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[0][3]=interpolated_data[17];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[0][4]=interpolated_data[18];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[0][5]=interpolated_data[19];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[1][0]=interpolated_data[20];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[1][1]=interpolated_data[21];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[1][2]=interpolated_data[22];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[1][3]=interpolated_data[23];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[1][4]=interpolated_data[24];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_exp_hand[1][5]=interpolated_data[25];
        robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_vcap_hand[0] = interpolated_data[26];
        //robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_vcap_hand[1] = demonstrate_data[curr_demonstrate_index][27];
    }else{
        demonstrator_run_timer->stop();
        qInfo()<<"Demonstrate end";
        if(bool_demonstrator_loop){
            qInfo()<<demonstrate_data.size()<<" "<<demonstrate_data[0].size()<<" loop";
            curr_demonstrate_index=0;
            accumulated_time=0.0;
            demonstrator_run_timer->start(10);
        }
    }
}

std::vector<float> OpenLoongControler::interpolate(const std::vector<float>& start, const std::vector<float>& end, double t)
{
    std::vector<float> result(start.size());
    for (unsigned int i = 0; i < start.size(); ++i)
    {
        result[i] = start[i] + t * (end[i] - start[i]);
    }
    return result;
}

void OpenLoongControler::on_horizontalSlider_valueChanged(int value)
{
    playback_speed = value/10.0;
    ui->lineEdit_demons_speed->setText(QString::number(playback_speed));
}

void OpenLoongControler::on_pushButton_demon_continue_replay_stop_clicked()
{
    demonstrator_run_timer->stop();
}

void OpenLoongControler::on_pushButton_demon_continue_replay_continue_clicked()
{
    demonstrator_run_timer->start(10);
}

void OpenLoongControler::on_pushButton_demon_continue_replay_end_clicked()
{
    demonstrator_run_timer->stop();
    curr_demonstrate_index=demonstrate_data.size();
}

//RCP
void OpenLoongControler::handleRCP(){
    rcp_l_0 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][0];
    rcp_l_1 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][1];
    rcp_l_2 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][2];
    rcp_l_3 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][3];
    rcp_l_4 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][4];
    rcp_l_5 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][5];
    rcp_l_6 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][6];
    rcp_r_0 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][0];
    rcp_r_1 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][1];
    rcp_r_2 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][2];
    rcp_r_3 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][3];
    rcp_r_4 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][4];
    rcp_r_5 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][5];
    rcp_r_6 = robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][6];
}

void OpenLoongControler::on_pushButton_rcp_record_start_clicked()
{
    rcp_timer->start(10);
    thread_demonstrator.startThread();
    emit startRecording(1);
}

void OpenLoongControler::on_pushButton_rcp_record_stop_clicked()
{
    emit stopRecording();
    rcp_timer->stop();
}

void OpenLoongControler::on_pushButton_rcp_record_save_clicked()
{
    emit saveRecording(ui->lineEdit_rcp_record_file->text());
}

void OpenLoongControler::on_pushButton_rcp_record_load_clicked()
{
    QString fileName = ui->lineEdit_rcp_load_file->text();
    demonstrate_data.clear();
    std::ifstream file(fileName.toStdString());
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::vector<float> row;
            std::vector<float> temp;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                double value_double = std::stod(cell);
                float value = static_cast<float>(value_double);
                temp.push_back(value);
            }
            if(temp.size()>25){
                int i=0;
                double dv3_l[7],dv3_r[7];
                double weizi_l[7],weizi_r[7];
                dv3_l[0]=temp[0]/57300;
                dv3_l[1]=temp[1]/57300;
                dv3_l[2]=temp[2]/57300;
                dv3_l[3]=temp[3]/57300;
                dv3_l[4]=temp[4]/57300;
                dv3_l[5]=temp[5]/57300;
                dv3_l[6]=temp[6]/57300;
                dv3_r[0]=temp[7]/57300;
                dv3_r[1]=temp[8]/57300;
                dv3_r[2]=temp[9]/57300;
                dv3_r[3]=temp[10]/57300;
                dv3_r[4]=temp[11]/57300;
                dv3_r[5]=temp[12]/57300;
                dv3_r[6]=temp[13]/57300;
                k_OLR2(dv3_l, weizi_l);
                k_OLR2(dv3_r, weizi_r);
                for(i=0;i<7;++i){
                    row.push_back(weizi_l[i]);
                }
                for(i=0;i<7;++i){
                    row.push_back(weizi_r[i]);
                }
                for(i=14;i<26;++i){
                    row.push_back(temp[i]);
                }
            }
            demonstrate_data.push_back(row);
        }
        file.close();

        // print
        for (const auto& row : demonstrate_data) {
            for (float value : row) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cerr << "Failed to open file for reading." << std::endl;
    }
}

void OpenLoongControler::on_pushButton_rcp_record_modify_clicked_old()
{
    float x=-577.228; float y=-32.4491; float z=104.332;
    float rx = 0; float ry = 0; float rz = 0;
    delta_r_x=0; delta_r_y=0; delta_r_z=0;
    delta_r_rx =0; delta_r_ry=0; delta_r_rz=0;
    int avg=10;
    for(int j=0;j<avg;j++){
        delta_r_x += robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_px-x;
        delta_r_y += robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_py-y;
        delta_r_z += robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_pz-z;
        delta_r_rx += robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_rx-rx;
        delta_r_ry += robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_ry-ry;
        delta_r_rz += robot_system->robot_data->robot_info_.motion_data_recieve_.left_arm_rz-rz;
        QThread::msleep(200);
    }
    delta_r_x /= avg;    delta_r_y /= avg;    delta_r_z /= avg;
    delta_r_rx /= avg;    delta_r_ry /= avg;    delta_r_rz /= avg;
    float tf = demonstrate_data.size()/100.0;
    unsigned long i;
    stamp = 0.25;
    for(i=0;i<stamp*demonstrate_data.size();++i){
        float theta_r_x = 10.0*delta_r_x/std::pow(tf,3)*pow((i/(stamp*100.0)),3)-15.0*delta_r_x/std::pow(tf,4)*pow((i/25.0),4)+6.0*delta_r_x/std::pow(tf,5)*pow((i/(stamp*100.0)),5);
        float theta_r_y = 10.0*delta_r_y/std::pow(tf,3)*pow((i/(stamp*100.0)),3)-15.0*delta_r_y/std::pow(tf,4)*pow((i/25.0),4)+6.0*delta_r_y/std::pow(tf,5)*pow((i/(stamp*100.0)),5);
        float theta_r_z = 10.0*delta_r_z/std::pow(tf,3)*pow((i/(stamp*100.0)),3)-15.0*delta_r_z/std::pow(tf,4)*pow((i/25.0),4)+6.0*delta_r_z/std::pow(tf,5)*pow((i/(stamp*100.0)),5);
        demonstrate_data[i][3] += theta_r_x;
        demonstrate_data[i][4] += theta_r_y;
        demonstrate_data[i][5] += theta_r_z;
        if(std::sqrt(demonstrate_data[i][3]*demonstrate_data[i][3]+demonstrate_data[i][4]*demonstrate_data[i][4]+demonstrate_data[i][5]*demonstrate_data[i][5])>=590){
            qDebug()<<"Out of Range!";
            break;
        }
        qInfo()<<demonstrate_data[i][3]<<" "<<demonstrate_data[i][4]<<" "<<demonstrate_data[i][5]<<" "<<theta_r_x<<" "<<theta_r_y<<" "<<theta_r_z;
        //qInfo()<<i<<" "<<theta_r_x<<" "<<theta_r_y<<" "<<theta_r_z;
    }
    while(i<demonstrate_data.size()){
        demonstrate_data[i][3] += delta_r_x;
        demonstrate_data[i][4] += delta_r_y;
        demonstrate_data[i][5] += delta_r_z;
        if(std::sqrt(demonstrate_data[i][3]*demonstrate_data[i][3]+demonstrate_data[i][4]*demonstrate_data[i][4]+demonstrate_data[i][5]*demonstrate_data[i][5])>=590){
            qDebug()<<"Out of Range!";
            break;
        }
        //qInfo()<<i<<" ";
        qInfo()<<demonstrate_data[i][3]<<" "<<demonstrate_data[i][4]<<" "<<demonstrate_data[i][5]<<" "<<delta_r_x<<" "<<delta_r_y<<" "<<delta_r_z;
        ++i;
    }
    qInfo()<<delta_r_x<<" "<<delta_r_y<<" "<<delta_r_z<<" "<<delta_r_rz<<" "<<delta_r_ry<<" "<<delta_r_rx;
}

void OpenLoongControler::on_pushButton_rcp_record_modify_clicked()
{
    Eigen::Matrix4f matrix_stcnew_left, matrix_stcnew_right, matrix_stc_inv_left, matrix_stc_inv_right;
    std::vector<Eigen::Matrix4f> transformation_matrices_left, transformation_matrices_right;

    std::cout << "shoulder to new camera" << std::endl;
    // receive matrix_stcnew from UDP port 8014
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            matrix_stcnew_left(i, j) = robot_system->robot_data->robot_info_.remote_computer_data_.camera_to_shoulder_left.matrix[i][j];
            matrix_stcnew_right(i, j) = robot_system->robot_data->robot_info_.remote_computer_data_.camera_to_shoulder_right.matrix[i][j];
        }
    }
    std::cout << "left stcnew matrix:" << std::endl;
    std::cout << matrix_stcnew_left << std::endl;
    std::cout << "right stcnew matrix:" << std::endl;
    std::cout << matrix_stcnew_right << std::endl;

//    matrix_cts<<-0.0222983,0.0257587,0.999419,-551.112,
//    0.671906,-0.739852,0.0340598,576.564,
//    0.7403,0.672276,-0.000809989,-92.8788,
//    0,0,0,1;
//    matrix_stcnew<<-0.0237363,0.0233373,0.999446,-551.326,
//    0.893276,-0.448391,0.0316848,548.753,
//    0.448882,0.893533,-0.0102035,-80.6764,
//    0,0 ,0 ,1;


    // Inverse matrix
    matrix_stc_inv_left = matrix_stc_left.inverse();
    matrix_stc_inv_right = matrix_stc_right.inverse();

    for (const auto& row : demonstrate_data) {
        float rz_left = row[0]; float ry_left = row[1]; float rx_left = row[2];
        float tx_left = row[3]; float ty_left = row[4]; float tz_left = row[5];
        float rz_right = row[7]; float ry_right = row[8]; float rx_right = row[9];
        float tx_right = row[10]; float ty_right = row[11]; float tz_right = row[12];
        //std::cout << tx << " " << ty << " " << tz << std::endl;
        Eigen::Matrix4f matrix_left = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f matrix_right = Eigen::Matrix4f::Identity();
        matrix_left(0, 3) = tx_left;        matrix_left(1, 3) = ty_left;        matrix_left(2, 3) = tz_left;
        matrix_right(0, 3) = tx_right;      matrix_right(1, 3) = ty_right;      matrix_right(2, 3) = tz_right;
        Eigen::Matrix3f rZ_left = Eigen::AngleAxisf(rz_left, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        Eigen::Matrix3f rY_left = Eigen::AngleAxisf(ry_left, Eigen::Vector3f::UnitY()).toRotationMatrix();
        Eigen::Matrix3f rX_left = Eigen::AngleAxisf(rx_left, Eigen::Vector3f::UnitX()).toRotationMatrix();
        Eigen::Matrix3f rZ_right = Eigen::AngleAxisf(rz_right, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        Eigen::Matrix3f rY_right = Eigen::AngleAxisf(ry_right, Eigen::Vector3f::UnitY()).toRotationMatrix();
        Eigen::Matrix3f rX_right = Eigen::AngleAxisf(rx_right, Eigen::Vector3f::UnitX()).toRotationMatrix();

        // 总的旋转矩阵，定系旋转顺序为 Rz * Ry * Rx
        Eigen::Matrix3f rotation_matrix_left = rX_left * rY_left * rZ_left;
        Eigen::Matrix3f rotation_matrix_right = rX_right * rY_right * rZ_right;
        matrix_left.block<3, 3>(0, 0) = rotation_matrix_left;  // 将 3x3 旋转矩阵插入到 4x4 矩阵中
        matrix_right.block<3, 3>(0, 0) = rotation_matrix_right;
//        std::cout << "matrix:" << std::endl;
//        std::cout << matrix << std::endl;

        matrix_left = matrix_stcnew_left * matrix_stc_inv_left * matrix_left;
        matrix_right = matrix_stcnew_right * matrix_stc_inv_right * matrix_right;

//        std::cout << "Transform matrix: " << std::endl;
//        std::cout << matrix << std::endl;

        transformation_matrices_left.push_back(matrix_left);
        transformation_matrices_right.push_back(matrix_right);
    }

    //transformation matrix to euler angle
    for (unsigned long i=0;i<transformation_matrices_left.size();++i) {
        //left
        float y_beta_left = atan2(transformation_matrices_left[i](0,2),std::sqrt(transformation_matrices_left[i](0,0)*transformation_matrices_left[i](0,0)+transformation_matrices_left[i](0,1)*transformation_matrices_left[i](0,1)));
        float x_gamma_left,z_alpha_left;
        if(std::abs(y_beta_left-M_PI/2.0)<1e-6){
            x_gamma_left=0; z_alpha_left=atan2(transformation_matrices_left[i](1,0),transformation_matrices_left[i](1,1));
        }else if(std::abs(y_beta_left-(-M_PI/2.0))<1e-6){
            x_gamma_left=0;z_alpha_left=atan2(transformation_matrices_left[i](1,0),transformation_matrices_left[i](1,1));
        }else{
            z_alpha_left = atan2(-transformation_matrices_left[i](0,1)/std::cos(y_beta_left),transformation_matrices_left[i](0,0)/std::cos(y_beta_left));
            x_gamma_left = atan2(-transformation_matrices_left[i](1,2)/std::cos(y_beta_left),transformation_matrices_left[i](2,2)/std::cos(y_beta_left));
        }
        float x_left = transformation_matrices_left[i](0,3); float y_left=transformation_matrices_left[i](1,3); float z_left=transformation_matrices_left[i](2,3);
        float length_left = std::sqrt(x_left*x_left+y_left*y_left+z_left*z_left);
        //right
        float y_beta_right = atan2(transformation_matrices_right[i](0,2),std::sqrt(transformation_matrices_right[i](0,0)*transformation_matrices_right[i](0,0)+transformation_matrices_right[i](0,1)*transformation_matrices_right[i](0,1)));
        float x_gamma_right,z_alpha_right;
        if(std::abs(y_beta_right-M_PI/2.0)<1e-6){
            x_gamma_right=0; z_alpha_right=atan2(transformation_matrices_right[i](1,0),transformation_matrices_right[i](1,1));
        }else if(std::abs(y_beta_right-(-M_PI/2.0))<1e-6){
            x_gamma_right=0;z_alpha_right=atan2(transformation_matrices_right[i](1,0),transformation_matrices_right[i](1,1));
        }else{
            z_alpha_right = atan2(-transformation_matrices_right[i](0,1)/std::cos(y_beta_right),transformation_matrices_right[i](0,0)/std::cos(y_beta_right));
            x_gamma_right = atan2(-transformation_matrices_right[i](1,2)/std::cos(y_beta_right),transformation_matrices_right[i](2,2)/std::cos(y_beta_right));
        }
        float x_right = transformation_matrices_right[i](0,3); float y_right=transformation_matrices_right[i](1,3); float z_right=transformation_matrices_right[i](2,3);
        float length_right = std::sqrt(x_right*x_right+y_right*y_right+z_right*z_right);

        if(length_left>=595||length_right>=595){
            qInfo()<<"Out of Range with x = "<<x_left<<" y = "<<y_left<<" z = "<<z_left<<" index = "<<i<<" length = "<<length_left;
            qInfo()<<"Out of Range with x = "<<x_right<<" y = "<<y_right<<" z = "<<z_right<<" index = "<<i<<" length = "<<length_right;
            break;
        }else{
            demonstrate_data[i][0] = z_alpha_left;            demonstrate_data[i][1] = y_beta_left;            demonstrate_data[i][2] = x_gamma_left;
            demonstrate_data[i][3] = x_left;                  demonstrate_data[i][4] = y_left;                 demonstrate_data[i][5] = z_left;
            demonstrate_data[i][7] = z_alpha_right;           demonstrate_data[i][8] = y_beta_right;           demonstrate_data[i][9] = x_gamma_right;
            demonstrate_data[i][10] = x_right;                demonstrate_data[i][11] = y_right;               demonstrate_data[i][12] = z_right;
            std::cout << "left i: "<< 0 <<", rz: " << z_alpha_left << ", ry: " << y_beta_left << ", rx: " << x_gamma_left << ", tx: " << x_left << ", ty: " << y_left << ", tz: " << z_left << std::endl;
            std::cout << "right i: "<< 0 <<", rz: " << z_alpha_right << ", ry: " << y_beta_right << ", rx: " << x_gamma_right << ", tx: " << x_right << ", ty: " << y_right << ", tz: " << z_right << std::endl;
        }
    }
}

void OpenLoongControler::on_pushButton_rcp_record_calibrate_clicked()
{
    std::cout<<"camera to shoulder "<<std::endl;
    for (int i=0;i<4;++i) {
        for (int j=0;j<4;++j) {
            matrix_stc_left(i,j) = robot_system->robot_data->robot_info_.remote_computer_data_.camera_to_shoulder_left.matrix[i][j];
            matrix_stc_right(i,j) = robot_system->robot_data->robot_info_.remote_computer_data_.camera_to_shoulder_right.matrix[i][j];
        }
    }
    std::cout << "left stcnew matrix:" << std::endl;
    std::cout << matrix_stc_left << std::endl;
    std::cout << "right stcnew matrix:" << std::endl;
    std::cout << matrix_stc_right << std::endl;

    double dv3_l[7],dv3_r[7];
    double weizi_l[7],weizi_r[7];
    float k = 180.0*1000/M_PI;
    dv3_l[0]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][0]/k;
    dv3_l[1]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][1]/k;
    dv3_l[2]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][2]/k;
    dv3_l[3]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][3]/k;
    dv3_l[4]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][4]/k;
    dv3_l[5]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][5]/k;
    dv3_l[6]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][6]/k;
    dv3_r[0]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][0]/k;
    dv3_r[1]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][1]/k;
    dv3_r[2]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][2]/k;
    dv3_r[3]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][3]/k;
    dv3_r[4]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][4]/k;
    dv3_r[5]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][5]/k;
    dv3_r[6]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][6]/k;

    k_OLR2(dv3_l, weizi_l);
    k_OLR2(dv3_r, weizi_r);

    std::cout<<weizi_l[0]<<" "<<weizi_l[1]<<" "<<weizi_l[2]<<" "<<weizi_l[3]<<" "<<weizi_l[4]<<" "<<weizi_l[5]<<" "<<weizi_l[6]<<std::endl;
    std::cout<<weizi_r[0]<<" "<<weizi_r[1]<<" "<<weizi_r[2]<<" "<<weizi_r[3]<<" "<<weizi_r[4]<<" "<<weizi_r[5]<<" "<<weizi_r[6]<<std::endl;

    float rz_left = weizi_l[0]; float ry_left = weizi_l[1]; float rx_left = weizi_l[2];
    float tx_left = weizi_l[3]; float ty_left = weizi_l[4]; float tz_left = weizi_l[5];
    float rz_right = weizi_r[0]; float ry_right = weizi_r[1]; float rx_right = weizi_r[2];
    float tx_right = weizi_r[3]; float ty_right = weizi_r[4]; float tz_right = weizi_r[5];


//    float rz_left =  1.6513; float ry_left =  1.2116; float rx_left =  3.0766;
//    float tx_left =  -349; float ty_left =  169; float tz_left =  -19;
//    float rz_right = -1.8774; float ry_right = 1.2392; float rx_right = -2.8543;
//    float tx_right = -311; float ty_right = -250; float tz_right = 34;

    //std::cout << tx << " " << ty << " " << tz << std::endl;
    Eigen::Matrix4f matrix_hand_left = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f matrix_hand_right = Eigen::Matrix4f::Identity();
    matrix_hand_left(0, 3) = tx_left;        matrix_hand_left(1, 3) = ty_left;        matrix_hand_left(2, 3) = tz_left;
    matrix_hand_right(0, 3) = tx_right;      matrix_hand_right(1, 3) = ty_right;      matrix_hand_right(2, 3) = tz_right;
    Eigen::Matrix3f rZ_left = Eigen::AngleAxisf(rz_left, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    Eigen::Matrix3f rY_left = Eigen::AngleAxisf(ry_left, Eigen::Vector3f::UnitY()).toRotationMatrix();
    Eigen::Matrix3f rX_left = Eigen::AngleAxisf(rx_left, Eigen::Vector3f::UnitX()).toRotationMatrix();
    Eigen::Matrix3f rZ_right = Eigen::AngleAxisf(rz_right, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    Eigen::Matrix3f rY_right = Eigen::AngleAxisf(ry_right, Eigen::Vector3f::UnitY()).toRotationMatrix();
    Eigen::Matrix3f rX_right = Eigen::AngleAxisf(rx_right, Eigen::Vector3f::UnitX()).toRotationMatrix();

    // 总的旋转矩阵，定系旋转顺序为 Rz * Ry * Rx
    Eigen::Matrix3f rotation_matrix_left = rX_left * rY_left * rZ_left;
    Eigen::Matrix3f rotation_matrix_right = rX_right * rY_right * rZ_right;
    matrix_hand_left.block<3, 3>(0, 0) = rotation_matrix_left;  // 将 3x3 旋转矩阵插入到 4x4 矩阵中
    matrix_hand_right.block<3, 3>(0, 0) = rotation_matrix_right;
    std::cout << "left hand matrix:" << std::endl;
    std::cout << matrix_hand_left << std::endl;
    std::cout << "right hand matrix:" << std::endl;
    std::cout << matrix_hand_right << std::endl;
    Eigen::Matrix4f stc_left,stc_right,matrix_rtl,matrix_HR_HL;
    matrix_rtl << 1,0,0,0,
                0,-1,0,0,
                0,0,-1,-405.2,
                0.0, 0.0, 0.0, 1.0;

    std::cout << "right hand matrix inverse:" << std::endl;
    std::cout << matrix_hand_right.inverse() << std::endl;
    matrix_HR_HL = matrix_hand_right.inverse()*matrix_rtl*matrix_hand_left;
    std::cout << matrix_HR_HL << std::endl;
    robot_system->matrix_HR_HL = matrix_HR_HL;
}

void OpenLoongControler::on_pushButton_rcp_record_stamp_clicked()
{
    if(!demonstrate_data.empty()){
        stamp = 1.0*curr_demonstrate_index/demonstrate_data.size();
        qInfo()<<stamp;
    }else{
        qInfo()<<"no trajectory loaded";
    }

    Eigen::Matrix4f transformation_matrices;
    transformation_matrices <<0.680614,  -0.616649, -0.395613,  -297.877,
                            -0.180424,  -0.664426,  0.725249,   376.252,
                            -0.710079,  -0.422237, -0.563475,    109.16,
                             0       ,   0       ,  0       ,  1;
//    transformation_matrices << 0.6786 ,  -0.0877 ,   0.7293 ,-297.1650,
//                               -0.7168,   -0.2959,    0.6314,  347.5200,
//                                0.1604,   -0.9512,   -0.2637,   21.4891,
//                                     0,         0,         0,    1.0000;
//    Eigen::Matrix3f rotation_matrix = transformation_matrices.block<3, 3>(0, 0);
//    Eigen::Vector3f translation = transformation_matrices.block<3, 1>(0, 3);
//    Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // 2, 1, 0 rotate  Z, Y, X axis
//    float rz = euler_angles(0);        float ry = euler_angles(1);        float rx = euler_angles(2);
//    float x = translation(0);          float y = translation(1);          float z = translation(2);

    float y_beta = atan2(transformation_matrices(0,2),std::sqrt(transformation_matrices(0,0)*transformation_matrices(0,0)+transformation_matrices(0,1)*transformation_matrices(0,1)));
    float x_gamma,z_alpha;
    if(std::abs(y_beta-M_PI/2.0)<1e-6){
        x_gamma=0; z_alpha=atan2(transformation_matrices(1,0),transformation_matrices(1,1));
    }else if(std::abs(y_beta-(-M_PI/2.0))<1e-6){
        x_gamma=0;z_alpha=atan2(transformation_matrices(1,0),transformation_matrices(1,1));
    }else{
        z_alpha = atan2(-transformation_matrices(0,1)/std::cos(y_beta),transformation_matrices(0,0)/std::cos(y_beta));
        x_gamma = atan2(-transformation_matrices(1,2)/std::cos(y_beta),transformation_matrices(2,2)/std::cos(y_beta));
    }

    std::cout << "i: "<< 0 <<", rz: " << z_alpha << ", ry: " << y_beta << ", rx: " << x_gamma << ", tx: " << transformation_matrices(0,3) << ", ty: " << transformation_matrices(1,3) << ", tz: " << transformation_matrices(2,3) << std::endl;

}

void OpenLoongControler::on_pushButton_rcp_record_run_clicked()
{
    if (demonstrate_data.empty()) {
        qWarning() << "Demonstrate data is empty";
        return;
    }
    qInfo()<<demonstrate_data.size()<<" "<<demonstrate_data[0].size();
    curr_demonstrate_index=0;
    accumulated_time=0.0;
    rcp_run_timer->start(10);
}

void OpenLoongControler::handleRCP_run(){
    if(curr_demonstrate_index< demonstrate_data.size()){
        qInfo()<<curr_demonstrate_index<<" "<<demonstrate_data[curr_demonstrate_index][0]<<" "<<demonstrate_data[curr_demonstrate_index][1]<<" "<<demonstrate_data[curr_demonstrate_index][2]<<" "<<demonstrate_data[curr_demonstrate_index][3]<<" "<<demonstrate_data[curr_demonstrate_index][4]<<" "<<demonstrate_data[curr_demonstrate_index][5];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][0]=demonstrate_data[curr_demonstrate_index][0];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][1]=demonstrate_data[curr_demonstrate_index][1];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][2]=demonstrate_data[curr_demonstrate_index][2];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][3]=demonstrate_data[curr_demonstrate_index][3];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][4]=demonstrate_data[curr_demonstrate_index][4];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][5]=demonstrate_data[curr_demonstrate_index][5];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][6]=demonstrate_data[curr_demonstrate_index][6];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][0]=demonstrate_data[curr_demonstrate_index][7];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][1]=demonstrate_data[curr_demonstrate_index][8];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][2]=demonstrate_data[curr_demonstrate_index][9];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][3]=demonstrate_data[curr_demonstrate_index][10];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][4]=demonstrate_data[curr_demonstrate_index][11];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][5]=demonstrate_data[curr_demonstrate_index][12];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][6]=demonstrate_data[curr_demonstrate_index][13];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[0][0]=demonstrate_data[curr_demonstrate_index][14];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[0][1]=demonstrate_data[curr_demonstrate_index][15];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[0][2]=demonstrate_data[curr_demonstrate_index][16];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[0][3]=demonstrate_data[curr_demonstrate_index][17];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[0][4]=demonstrate_data[curr_demonstrate_index][18];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[0][5]=demonstrate_data[curr_demonstrate_index][19];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[1][0]=demonstrate_data[curr_demonstrate_index][20];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[1][1]=demonstrate_data[curr_demonstrate_index][21];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[1][2]=demonstrate_data[curr_demonstrate_index][22];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[1][3]=demonstrate_data[curr_demonstrate_index][23];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[1][4]=demonstrate_data[curr_demonstrate_index][24];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_exp_hand[1][5]=demonstrate_data[curr_demonstrate_index][25];
        robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.q_vcap_hand[0] = demonstrate_data[curr_demonstrate_index][26];
        //robot_system->robot_data->robot_info_.joint_cmd_.basic_cmd_info.q_vcap_hand[1] = demonstrate_data[curr_demonstrate_index][27];
        curr_demonstrate_index += 1;
    }else{
        rcp_run_timer->stop();
        qInfo()<<"Demonstrate end";
        if(bool_demonstrator_loop){
            qInfo()<<demonstrate_data.size()<<" "<<demonstrate_data[0].size()<<" loop";
            curr_demonstrate_index=0;
            accumulated_time=0.0;
            rcp_run_timer->start(10);
        }
    }
}

void OpenLoongControler::on_pushButton_rcp_record_clear_clicked()
{
    emit clearRecording();
}

void OpenLoongControler::on_pushButton_rcp_init_clicked()
{
    double dv3_l[7],dv3_r[7];
    double weizi_l[7],weizi_r[7];
    float k = 180.0*1000/M_PI;
    dv3_l[0]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][0]/k;
    dv3_l[1]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][1]/k;
    dv3_l[2]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][2]/k;
    dv3_l[3]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][3]/k;
    dv3_l[4]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][4]/k;
    dv3_l[5]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][5]/k;
    dv3_l[6]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][6]/k;
    dv3_r[0]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][0]/k;
    dv3_r[1]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][1]/k;
    dv3_r[2]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][2]/k;
    dv3_r[3]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][3]/k;
    dv3_r[4]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][4]/k;
    dv3_r[5]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][5]/k;
    dv3_r[6]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][6]/k;

    k_OLR2(dv3_l, weizi_l);
    k_OLR2(dv3_r, weizi_r);

    //std::cout<<dv3_l[0]<<" "<<dv3_l[1]<<" "<<dv3_l[2]<<" "<<dv3_l[3]<<" "<<dv3_l[4]<<" "<<dv3_l[5]<<" "<<dv3_l[6]<<std::endl;
    std::cout<<weizi_l[0]<<" "<<weizi_l[1]<<" "<<weizi_l[2]<<" "<<weizi_l[3]<<" "<<weizi_l[4]<<" "<<weizi_l[5]<<" "<<weizi_l[6]<<std::endl;
    //std::cout<<dv3_r[0]<<" "<<dv3_r[1]<<" "<<dv3_r[2]<<" "<<dv3_r[3]<<" "<<dv3_r[4]<<" "<<dv3_r[5]<<" "<<dv3_r[6]<<std::endl;
    std::cout<<weizi_r[0]<<" "<<weizi_r[1]<<" "<<weizi_r[2]<<" "<<weizi_r[3]<<" "<<weizi_r[4]<<" "<<weizi_r[5]<<" "<<weizi_r[6]<<std::endl;

    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][0]=static_cast<float>(weizi_l[0]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][1]=static_cast<float>(weizi_l[1]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][2]=static_cast<float>(weizi_l[2]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][3]=static_cast<float>(weizi_l[3]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][4]=static_cast<float>(weizi_l[4]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][5]=static_cast<float>(weizi_l[5]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[0][6]=static_cast<float>(weizi_l[6]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][0]=static_cast<float>(weizi_r[0]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][1]=static_cast<float>(weizi_r[1]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][2]=static_cast<float>(weizi_r[2]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][3]=static_cast<float>(weizi_r[3]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][4]=static_cast<float>(weizi_r[4]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][5]=static_cast<float>(weizi_r[5]);
    robot_system->robot_data->robot_info_.motion_data_.basic_cmd_info.ee_motion[1][6]=static_cast<float>(weizi_r[6]);
}

void OpenLoongControler::on_pushButton_rcp_manual_clicked()
{
    robot_system->from_rcp=false;
    bool_manualEnabled=true;
    ui->label_rcp_mora->setText("Manual");
}

void OpenLoongControler::on_pushButton_rcp_auto_clicked()
{
    robot_system->from_rcp=true;
    bool_manualEnabled=false;
    ui->label_rcp_mora->setText("Auto");
}

void OpenLoongControler::on_pushButton_rpc_setpset_clicked()
{
    rcp_step_rxyz = ui->lineEdit_rcp_step_rxyz->text().toFloat();
    rcp_step_x = ui->lineEdit_rcp_step_x->text().toFloat();
    rcp_step_y = ui->lineEdit_rcp_step_y->text().toFloat();
    rcp_step_z = ui->lineEdit_rcp_step_z->text().toFloat();
}

void OpenLoongControler::on_horizontalSlider_rcp_k_valueChanged(int value)
{
    if(value>0){
        rcp_k=1.0*value;
    }else if(value<0){
        rcp_k=1.0/value;
    }else{
        rcp_k=1.0;
    }
    ui->lineEdit_rcp_k->setText(QString::number(value));
}

void OpenLoongControler::on_lineEdit_rcp_k_editingFinished()
{
    rcp_k=ui->lineEdit_rcp_k->text().toFloat();
    ui->horizontalSlider_rcp_k->setValue(rcp_k);
}

void OpenLoongControler::on_pushButton_rcp_lock_clicked()
{
    robot_system->rcp_lock = true;
}

void OpenLoongControler::on_pushButton_rcp_unlock_clicked()
{
    robot_system->rcp_lock = false;
}

//hand

void OpenLoongControler::drawHand(){
    QPainter p(ui->widget_hand);

    QPixmap image(":/hand_left.jpg");
    QPixmap scaledImage = image.scaled(370, 340);
    p.drawPixmap(30, 0, scaledImage);
    QPixmap mirroredImage(":/hand_right.jpg");
    QPixmap scaledMirrorImage = mirroredImage.scaled(370, 340);
    p.drawPixmap(340, 0, scaledMirrorImage);
    p.drawImage(0,0,heatImg);
    dataImg=QImage(ImgWidth,ImgHeight,QImage::Format_Alpha8);
    dataImg.fill(Qt::transparent);
    heatImg=QImage(ImgWidth,ImgHeight,QImage::Format_ARGB32);
    heatImg.fill(Qt::transparent);

    if(!draw_timer->isActive()){
        draw_timer->start(400);
    }
}

void OpenLoongControler::drawHand_update(){
    //random number
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 65536);
    point_list.clear();

    //left hand 63
//    point_list.push_back(Point{160, 170, dis(gen)});
//    point_list.push_back(Point{175, 170, dis(gen)});
//    point_list.push_back(Point{190, 170, dis(gen)});
//    point_list.push_back(Point{205, 170, dis(gen)});
//    point_list.push_back(Point{220, 170, dis(gen)});
//    point_list.push_back(Point{235, 170, dis(gen)});
//    point_list.push_back(Point{250, 170, dis(gen)});
//    point_list.push_back(Point{265, 170, dis(gen)});
//    point_list.push_back(Point{280, 170, dis(gen)});

//    point_list.push_back(Point{160, 180, dis(gen)});
//    point_list.push_back(Point{175, 180, dis(gen)});
//    point_list.push_back(Point{190, 180, dis(gen)});
//    point_list.push_back(Point{205, 180, dis(gen)});
//    point_list.push_back(Point{220, 180, dis(gen)});
//    point_list.push_back(Point{235, 180, dis(gen)});
//    point_list.push_back(Point{250, 180, dis(gen)});
//    point_list.push_back(Point{265, 180, dis(gen)});
//    point_list.push_back(Point{280, 180, dis(gen)});

//    point_list.push_back(Point{160, 190, dis(gen)});
//    point_list.push_back(Point{175, 190, dis(gen)});
//    point_list.push_back(Point{190, 190, dis(gen)});
//    point_list.push_back(Point{205, 190, dis(gen)});
//    point_list.push_back(Point{220, 190, dis(gen)});
//    point_list.push_back(Point{235, 190, dis(gen)});
//    point_list.push_back(Point{250, 190, dis(gen)});
//    point_list.push_back(Point{265, 190, dis(gen)});
//    point_list.push_back(Point{280, 190, dis(gen)});

//    point_list.push_back(Point{160, 200, dis(gen)});
//    point_list.push_back(Point{175, 200, dis(gen)});
//    point_list.push_back(Point{190, 200, dis(gen)});
//    point_list.push_back(Point{205, 200, dis(gen)});
//    point_list.push_back(Point{220, 200, dis(gen)});
//    point_list.push_back(Point{235, 200, dis(gen)});
//    point_list.push_back(Point{250, 200, dis(gen)});
//    point_list.push_back(Point{265, 200, dis(gen)});
//    point_list.push_back(Point{280, 200, dis(gen)});

//    point_list.push_back(Point{160, 210, dis(gen)});
//    point_list.push_back(Point{175, 210, dis(gen)});
//    point_list.push_back(Point{190, 210, dis(gen)});
//    point_list.push_back(Point{205, 210, dis(gen)});
//    point_list.push_back(Point{220, 210, dis(gen)});
//    point_list.push_back(Point{235, 210, dis(gen)});
//    point_list.push_back(Point{250, 210, dis(gen)});
//    point_list.push_back(Point{265, 210, dis(gen)});
//    point_list.push_back(Point{280, 210, dis(gen)});

//    point_list.push_back(Point{160, 220, dis(gen)});
//    point_list.push_back(Point{175, 220, dis(gen)});
//    point_list.push_back(Point{190, 220, dis(gen)});
//    point_list.push_back(Point{205, 220, dis(gen)});
//    point_list.push_back(Point{220, 220, dis(gen)});
//    point_list.push_back(Point{235, 220, dis(gen)});
//    point_list.push_back(Point{250, 220, dis(gen)});
//    point_list.push_back(Point{265, 220, dis(gen)});
//    point_list.push_back(Point{280, 220, dis(gen)});

//    point_list.push_back(Point{160, 230, dis(gen)});
//    point_list.push_back(Point{175, 230, dis(gen)});
//    point_list.push_back(Point{190, 230, dis(gen)});
//    point_list.push_back(Point{205, 230, dis(gen)});
//    point_list.push_back(Point{220, 230, dis(gen)});
//    point_list.push_back(Point{235, 230, dis(gen)});
//    point_list.push_back(Point{250, 230, dis(gen)});
//    point_list.push_back(Point{265, 230, dis(gen)});
//    point_list.push_back(Point{280, 230, dis(gen)});
    //left thumb
    point_list.push_back(Point{70, 180, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[0]});
    point_list.push_back(Point{70, 170, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[1]});
    point_list.push_back(Point{80, 180, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[2]});
    point_list.push_back(Point{80, 170, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[3]});
    //left index
    point_list.push_back(Point{130, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[0]});
    point_list.push_back(Point{130, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[1]});
    point_list.push_back(Point{140, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[2]});
    point_list.push_back(Point{140, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[3]});
    //left middle
    point_list.push_back(Point{200, 40, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[0]});
    point_list.push_back(Point{200, 30, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[1]});
    point_list.push_back(Point{210, 30, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[2]});
    point_list.push_back(Point{210, 40, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[3]});
    //left ring
    point_list.push_back(Point{255, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[0]});
    point_list.push_back(Point{255, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[1]});
    point_list.push_back(Point{265, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[2]});
    point_list.push_back(Point{265, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[3]});
    //left little
    point_list.push_back(Point{310, 110, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[0]});
    point_list.push_back(Point{310, 100, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[1]});
    point_list.push_back(Point{320, 110, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[2]});
    point_list.push_back(Point{320, 100, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[3]});

    //right hand 63
//    point_list.push_back(Point{460, 170, dis(gen)});
//    point_list.push_back(Point{475, 170, dis(gen)});
//    point_list.push_back(Point{490, 170, dis(gen)});
//    point_list.push_back(Point{505, 170, dis(gen)});
//    point_list.push_back(Point{520, 170, dis(gen)});
//    point_list.push_back(Point{535, 170, dis(gen)});
//    point_list.push_back(Point{550, 170, dis(gen)});
//    point_list.push_back(Point{565, 170, dis(gen)});
//    point_list.push_back(Point{580, 170, dis(gen)});

//    point_list.push_back(Point{460, 180, dis(gen)});
//    point_list.push_back(Point{475, 180, dis(gen)});
//    point_list.push_back(Point{490, 180, dis(gen)});
//    point_list.push_back(Point{505, 180, dis(gen)});
//    point_list.push_back(Point{520, 180, dis(gen)});
//    point_list.push_back(Point{535, 180, dis(gen)});
//    point_list.push_back(Point{550, 180, dis(gen)});
//    point_list.push_back(Point{565, 180, dis(gen)});
//    point_list.push_back(Point{580, 180, dis(gen)});

//    point_list.push_back(Point{460, 190, dis(gen)});
//    point_list.push_back(Point{475, 190, dis(gen)});
//    point_list.push_back(Point{490, 190, dis(gen)});
//    point_list.push_back(Point{505, 190, dis(gen)});
//    point_list.push_back(Point{520, 190, dis(gen)});
//    point_list.push_back(Point{535, 190, dis(gen)});
//    point_list.push_back(Point{550, 190, dis(gen)});
//    point_list.push_back(Point{565, 190, dis(gen)});
//    point_list.push_back(Point{580, 190, dis(gen)});

//    point_list.push_back(Point{460, 200, dis(gen)});
//    point_list.push_back(Point{475, 200, dis(gen)});
//    point_list.push_back(Point{490, 200, dis(gen)});
//    point_list.push_back(Point{505, 200, dis(gen)});
//    point_list.push_back(Point{520, 200, dis(gen)});
//    point_list.push_back(Point{535, 200, dis(gen)});
//    point_list.push_back(Point{550, 200, dis(gen)});
//    point_list.push_back(Point{565, 200, dis(gen)});
//    point_list.push_back(Point{580, 200, dis(gen)});

//    point_list.push_back(Point{460, 210, dis(gen)});
//    point_list.push_back(Point{475, 210, dis(gen)});
//    point_list.push_back(Point{490, 210, dis(gen)});
//    point_list.push_back(Point{505, 210, dis(gen)});
//    point_list.push_back(Point{520, 210, dis(gen)});
//    point_list.push_back(Point{535, 210, dis(gen)});
//    point_list.push_back(Point{550, 210, dis(gen)});
//    point_list.push_back(Point{565, 210, dis(gen)});
//    point_list.push_back(Point{580, 210, dis(gen)});

//    point_list.push_back(Point{460, 220, dis(gen)});
//    point_list.push_back(Point{475, 220, dis(gen)});
//    point_list.push_back(Point{490, 220, dis(gen)});
//    point_list.push_back(Point{505, 220, dis(gen)});
//    point_list.push_back(Point{520, 220, dis(gen)});
//    point_list.push_back(Point{535, 220, dis(gen)});
//    point_list.push_back(Point{550, 220, dis(gen)});
//    point_list.push_back(Point{565, 220, dis(gen)});
//    point_list.push_back(Point{580, 220, dis(gen)});

//    point_list.push_back(Point{460, 230, dis(gen)});
//    point_list.push_back(Point{475, 230, dis(gen)});
//    point_list.push_back(Point{490, 230, dis(gen)});
//    point_list.push_back(Point{505, 230, dis(gen)});
//    point_list.push_back(Point{520, 230, dis(gen)});
//    point_list.push_back(Point{535, 230, dis(gen)});
//    point_list.push_back(Point{550, 230, dis(gen)});
//    point_list.push_back(Point{565, 230, dis(gen)});
//    point_list.push_back(Point{580, 230, dis(gen)});
    //right thumb
    point_list.push_back(Point{660, 180, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_r[0]});
    point_list.push_back(Point{660, 170, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_r[1]});
    point_list.push_back(Point{670, 180, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_r[2]});
    point_list.push_back(Point{670, 170, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_r[3]});
    //right index
    point_list.push_back(Point{600, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_r[0]});
    point_list.push_back(Point{600, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_r[1]});
    point_list.push_back(Point{610, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_r[2]});
    point_list.push_back(Point{610, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_r[3]});
    //right middle
    point_list.push_back(Point{530, 30, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_r[0]});
    point_list.push_back(Point{530, 40, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_r[1]});
    point_list.push_back(Point{540, 30, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_r[2]});
    point_list.push_back(Point{540, 40, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_r[3]});
    //right ring
    point_list.push_back(Point{475, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_r[0]});
    point_list.push_back(Point{475, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_r[1]});
    point_list.push_back(Point{485, 50, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_r[2]});
    point_list.push_back(Point{485, 60, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_r[3]});
    //right little
    point_list.push_back(Point{420, 110, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_r[0]});
    point_list.push_back(Point{420, 100, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_r[1]});
    point_list.push_back(Point{430, 110, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_r[2]});
    point_list.push_back(Point{430, 100, robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_r[3]});

//    qInfo()<<"Left thumb"<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[3]<<" ";
//    qInfo()<<"Left Index: "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[3];
//    qInfo()<<"Left middel: "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[3];
//    qInfo()<<"Left ring"<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[3];
//    qInfo()<<"Left little"<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[3];
//    qInfo()<<"";
    max_count=0;
    for(int i=0;i<point_list.length();i++){
        countTable[point_list[i].posX+point_list[i].posY*ImgWidth]=point_list[i].count;
        if(point_list[i].count>max_count){
            max_count=point_list[i].count;
        }
    }

    //dataImg
    dataImg.fill(Qt::transparent);
    QPainter painter(&dataImg);
    painter.setPen(Qt::transparent);
    for(int i=0;i<point_list.count();i++)
    {
        const Point &pt=point_list.at(i);
        const uchar alpha=uchar(countTable[pt.posX+pt.posY*ImgWidth]/max_count*255);
        QRadialGradient gradient(pt.posX,pt.posY,radius);
        gradient.setColorAt(0,QColor(0,0,0,alpha));
        gradient.setColorAt(1,QColor(0,0,0,0));
        painter.setBrush(gradient);
        painter.drawEllipse(QPointF(pt.posX,pt.posY),radius,radius);
    }

    //heatImg
    heatImg.fill(Qt::transparent);
    for(int row=0;row<dataImg.height();row++)
    {
        const uchar *line_data=dataImg.scanLine(row);
        QRgb *line_heat=reinterpret_cast<QRgb*>(heatImg.scanLine(row));
        for(int col=0;col<dataImg.width();col++)
        {
            line_heat[col]=colorList[line_data[col]];
        }
    }
    update();
}

bool OpenLoongControler::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->widget_hand && event->type() == QEvent::Paint)
    {
        drawHand();
        //qDebug() << "drawing";
        return true;
    }
    return QWidget::eventFilter(watched, event);
}


