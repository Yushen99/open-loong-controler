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
    robot_system->SetCmdComm(robot_ip.data(), PORT_MOTION_CMD);
    robot_system->SetManualComm(robot_ip.data(), PORT_MOTION_JOINT);
    robot_system->SetMotionCaptureComm(robot_ip.data(), PORT_MOTION_EE);


    clients_com_timer = new QTimer(this);
    connect(clients_com_timer,&QTimer::timeout, robot_system, &RobotSystem::UdpClientsRun);

    clients_com_timer->start(2);

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
    if(pressedKeys.isEmpty())
        keyRespondTimer->stop();}

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
        }
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
    ui->label_feedback_r_A_19->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist_exp[0]));
    ui->label_feedback_r_A_20->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist_exp[1]));
    ui->label_feedback_r_A_21->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_waist_exp[2]));
    ui->label_feedback_r_A_22->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head_exp[0]));
    ui->label_feedback_r_A_23->setText(QString::number(
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_head_exp[1]));


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

    //fk
    ui->label_feedback_l_fk_1->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[0][0])));
    ui->label_feedback_l_fk_2->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[0][1])));
    ui->label_feedback_l_fk_3->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[0][2])));
    ui->label_feedback_l_fk_4->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[0][3])));
    ui->label_feedback_l_fk_5->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[0][4])));
    ui->label_feedback_l_fk_6->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[0][5])));
    ui->label_feedback_l_fk_7->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[0][6])));
    ui->label_feedback_r_fk_1->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[1][0])));
    ui->label_feedback_r_fk_2->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[1][1])));
    ui->label_feedback_r_fk_3->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[1][2])));
    ui->label_feedback_r_fk_4->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[1][3])));
    ui->label_feedback_r_fk_5->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[1][4])));
    ui->label_feedback_r_fk_6->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[1][5])));
    ui->label_feedback_r_fk_7->setText(QString::number((
         robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.arm_cartesion[1][6])));

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
        if(rgb){
            //qInfo()<<interpolated_data[27]<<" "<<interpolated_data[28]<<" "<<interpolated_data[29];
            R_value=interpolated_data[27];
            G_value=interpolated_data[28];
            B_value=interpolated_data[29];
        }
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

void OpenLoongControler::on_pushButton_rcp_record_modify_clicked()
{
    float tf = demonstrate_data.size()/100.0;
    for(unsigned long i=0;i<demonstrate_data.size();++i){
        float theta_r_x = 10.0*delta_r_x/std::pow(tf,3)*pow((i/100.0),3)-15.0*delta_r_x/std::pow(tf,4)*pow((i/100.0),4)+6.0*delta_r_x/std::pow(tf,5)*pow((i/100.0),5);
        float theta_r_y = 10.0*delta_r_y/std::pow(tf,3)*pow((i/100.0),3)-15.0*delta_r_y/std::pow(tf,4)*pow((i/100.0),4)+6.0*delta_r_y/std::pow(tf,5)*pow((i/100.0),5);
        float theta_r_z = 10.0*delta_r_z/std::pow(tf,3)*pow((i/100.0),3)-15.0*delta_r_z/std::pow(tf,4)*pow((i/100.0),4)+6.0*delta_r_z/std::pow(tf,5)*pow((i/100.0),5);
        qInfo()<<theta_r_x<<" "<<theta_r_y<<" "<<theta_r_z;
    }
}

void OpenLoongControler::on_pushButton_rcp_record_run_clicked()
{
    if (demonstrate_data.empty()) {
        qWarning() << "Demonstrate data is empty";
        return;
    }
    qInfo()<<demonstrate_data.size()<<" "<<demonstrate_data[0].size();

}

void OpenLoongControler::on_pushButton_rcp_record_clear_clicked()
{
    emit clearRecording();
}

void OpenLoongControler::on_pushButton_rcp_init_clicked()
{
    double dv3_l[7],dv3_r[7];
    double weizi_l[7],weizi_r[7];
    dv3_l[0]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][0]/57300;
    dv3_l[1]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][1]/57300;
    dv3_l[2]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][2]/57300;
    dv3_l[3]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][3]/57300;
    dv3_l[4]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][4]/57300;
    dv3_l[5]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][5]/57300;
    dv3_l[6]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[0][6]/57300;
    dv3_r[0]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][0]/57300;
    dv3_r[1]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][1]/57300;
    dv3_r[2]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][2]/57300;
    dv3_r[3]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][3]/57300;
    dv3_r[4]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][4]/57300;
    dv3_r[5]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][5]/57300;
    dv3_r[6]=robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.joint_q_arm[1][6]/57300;

    k_OLR2(dv3_l, weizi_l);
    k_OLR2(dv3_r, weizi_r);

    std::cout<<dv3_l[0]<<" "<<dv3_l[1]<<" "<<dv3_l[2]<<" "<<dv3_l[3]<<" "<<dv3_l[4]<<" "<<dv3_l[5]<<" "<<dv3_l[6]<<std::endl;
    std::cout<<weizi_l[0]<<" "<<weizi_l[1]<<" "<<weizi_l[2]<<" "<<weizi_l[3]<<" "<<weizi_l[4]<<" "<<weizi_l[5]<<" "<<weizi_l[6]<<std::endl;
    std::cout<<dv3_r[0]<<" "<<dv3_r[1]<<" "<<dv3_r[2]<<" "<<dv3_r[3]<<" "<<dv3_r[4]<<" "<<dv3_r[5]<<" "<<dv3_r[6]<<std::endl;
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

    qInfo()<<"Left thumb"<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_thumb_l[3]<<" ";
    qInfo()<<"Left Index: "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_index_finger_l[3];
    qInfo()<<"Left middel: "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_middle_finger_l[3];
    qInfo()<<"Left ring"<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_ring_finger_l[3];
    qInfo()<<"Left little"<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[0]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[1]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[2]<<" "<<robot_system->robot_data->robot_info_.robot_feedback_info_.basic_info.sensor_little_finger_l[3];
    qInfo()<<"";
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

void OpenLoongControler::on_pushButton_reset_error_clicked()
{
    robot_system->robot_data->robot_info_.robot_cmd_send_.reset_error = 1;
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (button) {
        button->setStyleSheet("background-color: yellow;"); //
    }
    qInfo()<<"reset error sent!";
}
