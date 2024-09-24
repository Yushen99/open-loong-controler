#ifndef ROBOTSYSTEM_H
#define ROBOTSYSTEM_H
#include "ArmFunction/RobotData.h"
#include <QTimer>
#include "ArmFunction/udp_client.h"
#include "ArmFunction/udp_server.h"
#include "vector"
#include <QObject>
#include <QDebug>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "k_ORL2/k_OLR2.h"
//#include "k_ORL2/k_OLR2_initialize.h"
//#include "k_ORL2/k_OLR2_terminate.h"
//#include "k_ORL2/rt_nonfinite.h"
//#include "k_ORL2/rtGetInf.h"
//#include "k_ORL2/rtGetNaN.h"
//#include "k_ORL2/norm.h"
//#include "k_ORL2/CoordinateTrans.h"

class RobotSystem : public QObject
{
    Q_OBJECT
public:
    RobotSystem(){
        robot_data = new RobotData();
        udp_server = new UdpServer(robot_data);

    }

    enum MotionMode{
      IDLE=0,
      DYNAMIC,
      MANUAL,
      AUTO,
      DEMONSTRATOR,
      MOTION_CAPTURE
    };
    enum RunningMode{
        INIT=0,
        INIT_OK,
        READY,
        READY_OK,
        RUN,
        STOP,
        DISABLE,
        ERROR
    };


    int SetMotionCaptureComm(const char * robot_ip, int port);
    int SetManualComm(const char * robot_ip, int port);
    int SetCmdComm(const char * robot_ip, int port);
    int SetFk(const char * robot_ip, int port);
    int SetRemoteDevice(const char * robot_ip, int port);
    int SetAgv(const char * robot_ip, int port);
    int CommunicationStart();
    int Enable();
    int Disable();
    unsigned short calculateCRC(const unsigned char* data, unsigned int length);
    void floatToBytes(float value, unsigned char* bytes);
public slots:
    void UdpClientsRun();
    void UdpClientRemoteRun();
    void UdpClientAgv();

public:
    UdpClient *udp_client_robot_ee;
    UdpClient *udp_client_robot_manual;
    UdpClient *udp_client_robot_command;
    UdpClient *udp_client_fk;
    UdpClient *udp_client_remote_device;
    UdpClient *udp_client_agv;
    std::vector<UdpClient *> remote_computers;

    UdpServer * udp_server;
    RobotData * robot_data;
    MotionMode  motion_mode, motion_mode_cur;
    RunningMode  running_mode, running_mode_cur;
    bool from_rcp=true;
    bool rcp_lock=false;
    Eigen::Matrix4f matrix_HR_HL;
    float x_v = 0.0f, y_v = 0.0f, z_angle = 0.0f;//agv
    bool agv_enable=false;


private:

    int ModeUpdate();



};

#endif // ROBOTSYSTEM_H
