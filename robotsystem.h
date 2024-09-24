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
    int CommunicationStart();
    int Enable();
    int Disable();
public slots:
    void UdpClientsRun();

public:
    UdpClient *udp_client_robot_ee;
    UdpClient *udp_client_robot_manual;
    UdpClient *udp_client_robot_command;

    std::vector<UdpClient *> remote_computers;

    UdpServer * udp_server;
    RobotData * robot_data;
    MotionMode  motion_mode, motion_mode_cur;
    RunningMode  running_mode, running_mode_cur;
    bool from_rcp=true;



private:

    int ModeUpdate();



};

#endif // ROBOTSYSTEM_H
