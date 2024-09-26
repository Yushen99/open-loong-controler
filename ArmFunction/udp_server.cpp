#include <memory.h>
#include <assert.h>
#include <sys/ioctl.h>
#include "ArmFunction/udp_server.h"
#include <iostream>
using namespace  std;

UdpServer::UdpServer(RobotData * robot_data):
    robot_data_(robot_data)
{
    for(int i = 0; i < PORT_NUM; i++)
    {
        server_socket[i] = new Server_Socket(SERVER_PORT[i]);
    }
}

void UdpServer::recv_data()
{
    Thread = std::thread(&UdpServer::recv_func, this);
    recv_ing = true;
}

void UdpServer::recv_func()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while(recv_ing)
    {
        for(int i = 0; i < PORT_NUM; i++)
        {
            int cn = server_socket[i]->recv_fun(recv_buf);
            if(cn > 0)
            {
                //if(server_socket[i]->_port == PORT_OCU) printf("###cn = %d\n",cn);
                //printf("port = %d\n", server_socket[i]->_port);
                rcv_calback(recv_buf, server_socket[i]->_port);
                fail_nn = 0;
            }
            else
            {
                fail_nn++;
                if(fail_nn>=5000)
                {
                    fail_nn = 5000;
                }
            }
        }
        usleep(10);
    }
}

void UdpServer::udpclose()
{
    for(int i = 0; i < 2; i++)
    {
        server_socket[i]->socket_close();
    }
}

int UdpServer::rcv_calback(void *package, uint16_t port){
    char *buf = (char *)package;
        //bool data_error = false;
        switch(port)
        {
            case PORT_MOTION:
                memcpy(&robot_data_->robot_info_.motion_data_recieve_,buf,sizeof(RobotData::Motion_Data_Recieve));

                break;
            case PORT_ROBOT_STATE:
                memcpy(&robot_data_->robot_info_.robot_state_,buf,sizeof(RobotData::RobotState));
                if(robot_data_->robot_info_.robot_state_.reset_feedback == 1 && robot_data_->robot_info_.robot_cmd_send_.reset_error == 1){
                    robot_data_->robot_info_.robot_cmd_send_.reset_error = 0;
                }
                break;
            case PORT_ROBOT_SERVO_INFO:
                memcpy(&robot_data_->robot_info_.robot_feedback_info_,buf,sizeof(RobotData::Robot_Feedback_Info));
                break;
            case PORT_REMOTE_COMPUTER:
                memcpy(&robot_data_->robot_info_.remote_computer_data_,buf,sizeof(RobotData::RemoteComputerData));
                break;
            case PORT_OTHER:
                break;

            default:
                break;
        }
    return 0;
}

