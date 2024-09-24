#ifndef _UDP_SERVER_H
#define _UDP_SERVER_H
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "RobotData.h"

#define MAX_BUFFER  1024*1024*2


using namespace std;

class Server_Socket
{
public:
    Server_Socket(uint16_t port)
    {
        sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sock_fd < 0)
        {
          perror("socket");
          return;
        }

        int op = 10000000;
        setsockopt(sock_fd,SOL_SOCKET,SO_RCVBUF,(const char*) &op,sizeof(int));
        memset(&addr_serv, 0, sizeof(struct sockaddr_in));
        addr_serv.sin_family = AF_INET;
        addr_serv.sin_port = htons(port);
        addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
        //addr_serv.sin_addr.s_addr = inet_addr("192.168.3.62");
        len = sizeof(addr_serv);
        //_port = port;
        if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
        {
          perror("bind error:");
          return;
        }
        unsigned long on = 1;

        ioctl(sock_fd, FIONBIO, &on);
        //printf("port = %d\n",ntohs(addr_serv.sin_port));
    }
    ~Server_Socket()
    {

    }
    void socket_close(){close(sock_fd);}
    int recv_fun(char *recv_buf)
    {
        int recv_num;
        recv_num = recvfrom(sock_fd, recv_buf, MAX_BUFFER, 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);
        _port = ntohs(addr_serv.sin_port);

        _client_port = ntohs(addr_serv.sin_port);
        _client_addr = addr_client.sin_addr.s_addr;
        return recv_num;
    }
    int len;
    int sock_fd;
    uint16_t _port;
    uint16_t _client_port;
    uint32_t _client_addr;

    struct sockaddr_in addr_serv;
    struct sockaddr_in addr_client;
};

class UdpServer
{
public:
    explicit UdpServer( RobotData * robot_data );//
    ~UdpServer()
    {
        recv_ing = false;
        //close(sock_fd);
    }
    void udpclose();
    int rcv_calback(void *package, uint16_t port);
    void recv_data();

    /***********************  add or reduce socket port  **************************/
    enum {PORT_MOTION = 8010,
          PORT_ROBOT_STATE = 8011,
          PORT_ROBOT_SERVO_INFO = 8012,
          PORT_REMOTE_COMPUTER = 8014,
          PORT_OTHER = 8015,
          PORT_NUM = 5};
    int SERVER_PORT[PORT_NUM] = {PORT_MOTION,PORT_ROBOT_STATE,PORT_ROBOT_SERVO_INFO,PORT_REMOTE_COMPUTER,PORT_OTHER};
    Server_Socket *server_socket[PORT_NUM];
    int fail_nn = 0;
    int extra_ocu = 0;

protected:
    char recv_buf[MAX_BUFFER];
    struct sockaddr_in addr_serv;
    struct sockaddr_in addr_client;
    int len;
    int recv_num;
    thread Thread;
    void recv_func();
    bool recv_ing = false;
    RobotData * robot_data_;
};
#endif
