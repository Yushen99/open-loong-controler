#ifndef _UDP_CLIENT_H
#define _UDP_CLIENT_H
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>


using namespace std;

union Temp
{
    int temp_int;
    float temp_float;
    char  temp_char[4];
};

class UdpClient
{
public:
    enum {MAX_BUFFER = 1024};
    UdpClient(uint16_t port, const char *ip);
    ~UdpClient(){close(sock_fd);}

    static UdpClient *get_instance(uint16_t port, const char *ip)
    {
        static UdpClient *ins = 0;
        if(!ins)
        {
            ins = new UdpClient(port, ip);
        }
        return ins;
    }

    void send_data_buf(char *buf);
    void send_data_struct(const void *package);
    void set_send_buffer(char *send_buffer,int size);
public:
    char send_buf[MAX_BUFFER];
    int sock_fd;
    struct sockaddr_in addr_serv;
    int len;
    int send_num;
};
#endif
