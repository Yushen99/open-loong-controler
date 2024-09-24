#include <memory.h>
#include <assert.h>
#include <sys/ioctl.h>
#include "ArmFunction/udp_client.h"
/**********************************/

/*
 *
 * current do nothing
 * */


UdpClient::UdpClient(uint16_t port, const char *ip)
{
    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
    perror("socket");
    return;
    }
    memset(&addr_serv, 0, sizeof(addr_serv));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_addr.s_addr = inet_addr(ip);
    addr_serv.sin_port = htons(port);
    len = sizeof(addr_serv);

    int imode=1;
    ioctl(sock_fd,FIONBIO,(u_long *)&imode);

}
void UdpClient::send_data_buf(char *buf)
{
    send_num = sendto(sock_fd, &buf, sizeof(buf), 0, (struct sockaddr *)&addr_serv, len);
    if(send_num < 0)
    {
      perror("sendto error:");
      return;
    }
}

void UdpClient::send_data_struct(const void *package)
{
    send_num = sendto(sock_fd, &package, sizeof(package), 0, (struct sockaddr *)&addr_serv, len);
    if(send_num < 0)
    {
      perror("sendto error:");
      return;
    }
}

void UdpClient::set_send_buffer(char *send_buffer,int size)
{
    memcpy(send_buf,send_buffer,size);
    send_num = sendto(sock_fd, send_buffer,size, 0, (struct sockaddr *)&addr_serv, len);
    if(send_num < 0)
    {
      perror("sendto error:");
      return;
    }
}
