#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h> //F_SETFL
#include <sys/socket.h> //sendto socket
//#include <arpa/inet.h> //htons
#include <netinet/in.h> //sockaddr_in
#include "udp_driver.h"

#define SERVER_IP "192.168.1.102"
#define UDP_PORT 6000

bool flag_udp_init = false;
struct sockaddr_in gc_addr;

int sock_udp = -1;

int udp_init()
{
	struct sockaddr_in loc_addr;
	sock_udp = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock_udp == -1)
	{
		fprintf(stderr, "sock initialize failed: %s \n", strerror(errno));
		return -1;
	}
	memset(&loc_addr, 0, sizeof(loc_addr));
	loc_addr.sin_family = AF_INET;
	loc_addr.sin_addr.s_addr = INADDR_ANY;
	loc_addr.sin_port = htons(UDP_PORT);
	if (-1 == bind(sock_udp, (struct sockaddr *)&loc_addr, sizeof(struct sockaddr)))
	{
		fprintf(stderr, "bind failed: %s \n", strerror(errno));
		close(sock_udp);
		return -1;
	}
	if (-1 == fcntl(sock_udp, F_SETFL, O_NONBLOCK | FASYNC))
	{
		fprintf(stderr, "setting nonblocking err: %s \n", strerror(errno));
		close(sock_udp);
		return -1;
	}
	memset(&gc_addr, 0, sizeof(gc_addr));
	gc_addr.sin_family = AF_INET;
	gc_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
	gc_addr.sin_port = htons(UDP_PORT);
	flag_udp_init = true;
	return 0;
}

void udp_close()
{
	flag_udp_init = false;
	if (sock_udp != -1)
	{
		close(sock_udp);
		sock_udp = -1;
	}
}

int udp_send(uint8_t *ch, uint16_t length)
{
	uint16_t bytes_send = 0;
	bytes_send = sendto(sock_udp, ch, length, 0, (struct sockaddr *)&gc_addr, 
								sizeof(struct sockaddr_in));
	if (bytes_send == -1)
	{
		fprintf(stderr, "udp send err: %s \n", strerror(errno));
		return -1;
	}
	return bytes_send;
}