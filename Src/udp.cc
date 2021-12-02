#include<udp.hpp>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

int CSIR::UDP::create_and_bind(int port, int& len){

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }


    struct sockaddr_in addr_serv;
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(port); 
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY); 
    len = sizeof(addr_serv);

    bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv));

    return sock_fd; 
}

