//
// Created by Admin on 25/10/2020.
//

#ifndef TCPSERVER_TCPSERVER_H
#define TCPSERVER_TCPSERVER_H


#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

#define BUFSIZE 1000

class TCPServer {

public:
    TCPServer(int port);
    // accept incomming communication (blocking)
    bool accept();
    // send message to current clientaddr
    void send(std::string message);
    std::string read();
    void close();


private:
    int sockfd, n, connfd;
    struct sockaddr_in servaddr;
    struct sockaddr_in clientaddr;

};


#endif //TCPSERVER_TCPSERVER_H
