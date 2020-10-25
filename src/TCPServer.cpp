//
// Created by Admin on 25/10/2020.
//

#include "TCPServer.h"

TCPServer::TCPServer(int port) {

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1) {
        printf("socket creation failed...\n");
        exit(0);
    }
    else
        printf("socket successfully created..\n");

    bzero(&servaddr, sizeof(servaddr));

    // assign ip and port
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(port);

    // Binding newly created socket to given IP and verification
    if ((bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr))) != 0) {
        printf("socket bind failed...\n");
        //exit(0);
    }
    else
        printf("socket successfully binded..\n");

    // Now server is ready to listen and verification
    if ((listen(sockfd, 5)) != 0) { /* allow 5 requests to queue up */
        printf("listen failed...\n");
        exit(0);
    }
    else
        printf("server listening..\n");
}

bool TCPServer::accept() {

    // Accept the data packet from client and verification
    socklen_t len = sizeof(clientaddr);
    this->connfd = ::accept(sockfd, (struct sockaddr *)&clientaddr, &len);
    if (connfd < 0) {
        printf("server accept failed...\n");
        return false;
    }

    printf("server accept the client...\n");
    return true;
}
std::string TCPServer::read() {
    char buff[BUFSIZE];
    bzero(buff, BUFSIZE);

    // read the message from client and copy it in buffer
    int bytes = ::read(this->connfd, buff, sizeof(buff));

    std::cout << "Receiving : " << bytes << " (bytes) : " << buff << std::endl;

    return std::string(buff);
}
void TCPServer::send(std::string message) {
    struct in_addr ipAddr = this->clientaddr.sin_addr;

    char str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &ipAddr, str, INET_ADDRSTRLEN );

    std::cout << "Sending to : (" << str << ") : " << message << std::endl;
    ::send(connfd, message.c_str(), message.length(), 0);
}

void TCPServer::close() {

    ::close(connfd);
    ::close(sockfd);
}
