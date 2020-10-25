#ifndef CLIENTUDP_H_INCLUDED
#define CLIENTUDP_H_INCLUDED

#include <arpa/inet.h>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <netdb.h>
#include <stdio.h>
#include <string>

class ClientUDP
{
public:
    ClientUDP(std::string ip, int port);
    bool sendMessage(std::string str);
    std::string receiveMessage();
    void setPoints(int pts);
    void addPoints(int pts, int proba);
    int getPoints();
    int points;
    bool sendTo(std::string str, std::string ip, int port);
    void setTime(int time);

private:
    //Log ipServeur
    int calculPoints(int nbPointsTheorique);
    struct sockaddr_in serveur;
    struct sockaddr_in serveurRobot;
    int sockfd;
    std::string ipServeur;
    int portServeur;
};

#endif //CLIENTUDP_H_INCLUDED
