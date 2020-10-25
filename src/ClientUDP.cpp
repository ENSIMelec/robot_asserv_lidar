#include "ClientUDP.h"

using namespace std;

ClientUDP::ClientUDP(string ip, int port)
{
    ipServeur = ip;
    portServeur= port;
    sockfd = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    cout << "Setup des infos Serveur :"<<ipServeur<<":"<<portServeur<<endl;

    // filling server info
    serveur.sin_family = AF_INET;
    serveur.sin_port = htons(portServeur);
    serveur.sin_addr.s_addr = inet_addr(ipServeur.c_str());

    serveurRobot.sin_family = AF_INET;
    serveurRobot.sin_port = htons(1111);
    serveurRobot.sin_addr.s_addr = inet_addr("127.0.0.1");
    points=0;
    //sendMessage("Reset");

}

// receive message from UDP
string ClientUDP::receiveMessage() {
    char buffer[1500];
    sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    int ret = recvfrom(sockfd, buffer, 1500, 0, reinterpret_cast<sockaddr*>(&from), &fromlen);
    if (ret <= 0)
    {
        cout << "Error receiving message." << endl;
        return "";
    }
    return string(buffer);
}
bool ClientUDP::sendTo(string str, string ip, int port) {

    cout << "Setup des infos client :"<<ip<<":"<<port<<endl;
    struct sockaddr_in client;
    // filling client info
    client.sin_family = AF_INET;
    client.sin_port = htons(port);
    client.sin_addr.s_addr = inet_addr(ip.c_str());

    int ret = sendto(sockfd, str.c_str(), str.size()+1, 0,(struct sockaddr *)&client, sizeof(client));
    if (ret <= 0)
    {
        cout << "Erreur lors de l'envoi au serveur de : ["<< str <<"]"<<endl;
        return false;
    }else{
        //cout << "Envoie au serveur de : ["<< str <<"]"<<endl;
        return true;
    }

}
bool ClientUDP::sendMessage(string str){
	int ret = sendto(sockfd, str.c_str(), str.size()+1, 0,(struct sockaddr *)&serveur, sizeof(serveur));
    sendto(sockfd, str.c_str(), str.size()+1, 0,(struct sockaddr *)&serveurRobot, sizeof(serveurRobot));
		if (ret <= 0)
		{
			cout << "Erreur lors de l'envoi au serveur de : ["<< str <<"]"<<endl;
			return false;
		}else{
			//cout << "Envoie au serveur de : ["<< str <<"]"<<endl;
			return true;
		}


}

/*int ClientUDP::calculPoints(int nbPointsTheorique) {
    if(nbPointsTheorique <= 10) { // 0-10
        return nbPointsTheorique;
    } else if(nbPointsTheorique <= 20) { //11-20
        return nbPointsTheorique - 10;
    } else if(nbPointsTheorique <= 40) { //21-40
        return nbPointsTheorique - 10;
    } else if(nbPointsTheorique <= 60) { //41-60
        return nbPointsTheorique - 20;
    } else if(nbPointsTheorique <= 80) { //61-80
        return nbPointsTheorique - 20;
    } else if(nbPointsTheorique <= 100) { //81-100
        return nbPointsTheorique - 20;
    } else if(nbPointsTheorique <= 130) { //101-130
        return nbPointsTheorique - 20;
    } else if(nbPointsTheorique <= 160) { //131-160
        return nbPointsTheorique - 30;
    } else if(nbPointsTheorique <= 190) { //161-190
        return nbPointsTheorique - 30;
    } else if(nbPointsTheorique <= 220) { //191-220
        return nbPointsTheorique - 30;
    } else if(nbPointsTheorique <= 260) { //221-260
        return nbPointsTheorique - 30;
    } else if(nbPointsTheorique <= 300) { //261-300
        return nbPointsTheorique - 40;
    } else if(nbPointsTheorique <= 340) { //301-340
        return nbPointsTheorique - 40;
    } else if(nbPointsTheorique <= 380) { //341-380
        return nbPointsTheorique - 40;
    } else if(nbPointsTheorique <= 420) { //381-420
        return nbPointsTheorique - 40;
    } else if(nbPointsTheorique <= 500) { //421-500
        return nbPointsTheorique - 40;
    } else { // > 500
        return nbPointsTheorique - 80;
    }

    return points;
}*/


void ClientUDP::addPoints(int pts, int proba){
    float probabPourcentage;

    if(proba == 0){
        points+=pts;
    }else
    {
        probabPourcentage = proba/100;
        points = points + static_cast<int>(pts*probabPourcentage);
    }
    //sendMessage("P "+to_string(points));
    //cout << "Points :" << points << endl;

    /*int pointsSeuil = calculPoints(points);*/      //Fonction commentée et rendue inutile
    sendMessage("P "+to_string(points));
    cout << "Points :" << points << endl;
}


/*N'est jamais appelée mais attention si elle est utilisée, elle pourrait fausser la prédiction*/
void ClientUDP::setPoints(int pts){
    points=pts;
    sendMessage("P "+to_string(points));
    cout << "Points :" << points << endl;
}

int ClientUDP::getPoints(){
    /*return calculPoints(points);*/
    return points; 
}

void ClientUDP::setTime(int time){
    sendMessage("T "+to_string(time));
    cout << "Time :" << time << endl;
}
