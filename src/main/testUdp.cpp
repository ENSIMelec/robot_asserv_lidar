
#include "ClientUDP.h"
#include "TCPServer.h"
int main() {

    //ClientUDP clientUdp("172.24.1.148", 1234);
    //clientUdp.sendMessage("/touch");

    //ClientUDP clientUdp("0.0.0.0", 5000);
    //clientUdp.sendTo("hello", "172.24.1.52", 5000);


    //std::cout << "Waiting receiving : " << std::endl;
    //std::string s = clientUdp.receiveMessage();
    // if string is empty error
    //std::cout << "I receive : " << s << std::endl;


    TCPServer tcpServer(1200);
    //while (true) {
        // connected then
        if(tcpServer.accept()) {
            tcpServer.send("Hello from server");
        }
        std::cout << tcpServer.read() << std::endl;
    //}

    return 0;
}