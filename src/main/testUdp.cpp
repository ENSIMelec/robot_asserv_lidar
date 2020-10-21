
#include "ClientUDP.h"

int main() {

    //ClientUDP clientUdp("172.24.1.148", 1234);
    //clientUdp.sendMessage("/touch");

    ClientUDP clientUdp("172.24.1.52", 5000);
    clientUdp.sendMessage("hello");

    std::cout << "Waiting receiving : " << std::endl;
    std::string s = clientUdp.receiveMessage();
    // if string is empty error
    std::cout << "I receive : " << s << std::endl;
    return 0;
}