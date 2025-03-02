#include "tcp_communication.h"
const char* server_ip = "127.0.0.1";

int main() {
    TCPClient client(server_ip, PORT);
    if (!client.connectToServer()) {
        std::cerr << "Connection to server failed" << std::endl;
        return -1;
    }

    CustomData data;
    data.value1 = 123.456;
    data.value2 = 789.101;
    data.id = 1;
    strncpy(data.message, "Hello!", sizeof(data.message) - 1);
    data.message[sizeof(data.message) - 1] = '\0';

    while(true)
    {
        client.sendData(data);

        std::cout << "Data sent: " << std::endl;
        std::cout << "Value1: " << data.value1 << std::endl;
        std::cout << "Value2: " << data.value2 << std::endl;
        std::cout << "ID: " << data.id << std::endl;
        std::cout << "Message: " << data.message << std::endl;
    }


    return 0;
}

