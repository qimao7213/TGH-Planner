#include "tcp_communication.h"

int main() {
    TCPServer server(PORT);
    if (!server.start()) {
        std::cerr << "Server failed to start" << std::endl;
        return -1;
    }

    std::cout << "Waiting for client..." << std::endl;
    server.waitForClient();

    CustomData data;
    data.value1 = 123.456;
    data.value2 = 789.101;
    data.id = 1;
    strncpy(data.message, "Hello!", sizeof(data.message) - 1);
    data.message[sizeof(data.message) - 1] = '\0';

    while(true)
    {
        server.sendData(data);

        std::cout << "Sended data: " << std::endl;
        std::cout << "Value1: " << data.value1 << std::endl;
        std::cout << "Value2: " << data.value2 << std::endl;
        std::cout << "ID: " << data.id << std::endl;
        std::cout << "Message: " << data.message << std::endl;
    }


    return 0;
}

