#ifndef TCP_COMMUNICATION_H
#define TCP_COMMUNICATION_H

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

#define PORT 8080

struct CustomData {
    double value1;
    double value2;
    int id;
    char message[64];
};

class TCPServer {
public:
    TCPServer(int port);
    ~TCPServer();
    bool start();
    void waitForClient();
    void receiveData(CustomData& data);
    void sendData(const CustomData& data);

private:
    int server_fd;
    int new_socket;
    struct sockaddr_in address;
    int addrlen;
};

class TCPClient {
public:
    TCPClient(const char* server_ip, int port);
    ~TCPClient();
    bool connectToServer();
    void sendData(const CustomData& data);

private:
    int sock;
    struct sockaddr_in serv_addr;
};

#endif // TCP_COMMUNICATION_H

