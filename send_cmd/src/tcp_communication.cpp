#include "tcp_communication.h"

// TCPServer implementation
TCPServer::TCPServer(int port) : server_fd(0), new_socket(0), addrlen(sizeof(address)) {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        close(server_fd);
        exit(EXIT_FAILURE);
    }
}

TCPServer::~TCPServer() {
    close(new_socket);
    close(server_fd);
}

bool TCPServer::start() {
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        close(server_fd);
        return false;
    }
    return true;
}

void TCPServer::waitForClient() {
    new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
    if (new_socket < 0) {
        perror("accept");
        close(server_fd);
        exit(EXIT_FAILURE);
    }
}

void TCPServer::receiveData(CustomData& data) {
    read(new_socket, &data, sizeof(data));
}

void TCPServer::sendData(const CustomData& data) {
    send(new_socket, &data, sizeof(data), 0);
}

// TCPClient implementation
TCPClient::TCPClient(const char* server_ip, int port) : sock(0) {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation error" << std::endl;
        exit(EXIT_FAILURE);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported" << std::endl;
        exit(EXIT_FAILURE);
    }
}

TCPClient::~TCPClient() {
    close(sock);
}

bool TCPClient::connectToServer() {
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed" << std::endl;
        return false;
    }
    return true;
}

void TCPClient::sendData(const CustomData& data) {
    send(sock, &data, sizeof(data), 0);
}

