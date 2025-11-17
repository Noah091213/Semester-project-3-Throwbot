#include "Gripper.h"

int Gripper::clientSocket = socket(AF_INET, SOCK_STREAM, 0);

int Gripper::connectGripper(){
    // Set up server address structure
    sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(GRIPPER_PORT);

    // Convert IP address from text to binary
    if (inet_pton(AF_INET, GRIPPER_IP, &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address or address not supported!" << std::endl;
        close(clientSocket);
        return -1;
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Connection failed!" << std::endl;
        close(clientSocket);
        return -1;
    }

    std::cout << "Connected to server!" << std::endl;
    return 0;
}

// to close all connectins, it works if it isnt used but its best to use bye();
void Gripper::bye(){
    std::string msg = "bye()\n";
    send(clientSocket, msg.c_str(), (int)msg.size(), 0);
    close(clientSocket);
}


void Gripper::grip(){
    command("grip()");
}

void Gripper::relase(){
    command("relase()");
}

// sends a string and waits for gripper to send "FIN"
void Gripper::command(std::string command){

    // Send a message
    std::string msg = command + "\n";
    send(clientSocket, msg.c_str(), (int)msg.size(), 0);

    // waiting to get "FIN" from gripper
    while (true) {
        char buffer[512];
        int bytes = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        buffer[bytes] = '\0';
        std::string response = (buffer);
        while (true) {

            if (bytes <= 0) {
                std::cout << "Connection closed or error\n";
                break;
            }

            if (response.find("FIN") != std::string::npos) {
                std::cout << "Command execution finished.\n";
                break;  // ready for next command
            }
        }
    }

}
