#ifndef GRIPPER_H
#define GRIPPER_H

// iostream and cstring is used

#include <iostream>
#include <unistd.h>     // For close()
#include <cstring>      // For memset()
#include <arpa/inet.h>  // For sockaddr_in, inet_pton()

#define GRIPPER_IP "192.168.1.20"
#define GRIPPER_PORT 1000

class Gripper {

public:
    int static connectGripper();
    void static bye();

    void static grip();
    void static relase();
private:
    void static command(std::string command);
    int static clientSocket;
};

#endif // MYCLASS_H
