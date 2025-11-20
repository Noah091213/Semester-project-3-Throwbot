#ifndef GRIPPER_H
#define GRIPPER_H

#include <iostream>
#include <unistd.h>     // For close()
#include <cstring>      // For memset()
#include <arpa/inet.h>  // For sockaddr_in, inet_pton()
#include <netinet/in.h> // For AF_INET, SOCK_STREAM, htons (anbefales)

// -------------------------------------------------------------------
// VIGTIGT: KONSTANTER
// -------------------------------------------------------------------

// Destination (Griberen)
#define GRIPPER_IP "192.168.1.20"
#define GRIPPER_PORT 1000

// Kilde-IP (Din WSL IP på griberens subnet) - NØDVENDIG ÆNDRING
#define SOURCE_IP "192.168.1.202"

class Gripper {

public:
    // connectGripper er nu den eneste funktion, der kræver den nye logik
    int static connectGripper();
    void static bye();

    void static grip();
    void static release();
    void static home();
private:
    void static command(std::string command);
    int static clientSocket;
};

#endif // GRIPPER_H