#include "Gripper.h" // Din headerfil
#include <iostream>
#include <cstring>   // For memset, strerror
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>  // For close
#include <string>

// ------------------------------------------------------------------
// OPRINDELIGE/NØDVENDIGE KONSTANTER (Antaget defineret i Gripper.h)
// ------------------------------------------------------------------
// Hvis de ikke er defineret i headeren, definer dem her:
#define GRIPPER_IP "192.168.1.20"
#define GRIPPER_PORT 1000
#define SOURCE_IP "192.168.1.202" // DIN WSL IP på samme subnet som griberen

// Initialiser socket som før
int Gripper::clientSocket = socket(AF_INET, SOCK_STREAM, 0);

// ------------------------------------------------------------------
// connectGripper() med bind() - LØSNINGEN
// ------------------------------------------------------------------
int Gripper::connectGripper(){

    std::cout << "[Gripper Info] Connecting to gripper server..." << std::endl;

    // ----------------------------------------------------
    // ➡️ TRIN 1: BIND SOCKET TIL KILDE-IP (NØDVENDIG ÆNDRING)
    // ----------------------------------------------------
    struct sockaddr_in sourceAddr;
    memset(&sourceAddr, 0, sizeof(sourceAddr));
    sourceAddr.sin_family = AF_INET;
    sourceAddr.sin_port = 0; // Lader systemet vælge en ledig port

    // Konverter kilde-IP fra tekst til binær form
    if (inet_pton(AF_INET, SOURCE_IP, &sourceAddr.sin_addr) <= 0) {
        std::cerr << "[Gripper Error] Invalid source IP address (" << SOURCE_IP << ")!" << std::endl;
        close(clientSocket);
        return -1;
    }

    // Bind socket'en til den specifikke kilde-IP
    if (bind(clientSocket, (struct sockaddr*)&sourceAddr, sizeof(sourceAddr)) < 0) {
        std::cerr << "[Gripper Error] Could not bind socket to source IP. Error: " << strerror(errno) << std::endl;
        close(clientSocket);
        return -1;
    }
    std::cout << "[Gripper Info] Socket bound to source IP: " << SOURCE_IP << std::endl;


    // ----------------------------------------------------
    // ➡️ TRIN 2: CONNECT TIL DESTINATIONEN (Oprindelig logik)
    // ----------------------------------------------------

    // Set up server address structure
    sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(GRIPPER_PORT);

    std::cout << "[Gripper Info] Server IP: " << GRIPPER_IP << ", Port: " << GRIPPER_PORT << std::endl;

    // Convert destination IP address from text to binary
    if (inet_pton(AF_INET, GRIPPER_IP, &serverAddr.sin_addr) <= 0) {
        std::cerr << "[Gripper Error] Invalid address or address not supported!" << std::endl;
        close(clientSocket);
        return -1;
    }

    std::cout << "[Gripper Info] Attempting to connect to server..." << std::endl;

    // Connect to the server. Bruger nu den bundne kilde-IP.
    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        // Tilføjet strerror(errno) for bedre fejlfinding
        std::cerr << "[Gripper Error] Connection failed! System error: " << strerror(errno) << std::endl; 
        close(clientSocket);
        return -1;
    }

    std::cout << "[Gripper Info] Connected to server!" << std::endl;

    home();

    return 0;
}
// ------------------------------------------------------------------
// RESTEN AF DIN KLASSE (Uændret)
// ------------------------------------------------------------------

// to close all connectins, it works if it isnt used but its best to use bye();
void Gripper::bye(){
    std::string msg = "bye()\n";
    send(clientSocket, msg.c_str(), (int)msg.size(), 0);
    close(clientSocket);
}


void Gripper::grip(){
    std::cout << "[Gripper Info] Gripping..." << std::endl;
    command("grip()");
}

void Gripper::release(){
    std::cout << "[Gripper Info] Releasing..." << std::endl;
    command("release()");
}

void Gripper::home(){
    std::cout << "[Gripper Info] Opening fully..." << std::endl;
    command("home()");
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

        if (bytes <= 0) {
            std::cout << "[Gripper Error] Connection closed or error" << std::endl;
            break;
        }

        if (response.find("FIN") != std::string::npos) {
            std::cout << "[Gripper Info] Movement complete" << std::endl;
            break;  // ready for next command
        }
        
    }

}