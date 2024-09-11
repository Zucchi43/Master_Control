// Server side C/C++ program to send data through Socket
#include "vehicle_interface_SOCKET.h"

int man_auto_qu = 0;
int connect_to_action_server() {
    // Socket setup
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8080);  

    server_addr.sin_addr.s_addr = INADDR_ANY;

    // Connect to the server
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("connect");
        return -1;
    }

    return sockfd;
}

void send_data(int sockfd, int speed, int steer, int brake) {
    //pwm_update(speed_cmd_val, steer_cmd_val, brake_cmd_val);
    //Create a vector with the values
    std::vector<int> arr = {speed, brake, steer, man_auto_qu};//ACEL,FREIO, DIR, MAN_AUTO
    // Pack the data
    char data[16];
    memcpy(data, &arr[0], sizeof(int) * arr.size());

    // Send the data
    if (send(sockfd, data, sizeof(data), 0) < 0) {
        perror("send");
    }
}

void toggle_auto() {
    man_auto_qu ^= 1;
}


