#include <iostream>

#include <string>

#include <vector>

#include <cstring>

#include <arpa/inet.h>

#include <sys/socket.h>

#include <unistd.h>


int man_auto_qu; //Value for disable/enable automation

// Function declarations
int connect_to_action_server();
void send_data(int , int , int, int );
void toggle_auto();
