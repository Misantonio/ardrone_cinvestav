//Socket Libraries
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <netdb.h>

//ROS Libraries
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <sstream>

using namespace std;

int main(int argc, char **argv){

    //Socket Variables
    int client;
    int portNum = 444;
    const char* ip = "192.168.43.81"; // IP of the pc publishing optitrack info
    int buffSize = 24;
    char buffer[buffSize];
    float num;
    int size_data;

    //ROS Variables
    ros::init(argc,argv,"talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("droneLeader",100);
    ros::Rate loop_rate(200);

    //Socket client
    sockaddr_in server_addr;

    client = socket(AF_INET, SOCK_STREAM, 0);

    if(client < 0){
        cout << "\nError establishing socket..." << endl;
        exit(1);
    }

    cout << "\n=> Socket has been created..." << endl;

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(portNum);
    server_addr.sin_addr.s_addr = inet_addr(ip);

    int connection = connect(client, (sockaddr *)&server_addr, sizeof(server_addr));
    cout << connection << endl;

    if(connection < 0){
        cout << "=> No connection established..." << endl;
        exit(1);
    }


    cout << "=> Connection to the server " << ip << " and the port number " << portNum << endl;

    int count = 0;

    do{
        std_msgs::Float32MultiArray arrayRos;
        arrayRos.data.clear();

        size_data = recv(client, buffer, buffSize, 0);
        if (size_data != 0){
            for(int i = 0; i < buffSize/4; i++){
                memcpy(&num,buffer+(i*4),sizeof(num));
                arrayRos.data.push_back(num);
                cout << count << " " << num;
            }
            cout << endl;

            chatter_pub.publish(arrayRos);

            ros::spinOnce();

            loop_rate.sleep();
            ++count;
        }

    } while((ros::ok()) && (size_data != 0));

    exit(0);
}
