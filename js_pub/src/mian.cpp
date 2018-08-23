#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <vector>
#define JNUMBER 6
using namespace std;

class tcp_client
{
private:
        int socket_fd;
        char message[4096];
        struct sockaddr_in server_addr;

public:
        tcp_client(char* server_ip,char* server_port);
        ros::NodeHandle n_;   
        ros::Publisher pub_;
        sensor_msgs::JointState msg;
        std::vector<float> pos; 
};

vector<string> split(const string& str, const string& delim) {
	vector<string> res;
	if("" == str) return res;
	//先将要切割的字符串从string类型转换为char*类型
	char * strs = new char[str.length() + 1] ; //不要忘了
	strcpy(strs, str.c_str());

	char * d = new char[delim.length() + 1];
	strcpy(d, delim.c_str());

	char *p = strtok(strs, d);
	while(p) {
		string s = p; //分割得到的字符串转换为string类型
		res.push_back(s); //存入结果数组
		p = strtok(NULL, d);
	}

	return res;
}





tcp_client::tcp_client(char* server_ip,char* server_port):pos(6,0.0)
{
        pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 1000);  
        ros::Rate loop_rate(10);
        msg.name.resize(JNUMBER);
        msg.name[0]="joint_1";
        msg.name[1]="joint_2";
        msg.name[2]="joint_3";
        msg.name[3]="joint_4";
        msg.name[4]="joint_5";
        msg.name[5]="joint_6";
        msg.position.resize(JNUMBER);

       if( (socket_fd = socket(AF_INET,SOCK_STREAM,0)) < 0 ) {
                printf("create socket error: %s(errno:%d)\n)",strerror(errno),errno);
                exit(0);
        }

        memset(&server_addr,0,sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(atoi(server_port));

        if( inet_pton(AF_INET,server_ip,&server_addr.sin_addr) <=0 ) {
                printf("inet_pton error for %s\n",server_ip);
                exit(0);
        }

        if( connect(socket_fd,(struct sockaddr*)&server_addr,sizeof(server_addr))<0) {
                printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
                exit(0);
        }

        printf("connect to server.\n");

        std::vector <string> list1;
        std::vector <string> list2;
        std::vector <string> list3;

        string position1[6];
        string position2[6];
        while(ros::ok()){
		if( recv(socket_fd,message,4096,0) < 0){
			printf("Received mssage error .\n");
				continue;
		}
		printf("Received message: %s\n",message);
		list1 = split(message,"\n");

		list2 = split(split(list1[0],"(")[1],",");
		for(int i = 0; i < JNUMBER; i++){
			position1[i] = list2[i];
			printf("position1[%d] = %s\n",i,position1[i].c_str());
                        msg.position[i]=atof(position1[i].c_str());
                        printf("pos[%d] is %f\n",i,msg.position[i]);
					}
                    
		list3 = split(split(list1[1],"(")[1],",");
		for(int i = 0; i < JNUMBER; i++){
			position2[i] = list3[i];
			printf("position2[%d] = %s\n",i,position2[i].c_str());
		}
                pub_.publish(msg);
                ros::spinOnce();

                loop_rate.sleep();
        }
        close(socket_fd);
        exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "j_state_pub");
     tcp_client tc("192.168.1.102","10000");
    std::string str="12.3";
    // std::cout<<str<<std::endl;
    // float pos=atof(str.c_str());
    // std::cout<<pos<<std::endl;
    return 0;
}
