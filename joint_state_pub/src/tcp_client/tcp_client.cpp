#include <iostream>
#include <string.h>
#include <vector>
#include <stdlib.h>
#include "tcp_client.h"

using namespace std;

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

tcp_client::tcp_client(char* server_ip,char* server_port)
{
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
        while(1){
					if( recv(socket_fd,message,4096,0) < 0){
						printf("Received mssage error .\n");
						continue;
					}
					printf("Received message: %s\n",message);
					list1 = split(message,"\n");

					list2 = split(split(list1[0],"(")[1],",");
					for(int i = 0; i < 6; i++){
						position1[i] = list2[i];
						printf("position1[%d] = %s\n",i,position1[i].c_str());
					}
					list3 = split(split(list1[1],"(")[1],",");
					for(int i = 0; i < 6; i++){
						position2[i] = list3[i];
						printf("position2[%d] = %s\n",i,position2[i].c_str());
					}
        }

        close(socket_fd);
        exit(0);
}

int main(int argc,char* argv[])
{
        tcp_client tc("192.168.2.85","10000");
		std::cout<<"sucess!"<<std::endl;
        return 0;
}
