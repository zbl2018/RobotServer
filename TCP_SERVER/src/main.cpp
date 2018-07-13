#include"../include/ev_tcpServer.h"
#include<iostream>
using namespace std;
int main(){
    //声明一个服务器对象（监听指定地址），并初始化数据库链接(数据库用户名、密码、数据库名称)
    ev_tcpServer Tcp_server("127.0.0.1","root","7259","car_control");
    //开始指定监听本机指定的端口号
    //ev_tcpServer::server_log("./log/server_log.txt",ios::app);//记录日志，追加的方式 
    cout<<"sysytem time :"<<ev_tcpServer::GetTime()<<endl;
    //运行监听程序，监听22222端口
    Tcp_server.start_server(/*"127.0.0.1"*/22222);
    
    return 1;

}