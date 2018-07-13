#include"../include/ws_server.h"
int main()
{
    ws_server WS;
    //初始化与tcp服务器程序的链接
    WS.InitTcpClient_ConnectServer("127.0.0.1",22222);
    WS.init_ws_server();
    //Listen on port 20000
    WS.ws_run(20000);

    return 0;
}