/*socket tcp客户端*/
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include"./include/json/json.h"
#include"./include/AES.h"
#include <exception>
#include <netinet/tcp.h>
#define SERVER_PORT 22222
#define byte unsigned char
AES my_aes;
int clientSocket;
struct sockaddr_in serverAddr;
char sendbuf[200];
const char * servInetAddr = "193.112.128.66";   
static pthread_rwlock_t rwlock;//读写锁对象
int cat=0;

int connect_remote_server(int &clientSocket, struct sockaddr_in serverAddr){
	int is_connect=-1;	
	//创建句柄
	
	if((clientSocket= socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket");
		return 1;
	}

	// struct sockaddr_in mine,dest;
    // bzero(&mine,sizeof(mine));
    // mine.sin_family = AF_INET;
    // mine.sin_port = htons(22223);
    // inet_pton(AF_INET,"127.0.0.1",&mine.sin_addr);

    serverAddr.sin_family = AF_INET; //ipv4协议簇  
    serverAddr.sin_port = htons(SERVER_PORT);//端口 
    //地址的表达格式由ASCII字符串转为二进制数值，servInetAddr->serv_addr.sin_addr  
    inet_pton(AF_INET, servInetAddr, &serverAddr.sin_addr);    
    //bzero(&(serverAddr.sin_zero), 8);
	//链接
	//int b = bind(clientSocket,(struct sockaddr*)&mine,sizeof(mine));
	is_connect=connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
	if(is_connect<0)
		{
			perror("connect");
			printf("wait for connect server...\n");
		}	
	while(is_connect<0){
		is_connect=connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
	}
	printf("connect with destination host...\n");
	return 1;
}
byte* intToBytes(int value,int byte_len){
    if(byte_len>4||byte_len<1)
    {
       cout<<"byte_len overflow!"<<endl;
        return 0;           
    }
    byte *des = new byte[byte_len]; 
    switch(byte_len){
            case 4:{ 
                //四位
                des[3] = (byte) (value & 0xff);
                des[2] = (byte) ((value >> 8) & 0xff);
                des[1] = (byte) ((value >> 16) & 0xff);
                des[0] = (byte) ((value >> 24) & 0xff); 
                break;
            }    
            case 3:{
                //三位
                des[2] = (byte) (value & 0xff);
                des[1] = (byte) ((value >>8) & 0xff);
                des[0] = (byte) ((value >>16) & 0xff); 
                break;
            }
        case 2:{
                //二位
                des[1] = (byte) (value& 0xff);
                des[0] = (byte) ((value >>8) & 0xff); 
                break;   
        }
        case 1:{
                //一位
                des[0] = (byte) (value & 0xff);
                break; 
        }      
        }  
        return des;  
}
int bytesToInt(byte* des, int byte_len){
        if(byte_len>4||byte_len<1){
        cout<<"byte_len overflow!"<<endl;
        return 0;           
        }  
        int value;    
        switch(byte_len){
            case 4:{ 
                //四位
                value = (int)((des[3] & 0xff)  
                | ((des[2] & 0xff) << 8)  
                | ((des[1] & 0xff) << 16)  
                | (des[0] & 0xff) << 24);
                break;
            }    
            case 3:{
                //三位
                value = (int) ((des[2] & 0xff)  
                | ((des[1] & 0xff) << 8)  
                | ((des[0] & 0xff) << 16));
                break;
            }
        case 2:{
                //二位
                value = (int) ((des[1] & 0xff)  
                | ((des[0] & 0xff) << 8));
                break;   
        }
        case 1:{
                //一位
                value = (int) ((des[0] & 0xff));
                break; 
        }      
        }       
        return value;  
    }
// void * read_function(void* args){
// 	int i =0;
// 	while(1){
// 		pthread_rwlock_wrlock(&rwlock);//获取写入锁
// 		cout<<"write:"<<cat<<endl;
// 		cat=i;
// 		i++;
		
// 		pthread_rwlock_unlock(&rwlock);
// 		sleep(2);
// 	}
// }
void * recv_function(void* args){
		char recvbuf[10000];
		unsigned char head[6];
		int iDataNum;
		string cipherHex;
	while(1){
		printf("111111\n");
		iDataNum = recv(clientSocket,head,6,MSG_WAITALL);
		int data_len = bytesToInt(head+2,4);
		iDataNum = recv(clientSocket,recvbuf,data_len ,MSG_WAITALL);
		printf("length:%d\n",data_len);
		printf("iDataNum:%d\n",iDataNum);
		printf("iDataNum:%s\n",recvbuf);
		if(iDataNum==-1||iDataNum==0)
		{
			printf("oooo2\n");
			close(clientSocket);
			//shutdown(clientSocket,SHUT_RDWR);
			//sleep(5);
			connect_remote_server(clientSocket,serverAddr);
			continue;
		}		
		//printf("链接断开3\n");
		//recvbuf[iDataNum] = '\0';
		cipherHex =my_aes.decrypt(recvbuf);
		cout<<"cipher : "<<cipherHex<<endl; 
	}
	
}
int main()
{
	// AES my_aes;
	// my_aes.initKV("1234567890123456","1234567890123456");  
	int res;
    pthread_t a_thread,b_thread,c_thread,d_thread;
    void *thread_result;
    res=pthread_rwlock_init(&rwlock,NULL);//初始化读写锁
	byte *head=new byte[6];

	// pthread_rwlock_rdlock(&rwlock);//获取读取锁	
	// pthread_rwlock_wrlock(&rwlock);//获取写入锁
	// pthread_rwlock_unlock(&rwlock);




    string text = "hello zbl !";  
    cout<<"text : "<<text<<endl;  
    my_aes.initKV("1234567890123456","1234567890123456");  
    string cipherHex =my_aes.encrypt(text);  
    cout<<"cipher : "<<cipherHex<<endl;  
    //writeCipher(cipherHex); 
    my_aes.initKV("1234567890123456","1234567890123456");
    cipherHex =my_aes.decrypt(cipherHex);
    cout<<"cipher : "<<cipherHex<<endl; 
	//sockaddr_in localAddr;  
    	//bzero(&localAddr, sizeof(localAddr));  
    	//localAddr.sin_family = AF_INET;  
    	//localAddr.sin_addr.s_addr = htonl(INADDR_ANY);  
    	//localAddr.sin_port = htons(10000);  
    	//bind(clientSocket, (const sockaddr*) &localAddr, sizeof(localAddr));
 
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(SERVER_PORT);
	serverAddr.sin_addr.s_addr = inet_addr("193.112.128.66");
	//链接远程主机
	connect_remote_server(clientSocket,serverAddr);
	pthread_t recv_th_id;
	
	int ret = pthread_create(&recv_th_id,NULL,recv_function,NULL);
	//int ret = pthread_create(&recv_th_id,NULL,read_function,NULL);
	// if(ret<0){
	// 	cout<<"successful!"<<endl;
	// }
	// else cout<<"fail"<<endl;
	// cout<<"cat:"<<endl;
	// while(1){
	// 	pthread_rwlock_rdlock(&rwlock);//获取读取锁	
	// 	cout<<"cat:"<<cat<<endl;
	// 	pthread_rwlock_unlock(&rwlock);
	// 	// if(cat==99)
	// 	// break;
	// }
	// printf("connect with destination host...\n");

	// string strValue = "{\"key1\":\"value1\",\"array\":[{\"key2\":\"value2\"},{\"key2\":\"value3\"},{\"key2\":\"value4\"}]}";
	// cout<<"11"<<strValue<<endl;
	Json::Value root,map_data;
	Json::FastWriter writer;
	root["key"]=1111;
	root["name"]="zbl";
	// map_data["action"] = "RosRealState";
    // map_data["content"]["direction"]["carDirection"] = heading;
    // map_data["content"]["direction"]["flTire"] = cangle1;
    // map_data["content"]["direction"]["frTire"] = cangle2;
    // map_data["content"]["direction"]["rlTire"] = cangle3;
    // map_data["content"]["direction"]["rrTire"] = cangle4;
    // map_data["content"]["model"] = cmode;
    // map_data["content"]["speed"]["carSpeed"] = cspeed;
    // map_data["content"]["speed"]["flSpeed"] = cspeed_m1;
    // map_data["content"]["speed"]["frSpeed"] = cspeed_m2;
    // map_data["content"]["speed"]["rlSpeed"] = cspeed_m3;
    // map_data["content"]["speed"]["rrSpeed"] = cspeed_m4;
    // map_data["content"]["x"] = lat_now;
    // map_data["content"]["y"] = lon_now;
    // map_data["serial"] = "12345678";







	map_data["action"] = "patrolStartPathPlan";
	map_data["result"] = "0000";
	map_data["serial"] = "1234";
	map_data["msg"] ="patrolStartPathPlan";
	map_data["mapNodeList"][2]["id"] = "1";
	map_data["mapNodeList"][2]["no"] = 1;
	map_data["mapNodeList"][2]["x"] = 0;
	map_data["mapNodeList"][2]["y"] = 0;

	map_data["mapNodeList"][1]["id"] = "2";
	map_data["mapNodeList"][1]["no"] = 2;
	map_data["mapNodeList"][1]["x"] = 9;
	map_data["mapNodeList"][1]["y"] = 9;





	string strValue= writer.write(map_data);//root.toStyledString();
	cout<<"11"<<strValue<<endl;
	printf("value:%s\n",strValue);
	strcpy(sendbuf,strValue.c_str());
	// strcpy(sendbuf,my_aes.encrypt(strValue).c_str());
	// printf("decryption:%s\n",my_aes.decrypt(sendbuf));
	printf("value:%s\n",strValue);
	//signal(SIGPIPE,SIG_IGN);
	

	while(1)
	{
		// printf("Input your world:>");
		// scanf("%s", sendbuf);
		// strcpy(sendbuf,my_aes.encrypt(sendbuf).c_str());
		// printf("\n");
		//cout<<"length:"<<strlen(sendbuf)+1<<endl;
		memcpy(head+2,intToBytes(strlen(sendbuf)+1,4),4);
		cout<<"len:"<<bytesToInt(head+2,4)<<endl;
		//cout<<"head:"<< hex <<head<<endl;
		head[0]=NULL;
		head[1]=NULL;
		//cout<<"headlengrh:"<<bytesToInt(head,2);
		char info[1000];
		memcpy(info,head,6);
		memcpy(info+6,sendbuf,strlen(sendbuf)+1);
		int temp=send(clientSocket, info, strlen(sendbuf)+7, MSG_NOSIGNAL);
		//cout<<sendbuf<<endl;
		if(temp<0)
		{
			printf("oooo\n");
			//close(clientSocket);
			sleep(5);			
			//connect_remote_server(clientSocket,serverAddr);
			continue;
		}
		printf("temp:%d\n",temp);
			//return 0;
		// if(strcmp(sendbuf, "quit") == 0)
		// 	break;
		// recvbuf[iDataNum] = '\0';
		// string temp="this is a test!";
		// temp=my_aes.decrypt(my_aes.encrypt(temp));
		// printf("recv data of my world is: %s\n", temp);
		//memset(recvbuf,'0',200);
		sleep(1);
	}
	close(clientSocket);
	return 0;
}
