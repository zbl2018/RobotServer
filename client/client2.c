#include <stdio.h>  
#include <string.h>  
#include <stdlib.h>  
#include <string.h>  
#include <unistd.h>  
#include <fcntl.h>  
#include <sys/types.h>  
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <iostream>
#include<fstream>
#define byte unsigned char
using namespace std;  
#define    MAXLINE        999999  
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
int main(int argc,char **argv)  
{  
    struct sockaddr_in     serv_addr;  
    char                   buf[MAXLINE];  
    int                    sock_id;  
    int                    read_len;  
    int                    send_len;  
    FILE                   *fp;  
    int                    i_ret;  
     
    if (argc != 4) 
    {  
        printf("usage :%s ipaddr portnum filename\n", argv[0]);  
        exit(0); 
    }  
    std::ifstream fin(argv[3], std::ios::binary);  
    fin.seekg(0, ios::end);  
    int iSize = fin.tellg();  
    cout<<"iSIZE:"<<iSize<<endl;
    char* szBuf = new (std::nothrow) char[iSize+6];  
  
    fin.seekg(0, ios::beg);  
    fin.read(szBuf+6, sizeof(char) * iSize);  
    fin.close();




    // if ((fp = fopen(argv[3],"rb")) == NULL) 
    // {  
    //     perror("Open file failed\n");  
    //     exit(0);  
    // }  
      
/*<-----------------------------------------socket---------------------------------------------->*/
    if ((sock_id = socket(AF_INET,SOCK_STREAM,0)) < 0) {  
        perror("Create socket failed\n");  
        exit(0);  
    }  
/*<-----------------------------------------connect---------------------------------------------->*/
    memset(&serv_addr, 0, sizeof(serv_addr));  
    serv_addr.sin_family = AF_INET;  
    serv_addr.sin_port = htons(atoi(argv[2]));  
    inet_pton(AF_INET, argv[1], &serv_addr.sin_addr);  
     
    i_ret = connect(sock_id, (struct sockaddr *)&serv_addr, sizeof(struct sockaddr));  
    if (-1 == i_ret) 
    {  
        printf("Connect socket failed\n");  
       return -1;  
    }  
/*<-------------------------------------client send part---------------------------------->*/ 
    bzero(buf, MAXLINE);  
    // while ((read_len = fread(buf+6, sizeof(unsigned char), MAXLINE, fp)) >0 ) 
    // {  
        memcpy(szBuf+2,intToBytes(iSize,4),4);
        memcpy(szBuf,intToBytes(99,2),2);
        send_len = send(sock_id, szBuf, iSize+6, 0); 
        cout<<"send_len:"<<send_len<<endl; 
        if ( send_len < 0 ) 
        {  
            perror("Send file failed\n");  
            exit(0);  
        }  
       // bzero(buf, MAXLINE);  
    // }  
  
    // fclose(fp);  
    close(sock_id);  
    printf("Send Finish\n");  
    return 0;  
} 
