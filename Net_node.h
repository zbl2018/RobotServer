#include <sys/types.h>   
#include <sys/socket.h>   
#include <stdio.h>   
#include <stdlib.h>   
#include <string>   
#include <cstring>
#include <sys/ioctl.h>   
#include <unistd.h>   
#include <netdb.h>   
#include <netinet/in.h>     
#include <arpa/inet.h>     
#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <string>      
#include <ros/ros.h> 
#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include <json/json.h>
#include <pthread.h>
#include <sstream>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>
#include <cmath>
#include <vector>  
#include <set> 
#include <list>
#include <algorithm>
#include <dirent.h>  
#include <pthread.h>
#include <loam_velodyne/pathPlan.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
//#include <loam_velodyne/GaussProjection.hpp>
#include <loam_velodyne/LocalGeographicCS.hpp>
#include"MyJson.h"
using namespace std;
#define byte unsigned char
#define HEAD_LEN 6 
#define MAX_BUF_LEN 1024
#define HEAD_BACKUP_LEN 2
#define HEAD_DATA_LEN 4  
struct Pointll
{
    double lat;
    double lon;
};
struct PointGS{
    double x;
    double y;
};
//#define PORT 22222
class Net_node{
    public:
            //init_function
            void init_member_variable(string log_index);
            void InitTcpClient_ConnectServer(string ip_addr,int tcp_server_port);
            void sys_init(ros::NodeHandle nh);
            //used in InitTcpClient_ConnectServer
            int reconnect_tcp_server(int &clientSocket, struct sockaddr_in serverAddr);
            
            //used in the function of thread
            static void* sendData(void* args)
            void sendStatus(int sockfd);
            void sendCarStatus(int sockfd);
            static void *recvInfo_from_TcpServer(void* args);

            //the recall functions of ros node
            void LocationHandler(const std_msgs::String::ConstPtr &msg);
            void Cross_Car_Handler(const std_msgs::String::ConstPtr &msg);
            void Arrive_Car_Handler(const std_msgs::Int8::ConstPtr &msg);
            void GPS_RTK_Handler(const sensor_msgs::NavSatFixConstPtr& gpslast);
            void GPS_STATUS_Handler(const std_msgs::Int64::ConstPtr &msg);
            void InfoHandler(const std_msgs::String::ConstPtr &msg);
            void Obstacle_plan_Handle(const std_msgs::Int8::ConstPtr &msg);

            //the function used send() to send information to tcp_server
            void send_cross_info(int count);
            void sendTask_Arrived();
            void send_Stop();
            void sendPlanner(const char* s);

            //public message to other node 
            void SendChargeMessage(string pathplan);

            //TOOL 
            int deal_info_from_server(string info);
            static  double length_two_points(double x, double y, double xx, double yy); 
            void delay_sec(int sec);               
            void ReturnLocalPos(double lat,double lon,double& localx,double& localy);
            String decodejson(char* s);
            String decodejson_serial(char* s);
            void decodeMove(char* s);
            void decodeTask(char* s);
            void decodeStop(int isstop);
            void Planner(char* s);
            PointS search_cross_point_from_node2(double x,double y,vector<PointS> map,int &count);
            static byte* intToBytes(int value,int byte_len);
            static int bytesToInt(byte* des, int byte_len);
            string GetTime();
            void SendInfoToServer(string info,string tip);
    protected:


    private:
            MyJson my_json;
            pthread_t recv_th_id;
            pthread_t send_th_id;
            //////////////////
            static int send_count ;
            static int stop_count ;
            static double cmode,cangle,cspeed;
            static bool is_stop;
            static int is_car_arrvie;
            static int is_car_arriveCrossing;
            //////////////////
            int gps_status;
            double last_lat,last_lon,last_height;
            bool net_node_init = true;
            
            double cangle1,cangle2,cangle3,cangle4;
            double ccur_m1,ccur_m2,ccur_m3,ccur_m4,cspeed_m1,cspeed_m2,cspeed_m3,cspeed_m4;
            double cpul_m1,cpul_m2,cpul_m3,cpul_m4;
            double cus1,cus2,cus3,cus4,cus5,cus6,cus7,cus8,cobstacle,cdis_keep;
            double ccell[12];
            double cmaxV,cminV,cmaxVP,cminVP,cVD,cAV,cTV,cCC,cDC,cSOC;
            double cTem1,cTem2,cTem3,cTem4,cTem5,cTem6,cmaxTem,cminTem,cavrTem,cenvirTem;
            double cRc,cWr;
            
            int is_obstacle;
            ros::Publisher Planer_net;
            ros::Publisher Recv_net;
            ros::Publisher Task_net;
            ros::Publisher Move_net;
            ros::Publisher stop_net;
            ros::Publisher map_send;
            ros::Publisher Charge_pub;
            ros::Subscriber Location_net;
            ros::Subscriber car_info_net;
            ros::Subscriber gps_status_net;
            ros::Subscriber sub_car_arrive;
            ros::Subscriber sub_gps_rtk;
            ros::Subscriber Obstacle_sub;
            string lat,lon,heading;
            // double lat_now = 5.1203;//x
            // double lon_now = 2.8689;//y
            double lat_now;//x
            double lon_now;//y
            double chargex;
            double chargey;
            // lat_now = 5.1203;//2.8689
            // lon_now = 2.8689;//5.1203
            double heading_now;
            double task_x ,task_y =;
            double last_task_x,last_task_y;
            double net_speed;
            int is_recv,is_first;
            int length_read;
            
            ///////////////////////////////
            //decoded from zbl on jan 21
            //判断拐点 1:出现拐点
            
            //订阅函数：订阅chmod话题：包含出现拐点信息
            ros::Subscriber sub_car_cross;
            //临时暂存拐点坐标
            string cross_x,cross_y,cross_h;
            //拐点计数
            int count_CrossInfo;
            bool charge_flag ; //zqq 0203
            string servInetAddr;  
            int sockfd;
            struct sockaddr_in serv_addr;
            string plan_s;
            string  serial;
            vector<PointS> node;
            vector<PointS> node2;
            vector<PointS>cross_node2;
            vector<PointS> tasknode;
            vector<PointS> tasknode2;
            ofstream path_log;
            ///////////////////////////////
}