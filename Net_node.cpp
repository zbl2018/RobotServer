#include"Net_node.h"
//===================静态变量初始化======================
    int Net_node::send_count = 0;
    int Net_node::stop_count = 0;
    double Net_node::cmode,Net_node::cangle,Net_node::cspeed;
    bool Net_node::is_stop = false;
    int Net_node::is_car_arrvie = 0;
    int Net_node::is_car_arriveCrossing = 0;
//=====================================================
void Net_node::init_member_variable(string log_index){
    //////////////////
    gps_status = -1;
    net_node_init = true;
    is_obstacle = 1;
    lat_now = 0;//x
    lon_now = 0;//y
    chargex = 0;
    chargey = 0.1;
    heading_now = 0;
    task_x  = 100,task_y = 100;
    last_task_x = 100,last_task_y = 100;
    net_speed = 0.2;
    is_recv,is_first = 0;
    length_read;
    //decoded from zbl on jan 21
    //判断拐点 1:出现拐点
    count_CrossInfo=0;
    charge_flag = false; //zqq 0203
    serial = "";
    path_log.open(log_index);
}
void sys_init(ros::NodeHandle nh){
    Task_net = nh.advertise<std_msgs::String>("/task_net",1000);//任务点数据发布
    Move_net = nh.advertise<std_msgs::String>("/move_net",1000);//后台遥控指令发布
    stop_net = nh.advertise<std_msgs::Int8>("stop_net",100);

    Charge_pub = nh.advertise<std_msgs::Int8>("charge", 5);

    Location_net = nh.subscribe<std_msgs::String>("/final_data",1000,LocationHandler);
    car_info_net = nh.subscribe<std_msgs::String>("/car_info",100,InfoHandler);//接收底层数据

    map_send = nh.advertise<std_msgs::String>("/map_send",1000);//map send test

    sub_gps_rtk = nh.subscribe<sensor_msgs::NavSatFix>("fix", 10,GPS_RTK_Handler);

    gps_status_net = nh.subscribe<std_msgs::Int64>("gps_status",10,GPS_STATUS_Handler);

    sub_car_arrive = nh.subscribe<std_msgs::Int8>("car_arrive",5,Arrive_Car_Handler);

    Obstacle_sub = nh.subscribe<std_msgs::Int8>("/obstacle_plan",1,Obstacle_plan_Handle);

    ////////////////////////
    //decoded from zbl on jan 21
    sub_car_cross = nh.subscribe<std_msgs::String>("chmod",5,Cross_Car_Handler);
    /////////////////////////////
}
void Net_node::InitTcpClient_ConnectServer(string ip_addr,int tcp_server_port){
    serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(tcp_server_port);
	serverAddr.sin_addr.s_addr = inet_addr(ip_addr.c_str());
	//链接远程主机
	reconnect_tcp_server(sockfd,serverAddr);
    //与tcp_server连接成功，创建接收线程
    int res = pthread_create(&recv_th_id,NULL,recvInfo_from_TcpServer,this);
    if(res==0){
        printf("create recv pthread successfully!\n");
    }
    else {
         printf("fail to create recv pthread!\n");
    }
    res = pthread_create(&send_th_id,NULL,sendData,this);
    if(res==0){
        printf("create recv pthread successfully!\n");
    }
    else {
         printf("fail to create recv pthread!\n");
    }
}
int Net_node::reconnect_tcp_server(int &clientSocket_fd, struct sockaddr_in serverAddr){
	int is_connect=-1;
	//创建句柄	
	if((clientSocket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket");
		return 1;
	}
	//链接	
	is_connect=connect(clientSocket_fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
	if(is_connect<0)
		{
			perror("connect");
			printf("wait for connect server...\n");
		}	
	while(is_connect<0){
		is_connect=connect(clientSocket_fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
	}
	printf("connect with destination host...\n");
	return 1;
}
void * Net_node::recvInfo_from_TcpServer(void* args){
        net_node *net_node_object = (net_node*)args;
		char data_buf[MAX_BUF_LEN];
		byte head_buf[HEAD_LEN];
		int ret_len;
        int data_len;
	while(1){
		ret_len = recv(net_node_object->sockfd,head_buf,HEAD_LEN,MSG_WAITALL);
        //cout<<"hex:"<<hex<<head_buf<<endl;
        if(ret_len==0||ret_len<0){
             printf("remote socket closed 1\n");
             close(net_node_object->sockfd);
             net_node_object->reconnect_tcp_server(net_node_object->sockfd,net_node_object->serverAddr);
        }
        else{
            data_len = bytesToInt(head_buf+HEAD_BACKUP_LEN,HEAD_DATA_LEN);//后四个字节为报文数据部分长度
        }
        cout<<"length:"<<data_len<<endl;
        //2.数据部分长度大于接收缓存的最大上限则主动关闭tcp链接
        if(data_len>MAX_BUF_LEN)
        {
            printf("buffer overflow \n");
            close(sockfd);
            net_node_object->reconnect_tcp_server(net_node_object->sockfd,net_node_object->serverAddr);
        }
        //3.获取数据部分
        ret_len = recv(net_node_object->sockfd,data_buf,data_len, MSG_WAITALL);
        if(ret_len>0){
            //转发
            printf("%d,recv message:\n'%s'\n",sockfd,data_buf);
            //解析收到的报文信息
            net_node_object->deal_info_from_server(string info);

        }
        else{
            printf("remote socket closed 2\n"); //客户端断开链接
            close(net_node_object->sockfd);
            net_node_object->reconnect_tcp_server(net_node_object->sockfd,net_node_object->serverAddr);
        }
	}
	
}
void* Net_node::sendData(void* args){
    net_node *net_node_object = (net_node*)args;
    while(true){
        net_node_object->sendCarStatus(net_node_object->sockfd);
        net_node_object->sendStatus(net_node_object->sockfd);
        usleep(500000);
    }
}

void Net_node::sendStatus(int sockfd){ 
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;
    map_data["action"] = "RosRealState";
    map_data["content"]["direction"]["carDirection"] = heading;
    map_data["content"]["direction"]["flTire"] = cangle1;
    map_data["content"]["direction"]["frTire"] = cangle2;
    map_data["content"]["direction"]["rlTire"] = cangle3;
    map_data["content"]["direction"]["rrTire"] = cangle4;
    map_data["content"]["model"] = cmode;
    map_data["content"]["speed"]["carSpeed"] = cspeed;
    map_data["content"]["speed"]["flSpeed"] = cspeed_m1;
    map_data["content"]["speed"]["frSpeed"] = cspeed_m2;
    map_data["content"]["speed"]["rlSpeed"] = cspeed_m3;
    map_data["content"]["speed"]["rrSpeed"] = cspeed_m4;
    map_data["content"]["x"] = lat_now;
    map_data["content"]["y"] = lon_now;
    map_data["serial"] = "12345678";
    string jstr = writer.write(map_data);
    SendInfoToServer(jstr,"sendStatus");
}
void Net_node::sendCarStatus(int sockfd){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;
    string powerTem;
    string powertmp;
    for(int i=0;i<12;i++)
    {
        if(i == 0)
        {
            powerTem.clear();
            powertmp.clear();
            powertmp = boost::lexical_cast<string>(ccell[0]);
            powerTem.append(powertmp);
        }
        else
        {
            powertmp.clear();
            powertmp = boost::lexical_cast<string>(ccell[i]);
            powerTem.append(",").append(powertmp);
        }
    }
    //int cSOC2 = boost::lexical_cast<int>(cSOC);
    map_data["action"] = "RosState";
    map_data["content"]["boxTemp"] = cenvirTem;
    map_data["content"]["infrared"] = 1;
    map_data["content"]["pavement"] = is_obstacle;
    map_data["content"]["power"] = cSOC;
    map_data["content"]["powerNode"] = powerTem;
    map_data["content"]["powerTemp"] = cmaxTem;
    map_data["content"]["stray"] = 1;
    map_data["content"]["ultrasonic"] = cobstacle;
    map_data["content"]["gpsStatus"] = gps_status;
    map_data["serial"] = "0986754";
    string jstr = writer.write(map_data); 
    if(cSOC > 5.0){
    	SendInfoToServer(jstr,"sendCarStatus");
    }
}
void Net_node::SendChargeMessage(string pathplan){
    std_msgs::Int8 charge_msg;
    charge_msg.data = 1;
    Charge_pub.publish(charge_msg);

    std_msgs::String charge_msg2;
    string tmp = (boost::lexical_cast<string>(chargex));
    tmp.append(" ").append(boost::lexical_cast<string>(chargey));
    tmp.append(" ").append(boost::lexical_cast<string>("-0.2"));
    charge_msg2.data = tmp;
    Task_net.publish(charge_msg2);

    tmp = pathplan;
    charge_msg2.data = tmp;
    map_send.publish(charge_msg2);

}

///////////////////////////////////////
//decoded from zbl on jan 21
//send CrossInfo to server
PointS Net_node::search_cross_point_from_node2(double x,double y,vector<PointS> map,int &count){
//   double lat;
//  double lon;
//  int id;//start in 1
    vector<PointS>::iterator it;
    double min=99999;//初始较大
    double temp=0;
    int location=0;
    //查找一个与cross_x，cross_y相差最小的拐点
    for(it=map.begin(); it!=map.end(); it++){
        cout<<"node2:"<<it->lat<<" "<<it->lon<<endl;
       temp=fabs(x-it->lat)+fabs(y-it->lon);
      //temp=fabs((x+y)-(it->lat+it->lon));
    if(temp<min){
      min=temp;
      location=it-map.begin();
    }
   // cout<<*it<<" " ;
  }
  //cout<<"result of searching :"<<(map.begin()+location)->lat<<endl;
  count=location+1;
  return *(map.begin()+location);
}
//decoded from zbl on jan 21
//send CrossInfo to server
void Net_node::send_cross_info(int count){
    //构建json消息格式
    Json::FastWriter writer;
    Json::Value CrossInfo;
   // int len;
    //char finInfo[1480];
    PointS crossPoint;
    double x=boost::lexical_cast<double>(cross_x);
    double y=boost::lexical_cast<double>(cross_y);
    //在node2中获取与当前接收到的拐点的位置cross_x,cross_y，最接近的拐点
    crossPoint=search_cross_point_from_node2(x,y,cross_node2,count);
    cout<<"search node2 successfully!"<<endl;
    CrossInfo["action"]="arriveCrossing";
    CrossInfo["serial"]="123456";
    //boost::lexical_cast
    CrossInfo["x"]=crossPoint.lat;
    CrossInfo["y"]=crossPoint.lon;
    CrossInfo["no"]=count;
    string jstr =writer.write(CrossInfo);
    SendInfoToServer(jstr,"send_cross_info");
}

void Net_node::sendTask_Arrived(){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;
    if(charge_flag)
    {
        map_data["action"] = "charge";
        map_data["result"] = "0000";
        map_data["serial"] = serial;
        map_data["msg"] = "charge";
        charge_flag = false;
    }
    else{
        map_data["action"] = "patrolArriveNode";
        map_data["result"] = "0000";
        map_data["serial"] = serial;
        map_data["msg"] = "patrolArriveNode";
    }
    string jstr = writer.write(map_data);
    SendInfoToServer(jstr,"sendTask_Arrived");
}
void Net_node::send_Stop(){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;
    map_data["action"] = "stop";
    map_data["result"] = "0000";
    map_data["serial"] = serial;
    map_data["msg"] = "stop";

    string jstr = writer.write(map_data);
    //cout<<"josn:"<<jstr<<endl;
    SendInfoToServer(jstr,"send_Stop");
    is_stop = false;
}
void Net_node::sendPlanner(const char* s){
    Json::Value root;
    Json::FastWriter writer;
    Json::Value map_data;
    Json::Reader reader;
    Json::Value value;
    cout<<"plan s:"<<s<<endl;
    if(reader.parse(s,value)){
        map_data["action"] = value["action"].asString();
        map_data["result"] = "0000";
        map_data["serial"] = serial;
        map_data["msg"] = value["action"].asString();
        //int node_size = value["mapNodeList"].size();
        int node_size = node.size();
        int node2_size = node2.size();
        int tag = 0;
        vector<int> task_visited;
        task_visited.resize(node_size);
        task_visited[0] = -1;
        for(int i = 1;i<node_size;i++)
        {
        	task_visited[i] = 1;
        }
        cout<<"node1: "<<node_size<<"  node2: "<<node2_size<<endl;
        for(int i=0;i<node2_size;i++)
        {
            for(int j=1;j<node.size();j++)
            {
                //if(length_two_points(node2[i].lat, node2[i].lon, node[j].lat, node[j].lon) <=0.3)
                //if(( node2[i].lat - node[j].lat < 0.00001 ) && ( node2[i].lon - node[j].lon < 0.00001 ) )
                if(( fabs(node2[i].lat - node[j].lat) < 0.1 ) && ( fabs(node2[i].lon - node[j].lon) < 0.1 ) && (task_visited[j] == 1) )
                {
                    tag = 1;
                    map_data["mapNodeList"][i]["id"] = value["mapNodeList"][j-1]["id"].asString();
                    map_data["mapNodeList"][i]["no"] = i+1;
                    map_data["mapNodeList"][i]["x"] = node2[i].lat;
                    map_data["mapNodeList"][i]["y"] = node2[i].lon;
                    task_visited[j] = -1;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
                    break;
                }
            }
            if(tag == 0)
            {
                map_data["mapNodeList"][i]["no"] = i+1;
                map_data["mapNodeList"][i]["x"] = node2[i].lat;
                map_data["mapNodeList"][i]["y"] = node2[i].lon; 
            }
            tag = 0;
        }
    }
    string jstr = writer.write(map_data);
    //cout<<"plan josn:"<<jstr<<endl;
    path_log<<"plan josn:"<<jstr<<endl;
    path_log<<endl<<endl<<endl;
    SendInfoToServer(jstr,"sendPlanner");
}
int Net_node::deal_info_from_server(string info){
    //解析json数据
    if(RES_SUCC == my_json->decodejson(info)){
        switch(my_json->action_type){//遥控指令
            case  MOVE_ORD:{
                    decodeMove(info);
                    break;
            }
            case DeployRosParameter_ORD :{//获取机器人坐标方位信息
                    cout<<"deployRosParameter"<<endl;
                    Json::Value root;
                    Json::FastWriter writer;
                    Json::Value map_data;
                    map_data["action"] = "deployRosParameter";
                    map_data["result"] = "0000";
                    map_data["serial"] = serial;
                    map_data["msg"] = "deployRosParameter";
                    map_data["x"] = lat_now;
                    map_data["y"] = lon_now;
                    map_data["orientation"] = heading_now;
                    string jstr = writer.write(map_data);
                    SendInfoToServer(jstr,"DeployRosParameter_ORD");
                    break;

            }
            case PatrolStartPathPlan_ORD :{//开始路径规划
                    send_count = 0;
                    //is_obstacle = 1;//dyy obstacle reset
                    cout<<"plan road "<<is_car_arrvie<<" "<<send_count<<endl;                    
                    node.clear();
                    node2.clear();
                    pathPlan pa;
                    Planner(sss);
                    path_log<<"planner: "<<sss<<endl;
                    path_log<<"node: ";
                    for(int i=0;i<node.size();i++)
                    {
                        path_log<<setprecision(12)<<"("<<node[i].lat<<","<<node[i].lon<<");  ";
                    }
                    path_log<<endl;
                    if(is_obstacle)
                    {
                        node2 = pa.calculate(node);
                    }else
                    {
                        node2 = pa.obstacles(node,heading_now);
                    }
                    cross_node2=node2;
                    path_log<<"node2: ";
                    for(int i=0;i<node2.size();i++)
                    {
                        path_log<<setprecision(12)<<"("<<node2[i].lat<<","<<node2[i].lon<<");  ";
                    }
                    path_log<<endl;
                    sendPlanner(sss);
                    break;
            }
            case PatrolArriveNode_ORD:{//接收任务点并自动巡检
                    send_count = 0;
                    is_car_arrvie = 0;
                    stop_count = 0;
                    cout<<"patrolArriveNode "<<is_car_arrvie<<" "<<send_count<<endl<<endl;
                    decodeTask(sss);
                    decodeStop(0);
                    break;
            }
            case STOP_ORD:{//暂停小车
                    decodeStop(1);
                    is_stop = true;
                    stop_count = 0;
                    break;
            }
            case CHARGE_ORD :{
                    send_count = 0;
                    is_car_arrvie = 0;
                    stop_count = 0;
                    vector<PointS> ChargeNode;
                    PointS ChargeTask;
                    ChargeTask.lat = -0.213751;
                    ChargeTask.lon = 2.645041;
                    ChargeNode.push_back(ChargeTask);
                    ChargeTask.lat = chargex;
                    ChargeTask.lon = chargey;
                    ChargeNode.push_back(ChargeTask);
                    charge_flag = true;  //zqq
                    pathPlan pa;
                    vector<PointS> ChargeResult = pa.calculate(ChargeNode);
                    SendChargeMessage(pa.pathplan);
                    break;
            }
        }
    }
    else{
        printf("the order that is from server is uncertain!\n");
    }

 }
double Net_node::length_two_points(double x, double y, double xx, double yy){
    double x_xx = x - xx;
    double y_yy = y - yy;
    return sqrt(x_xx * x_xx + y_yy * y_yy);
}
void delay_sec(int sec)
{  
    time_t start_time, cur_time;  
    time(&start_time);  
    do  
    {  
        time(&cur_time);  
    } while((cur_time - start_time) < sec);  
}
void Net_node::ReturnLocalPos(double lat,double lon,double& localx,double& localy){
    ifstream infile_feat("/home/chen/testdata/srk_1018/taskmap.txt");
    vector<GPSPoints> all_pos;
    GPSPoints gps;
    LocalGeographicCS lcs;
    double mlat,mlon;
    lcs.ll2xy(lat,lon,mlat,mlon);

    while(infile_feat>>setprecision(12)>>gps.lat>>gps.lon>>gps.x>>gps.y){
        double tmp_x,tmp_y;
        lcs.ll2xy(gps.lat,gps.lon,tmp_x,tmp_y);
        gps.lat = tmp_x;
        gps.lon = tmp_y;
        all_pos.push_back(gps);
    }
    infile_feat.close();
    ////////////////////////////////
    double mindis = 1000;
    int min_index = 0;
    for(int i = 0; i < all_pos.size(); i++){
        double dis = length_two_points(mlat,mlon,all_pos[i].lat,all_pos[i].lon);
        if(mindis > dis){
            mindis = dis;
            min_index = i;
        }
    }
    localx = all_pos[min_index].x;
    localy = all_pos[min_index].y;
}

String Net_node::decodejson(char* s){ //decode the json from net by shenrks
    Json::Reader reader;
    Json::Value value;
    string js;
    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            js = value["action"].asString();
            serial = value["serial"].asString();
        }
    }
    return js;
}
String Net_node::decodejson_serial(char* s){ //decode the json from net by shenrks
    Json::Reader reader;
    Json::Value value;
    string js;
    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            js = value["serial"].asString();
        }
    }
    return js;
}

void Net_node::decodeMove(char* s){
    Json::Reader reader;
    Json::Value value;
    string js;
    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            //cout<<value["action"].asString()<<endl;
            time_t timep;  
            time(&timep);

            std_msgs::String msg;
            string msgs = value["mode"].asString().append(" ").append(value["angle"].asString()).append(" ").append(value["speed"].asString());
            msg.data = msgs;
            Move_net.publish(msg);
            cout<<msgs<<endl;
            printf("%s", asctime(gmtime(&timep))); 
        }
    }
}
void Net_node::decodeTask(char* s){
    Json::Reader reader;
    Json::Value value;
    if(reader.parse(s,value)){
        if(!value["action"].isNull()){
            //cout<<value["action"].asString()<<endl;
            std_msgs::String msg;

            cout<<value["x"].asString()<<" "<<value["y"].asString()<<endl;

            string msgs = value["mapNode"]["x"].asString().append(" ").append(value["mapNode"]["y"].asString()).append(" ").append(value["mapNode"]["speed"].asString());

            task_x = boost::lexical_cast<double>(value["mapNode"]["x"].asString());
            task_y = boost::lexical_cast<double>(value["mapNode"]["y"].asString());
            //net_speed = boost::lexical_cast<double>(value["mapNode"]["speed"].asString());//get the speed from the server

            serial = value["serial"].asString();

            string speed_rev = value["mapNode"]["speed"].asString();
            if(speed_rev.compare("") == 0)
            {
                net_speed = 0.2;
            }
            else
            {
                net_speed = boost::lexical_cast<double>(value["mapNode"]["speed"].asString());
            }

            msg.data = msgs;
            Task_net.publish(msg);
            cout<<msgs<<endl;


            //////////road plan//////////
            tasknode.clear();
            tasknode2.clear();
            
            PointS newpoint;//获取当前坐标加入规划
            if(net_node_init == true){
                newpoint.lat = 0;
                newpoint.lon = 0;
                tasknode.push_back(newpoint);

                net_node_init = false;
            }else{
                newpoint.lat = lat_now;
                newpoint.lon = lon_now;
                tasknode.push_back(newpoint);
            }

            newpoint.lat = task_x;
            newpoint.lon = task_y;
            tasknode.push_back(newpoint);

            pathPlan pa_tmp;
            if(is_obstacle)
            {
                tasknode2 = pa_tmp.calculate(tasknode);
            }else
            {
                tasknode2 = pa_tmp.obstacles(tasknode,heading_now);
                int cot = 5;
                while(cot--)
                {
                    cout << "obstacle replan!!!!!!!!!!!!!!" << endl;
                }
                is_obstacle = 1;
            }
            string path_send = pa_tmp.pathplan;//规划完成的导航路径
            //std_msgs::String msg;
            msg.data = path_send;
            map_send.publish(msg);

            /////////////////////////////
        }
    }
}

void Net_node::decodeStop(int isstop){//发送stop指令
    std_msgs::Int8 msg;
    msg.data = isstop;
    stop_net.publish(msg);
}

void Net_node::LocationHandler(const std_msgs::String::ConstPtr &msg){
    
    string moves = msg->data;
    stringstream stringin(moves);
    stringstream stringin2(moves);
    stringin>>lat>>lon>>heading;

    stringin2>>setprecision(12)>>lat_now>>lon_now>>heading_now;
   // cout<<"net location: "<<lat_now<<" "<<lon_now<<endl;
    //heading = heading * 180.0 / 3.14159265;
}   


//decoded from zbl on jan 21
//deal with subscription of "chmod" 
void Net_node::Cross_Car_Handler(const std_msgs::String::ConstPtr &msg){
    string temp;//临时存储字符流中的mode部分
    stringstream datastream(msg->data);
    datastream>>cross_x>>cross_y>>cross_h>>temp;
    //将mode转换为int类型，下面用于判断是否为1：处于拐点位置
    is_car_arriveCrossing =boost::lexical_cast<int>(temp);
   // cout<<"zbl:"<<cross_x<<" "<<cross_y<<" "<<cross_h<<" "<<temp<<" "<<is_car_arriveCrossing<<endl;
}

void Net_node::Arrive_Car_Handler(const std_msgs::Int8::ConstPtr &msg){
	is_car_arrvie = msg->data;
	//cout<<"receive  arrived data from lidar_ctrl: "<<msg->data<<endl; 
	//cout<<"the car have arrived ang get the msg "<<is_car_arrvie<<endl;
}

void Net_node::GPS_RTK_Handler(const sensor_msgs::NavSatFixConstPtr& gpslast){
    last_lat = gpslast->latitude;
    last_lon = gpslast->longitude;
    last_height = gpslast->altitude;
}

void Net_node::GPS_STATUS_Handler(const std_msgs::Int64::ConstPtr &msg){
    gps_status = msg->data;
}

void Net_node::InfoHandler(const std_msgs::String::ConstPtr &msg){
  string moves = msg->data;
  stringstream stringin(moves);
  //cout<<"mother fucker: "<<moves<<endl;
  //stringin>>o1>>o2>>o3>>o4>>a1>>a2>>a3>>a4>>sm1>>sm2>>sm3>>sm4;
  stringin>>cmode>>cangle>>cspeed;
  stringin>>cangle1>>cangle2>>cangle3>>cangle4;
  stringin>>ccur_m1>>ccur_m2>>ccur_m3>>ccur_m4>>cspeed_m1>>cspeed_m2>>cspeed_m3>>cspeed_m4;
  stringin>>cpul_m1>>cpul_m2>>cpul_m3>>cpul_m4;

  //cout<<cpul_m1<<" "<<cpul_m2<<" "<<cpul_m3<<" "<<cpul_m4<<endl;

  stringin>>cus1>>cus2>>cus3>>cus4>>cus5>>cus6>>cus7>>cus8>>cobstacle>>cdis_keep;
  stringin>>ccell[0]>>ccell[1]>>ccell[2]>>ccell[3]>>ccell[4]>>ccell[5]>>ccell[6]>>ccell[7]>>ccell[8]>>ccell[9]>>ccell[10]>>ccell[11];
  stringin>>cmaxV>>cminV>>cmaxVP>>cminVP>>cVD>>cAV>>cTV>>cCC>>cDC>>cSOC;
  stringin>>cTem1>>cTem2>>cTem3>>cTem4>>cTem5>>cTem6>>cmaxTem>>cminTem>>cavrTem>>cenvirTem;
  stringin>>cRc>>cWr;
  //cout<<"soc: "<<cSOC<<" "
} 

void Net_node::Obstacle_plan_Handle(const std_msgs::Int8::ConstPtr &msg){
    is_obstacle = msg->data;
}//dyq obstacle

void Net_node::Planner(char* s){
	is_car_arrvie = 0;
    Json::Reader reader;
    Json::Value value;
    string id,x,y,z;
    stringstream in_s;
    string s_,r;
    int is_one = 0;
    // lat_now = 5.1203;//2.8689
    // lon_now = 2.8689;//5.1203 
    PointS newpoint;//获取当前坐标加入规划   zqq 
    if(net_node_init == true){
        newpoint.lat = 0;
        newpoint.lon = 0;
        node.push_back(newpoint);
        //net_node_init = false;
    }else{
        newpoint.lat = lat_now;
        newpoint.lon = lon_now;
        node.push_back(newpoint);
    }   
    cout<<"lat_now: "<<lat_now<<" lon_now: "<<lon_now<<endl;  
    if(reader.parse(s,value)){
        int node_size = value["mapNodeList"].size();
        cout<<"node size:"<<node_size<<endl;
        for(int i=0;i<node_size;i++){
            id = value["mapNodeList"][i]["id"].asString();
            x = value["mapNodeList"][i]["x"].asString();
            y = value["mapNodeList"][i]["y"].asString();
            z = value["mapNodeList"][i]["z"].asString();
            if(is_one == 0){
                r.append(id);
                is_one = 1;
            }else{
                r.append(" ").append(id);
            }
            r.append(" ").append(x);
            r.append(" ").append(y);
            r.append(" ").append(z);
            double lat_ = strtod(x.c_str(),NULL);
            double lon_ = strtod(y.c_str(),NULL);
            newpoint.lat = lat_;
            newpoint.lon = lon_;
            node.push_back(newpoint);           
        }       
        //cout<<"task node:"<<r<<endl;
    }
}

byte* Net_node::intToBytes(int value,int byte_len){
    if(byte_len>4||byte_len<1)
    {
       cout<<"byte_len overflow!"<<endl;
        return 0;
    }
    byte *des = new byte[byte_len]; 
    // for(int i = 0; i < byte_len; i++)
    // {
    //     des[i] = (byte)((value >> ((byte_len - i - 1) * 8)) & 0xFF);
    // }
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
int Net_node::bytesToInt(byte* des, int byte_len){
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
string Net_node::GetTime(){
        time_t t = time( 0 );   
        char tmpBuf[255];   
        strftime(tmpBuf, 255, "%Y-%m-%d %H:%M:%S", localtime(&t)); //format date and time.
        return tmpBuf; 
}

void Net_node::SendInfoToServer(string info,string tip){
    int len = info.length();
    byte head_buf[HEAD_LEN];
    char send_str[len+HEAD_LEN];
    //创建头部
    memcpy(head_buf+HEAD_BACKUP_LEN,intToBytes(len,HEAD_DATA_LEN),HEAD_DATA_LEN);
    //整合报文
    memcpy(send_str,head_buf,HEAD_LEN);
    memcpy(send_str+HEAD_LEN,info,len);
    //发送报文，忽略异常
    if(send(sockfd,send_str,len+HEAD_LEN,MSG_NOSIGNAL)<=0){
        printf("% fail \n",tip);
        perror("send");
    }
    else{
        printf("%s:%s\n",tip,info);
       // cout<<"send_cross_info:"<<info<<endl;
    }
}

int main(int argc, char **argv){
    bool status = ros::ok();
    int count_tmp = 0;
    ros::init(argc, argv, "Net_test");
    ros::NodeHandle nh;
    Net_node my_net_node;

    my_net_node.sys_init(nh);
    my_net_node.init_member_variable("/home/intel/Project/catkin_lidar/src/loam_velodyne/output/plan_log.txt")
    my_net_node.InitTcpClient_ConnectServer("193.112.128.66",22222);
    ros::Rate rate(50);
   
    while(status){
        ros::spinOnce();
        if(is_stop == true && cspeed == 0 && stop_count < 1){
            cout<<"car have stop."<<endl;
            stop_count++;
            my_net_node.send_Stop();
            cout<<"car have stop. send"<<endl;
            continue;
        }     
        /////////////////////////////////
        //decoded from zbl on jan 21
        //小车到达拐点,向服务后台发送数据
        if(is_car_arriveCrossing == 1)
        {
            //每次计数发送的拐点信息个数
            my_net_node.send_cross_info(count_CrossInfo);
            is_car_arriveCrossing=0;
            continue;
        }
        if(((is_car_arrvie == 1) && (cspeed < 0.01))){//length_two_points(task_x,task_y,lat_now,lon_now) < 0.5
            if(send_count < 1 && cmode == 0){

            	if(((is_car_arrvie == 1) && (cspeed < 0.01)))
                {
	        		cout<<"send is_car_arrvie to net"<<endl<<endl;
	        	}
	        	if((length_two_points(task_x,task_y,lat_now,lon_now) < 0.4))
                {
	        		cout<<"length of two point less 0.2"<<endl<<endl;
	        	}
            	cout<<"car have arrive the task point"<<endl;
	            send_count = 2;   
	            my_net_node.sendTask_Arrived();
	            cout<<"car have arrive the task point send "<<is_car_arrvie<<" "<<send_count<<endl;
	            last_task_x = task_x;
	            last_task_y = task_y;
	            task_x = 100;
	            task_y = 100;
            }
            continue;
        }   
        if(is_first == 0){
            string regist_str = "{\"action\":\"Regist\",\"deviceId\":\"ROS\"}";
            my_net_node.SendInfoToServer(regist_str,"regist_str");
        }
        status = ros::ok();
        rate.sleep();
    }
    my_net_node.path_log.close();
    return 0;  
}