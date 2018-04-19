#include <stdio.h>  
#include <unistd.h>
#include<pthread.h>
#include <signal.h> // signal functions 
#include<string.h>
#include<stdlib.h>
FILE *fp[4];   
static void *print_ps_result(void *arg){
	FILE * fp =(FILE *)arg;
	char line[256]; 
	while(!feof(fp))  
		{  
			if(fgets(line, 256, fp) != NULL)  
				printf("%s", line);  
		}  
}
static void my_handler(int sig){ // can be called asynchronously 
  pclose(fp[1]);
  pclose(fp[3]);
  printf("quit!\n");
  exit(0);
} 
int main()  
    {  
	
	/*FILE *fp;
	fp = popen("source /home/chen/Project/catkin_lidar/devel/setup.bash","w");
	fprintf(fp, "ping 127.0.0.1\n");
	//fprintf(fp, "roslaunch loam_velodyne loam_velodyne.launch\n");
	//fprintf(fp, "plot tan(x)\n");

	pclose(fp);*/
	char line[256];
	pid_t pid;
	//memset(line,'\0',256); 
	
	pthread_t thread_id[2];

	//int i=5;
	//fp = popen("cd /home/chen/Project/catkin_lidar", "r"); 
	//fp = popen("source /home/chen/Project/catkin_lidar/devel/setup.bash", "r"); 
	// popen("gnome-terminal -x ping 127.0.0.1","r");
	// while(i>0){
	// 	popen("gnome-terminal -x /home/zbl/family_server/client/client","r");
	// 	popen("gnome-terminal -x /home/zbl/family_server/main","r");
	// 	i--;
	// }
	
	//fp = popen("ping 127.0.0.1", "r");
	//pclose(fp);  
	//sleep(3);
	signal(SIGINT, my_handler);


	while(1){
		//第一个程序
		memset(line,'\0',256);
		//fp=popen("ps -ef | grep -w /home/zbl/ROBOT_SERVER/TCP_SERVER/main| grep -v grep","r");
		fp[0]=popen("ps -ef | grep -w ../TCP_SERVER/main| grep -v grep","r");		
		fgets(line, 10, fp[0]);
		//printf("%s\n",line);
		if(*line==NULL)  
		{  
			printf("没有执行第一个!\n");
			//创建子进程
				// system("gnome-terminal -x /home/zbl/ROBOT_SERVER/TCP_SERVER/main");
				fp[1]=popen("../TCP_SERVER/main","r");
				int res = pthread_create(&thread_id[0],NULL,print_ps_result,fp[1]);
				if(res==0){
					printf("create tcp_info pthread successfully!\n");
				}
				else {
					printf("fail to create tcp_info pthread!\n");
				}	
				//system("gnome-terminal -x /home/ubuntu/tcp/main");
				sleep(1);  				  
			//return -1;  
		}  
		else{
			//printf("22222222222\n");
			//printf("第一个正在执行!\n");
			//print_ps_result(fp[1]);
			pclose(fp[0]);
		}

		//第二个程序
		memset(line,'\0',256);
		// fp=popen("ps -ef | grep -w /home/zbl/ROBOT_SERVER/WS_SERVER/main| grep -v grep","r");
		fp[2]=popen("ps -ef | grep -w ../WS_SERVER/main| grep -v grep","r");
		fgets(line, 10, fp[2]);
		//printf("%s\n",line);
		if(*line==NULL)  
		{  
			printf("没有执行第二个!\n");
			//pid = fork();
			//创建子进程
				//system("gnome-terminal -x /home/zbl/ROBOT_SERVER/WS_SERVER/src/main"); 
				fp[3]=popen("../WS_SERVER/main","r");
				if(!fp[3]){
					printf("启动第二个程序失败\n");
					continue;
				}
				int res = pthread_create(&thread_id[1],NULL,print_ps_result,fp[3]);
				if(res==0){
					printf("create ws_info pthread successfully!\n");
				}
				else {
					printf("fail to create ws_info pthread!\n");
				}		
				//system("gnome-terminal -x /home/ubuntu/ws/main");  					 	
			//return -1;  
		}  
		else{
			//printf("第二个正在执行!\n");
			//print_ps_result(fp[3]);
			pclose(fp[2]);
		}
	sleep(1);	 
	}
	return 0;  
    } 
