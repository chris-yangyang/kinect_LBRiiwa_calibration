#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/robot/vpWireFrameSimulator.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <cstdlib>
#include <fstream>

//#include <conio.h>
#include <math.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <stdlib.h>
 #include <arpa/inet.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
 
#define BUFLEN 1024  //Max length of buffer
#define PORT 12358   //The port on which to listen for incoming data
#define UDP_SERVER_IP "172.31.1.148"//define the udp server ip address  //"127.0.0.1" "172.31.1.147"/
#define DofNum 2
using namespace std;

void die(char *s)
{
    perror(s);
    exit(1);
}

 std::vector<float> convert2Float(std::vector<double> v)
 {
   std::vector<float> t;
   for(int i=0;i<v.size();i++)
     t.push_back((float)v[i]);
   return t;
}

void printOutStdVector(std::vector<double> v)
{
  cout<<"print out v:"<<endl;
  for(int i=0;i<v.size();i++)
  {
    cout<<v.at(i)<<"  ";
  }
  cout<<endl;
}

bool packageValid(string inputStr)
{
   if(inputStr.find("l@") != std::string::npos)
     return true;
   else
     return false;
}

//"[0.53196,1.39524,1.09715,0.955755,-4.0458,-1.292023,-0.000904]"
//"[0.1037,1.167,1.3004,1.27976,-4.7181,-1.54276,-0.0039043]"

int main(int argc, char **argv )
{
    int clientSocket, portNum, nBytes, inNBytes;
    char buffer[BUFLEN];
    char in_buffer[BUFLEN];
    struct sockaddr_in serverAddr;
    socklen_t addr_size;

    /*Create UDP socket*/
    clientSocket = socket(PF_INET, SOCK_DGRAM, 0);

    /*Configure settings in address struct*/
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);//172.31.1.147
    memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);  

    /*Initialize size variable to be used later on*/
    addr_size = sizeof serverAddr;
    
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "chris_calibrator");
    ros::NodeHandle nh;
    //ros::Subscriber subP = nh.subscribe("/chris_tracker/currentPoint", 1000, currentPointCallback);
    //ros::Subscriber subPd = nh.subscribe("/chris_tracker/desiredPoint", 1000, desiredPointCallback);
    ros::Publisher pubTask = nh.advertise<std_msgs::String>("/chris_tracker/taskFinished", 1);
    
    //ros::ServiceClient Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
    cout << "Press any key to continue..." << endl;
    getchar();
     // Joint_move_client.call(mv_srv);///////////////////////////////////////////////////
     
    // cout << "Press any key to continue the servoing loop..." << endl;
    ///  getchar();
    ros::Rate loop_rate(3);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool done=false;
    cout << "UDP client is trying to connect to ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
    while(ros::ok()&&!done)
    {
      ros::spinOnce();

       try {
	
	  std::cout << "Enter message: ";
	  // char request[max_length];
	  std::cin.getline(buffer, BUFLEN);
	  string str(buffer);
	  std::cout << "you typed:  "+str<<endl;
	 
	  if(str.find("esc") != std::string::npos)
	  {   
	    done=true;
	    break;
	  }
	  
          nBytes = strlen(buffer);
    
	  /*Send message to server*/
	  sendto(clientSocket,buffer,nBytes,0,(struct sockaddr *)&serverAddr,addr_size);

	  /*Receive message from server*/
          inNBytes = recvfrom(clientSocket,in_buffer,nBytes,0,(struct sockaddr *)&serverAddr,&addr_size);
	  cout<<"start receiving msgs, inNBytes:"<<inNBytes<<endl;
          if(inNBytes>0)
	  {
	    char readfrom[inNBytes];
	    for(int i=0;i<inNBytes;i++)
	      readfrom[i]=in_buffer[i];
	    string inStr(readfrom);
	    std::cout << "msg received:  "+inStr<<endl;
	    if (packageValid(inStr)&&inStr.find("esc") != std::string::npos)
            {
               std::cout << "esc command received";
	       done=true;
            }
	    
	  }
// 	      
// 	    // cout << "Dig into:-->>" << endl;
// 	    //getchar();
// 	    //Joint_move_client.call(mv_srv3);
// 	      //ros::spinOnce();
// 	      
// 	      //loop_rate.sleep();
// 	      //waitKey(0);
	     boost::this_thread::sleep(boost::posix_time::milliseconds(200));
       }
      catch(vpException &e) {
          std::cout << "Catch an exception: " << e << std::endl;
       }
    }
}

