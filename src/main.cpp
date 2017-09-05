
#include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Point.h>
// #include <image_transport/image_transport.h>
#include "std_msgs/String.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"

// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float64MultiArray.h"
#include "normal_surface_calc/targetPoints.h"

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <exception>
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
 #include <boost/asio.hpp>
#include <geometry_msgs/Point32.h>
#include "path_checker.h"
#include <opencv2/opencv.hpp>

#define BUFLEN 1024  //Max length of buffer
#define PORT "12358"   //The port on which to listen for incoming data
#define UDP_SERVER_IP "172.31.1.147"//define the udp server ip address  //"127.0.0.1" "172.31.1.147"/

using boost::asio::ip::udp;

enum { max_length = 1024 };


using namespace std;
using namespace cv;
vector< vector <Point2d> > vPtSignature;

boost::asio::io_service io_service;

udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
//ros::Publisher pubTask;
//store the previous target points
std::vector<Point3d> position;
std::vector<Point3d> normals;
int data_size=0;
string endTag="@l@";

udp::resolver resolver(io_service);
udp::resolver::query query(udp::v4(), UDP_SERVER_IP , PORT);
udp::resolver::iterator iterator4 = resolver.resolve(query);

double differThreshold=0.05;

void die(char *s)
{
    perror(s);
    exit(1);
}

string double2str(double d)
{
  std::ostringstream d2str;
  d2str<<d;
  return d2str.str();
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
  //cout<<"print out v:"<<endl;
  for(int i=0;i<v.size();i++)
  {
    cout<<v.at(i)<<"  ";
  }
  cout<<endl;
}

bool packageValid(string inputStr)
{
   if(inputStr.find("@l@") != std::string::npos)
     return true;
   else
     return false;
}

 std::vector< double > fromString2Array(string inStr)
{
    std::vector< double > vd;
    string buf; // Have a buffer string
    stringstream ss(inStr); // Insert the string into a stream
    while (ss >> buf)
        vd.push_back(atof(buf.c_str()));
    return vd;
}

std::vector< string > fromString2ArrayStr(string inStr)
{
   std::vector< string > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back((buf.c_str()));
   return vd;
}

// string constructSinglePointStr(geometry_msgs::Point32 pt)
// {
//   return double2str(pt.x)+","+double2str(pt.y)+","+double2str(pt.z);
// }

string constructSinglePointStr(cv::Point3d pt)
{
  return double2str(pt.x)+","+double2str(pt.y)+","+double2str(pt.z);
}

string constructPubStr(vector< vector <Point3d> > ps, vector< vector <Point3d> > ns)
{
  string rtStr="";
  size_t strokesNum=ps.size();
  for(int i=0;i<strokesNum;i++)
  {
     size_t pointsNum=ps[i].size();
     if(pointsNum>0)
     {
       string thisLineStr="";
       for(int j=0;j<pointsNum-1;j++)
         thisLineStr += constructSinglePointStr(ps[i][j])+","+constructSinglePointStr(ns[i][j])+",";
       thisLineStr+=constructSinglePointStr(ps[i][pointsNum-1])+","+constructSinglePointStr(ns[i][pointsNum-1]);
       if(i==strokesNum-1)
         rtStr+=thisLineStr;
       else
         rtStr+=thisLineStr+";";
     }
  }
  return rtStr;
}

double sumPositionPoints()
{
  double rtd=0;
  size_t pointNumber=position.size();
  for(int i=0;i<pointNumber;i++)
    rtd +=position[i].x+position[i].y+position[i].z;
  return rtd;
}

bool checkNAN(const normal_surface_calc::targetPoints::ConstPtr& msg)
{
  bool rtb=false;
  for (int i = 0; i < msg->path_robot.size(); i++) {
    if(isnan(msg->path_robot[i].x)||isnan(msg->path_robot[i].y)||isnan(msg->path_robot[i].z))
        rtb=true;
    if(isnan(msg->normals_robot[i].x)||isnan(msg->normals_robot[i].y)||isnan(msg->normals_robot[i].z))
        rtb=true;
  }
  return rtb;
}

std::vector<string> split(const std::string &s, char delim) {
    std::vector<string> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(number);
    }
    return elems;
}

/**1 2 3 4 5 splitor: space
* input string 1 2 3 4 5 6
output array [1,2,3,4,5,6]
*/
std::vector< double > fromString2Array(string inStr)
{
   std::vector< double > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back(atof(buf.c_str()));
   return vd;
}



void path_dataCallback(const normal_surface_calc::targetPoints::ConstPtr& msg)
{
  double currentPositionSum=sumPositionPoints();
  position.resize(msg->path_robot.size());
  normals.resize(msg->path_robot.size());
  string sendback_cmd="Seq_Points:";
 // std::cout<<"topic received!"<<endl;
  path_checker pcheck(msg, vPtSignature);
  position=pcheck.getPathPositions();
  normals=pcheck.getNormalVects();
  vector< vector <Point3d> > ps=pcheck.getStrokesPathPositions();
  vector< vector <Point3d> > ns=pcheck.getStrokesNormalVects();
  data_size=position.size();
  if(data_size>0)
  {
    // for (int i = 0; i < msg->path_robot.size(); i++) {
    //     if(i==data_size-1)
    //       sendback_cmd +=constructSinglePointStr(position[i])+","+constructSinglePointStr(normals[i]);
    //     else
    //       sendback_cmd +=constructSinglePointStr(position[i])+","+constructSinglePointStr(normals[i])+",";
        sendback_cmd=constructPubStr(ps, ns);
    }
    //std::cout<<"topic received! no NAN."<<endl;
  }

  sendback_cmd+=endTag;
  double newPositionSum=sumPositionPoints();
  if(abs(currentPositionSum-newPositionSum)>differThreshold)
  {
    std::cout<<"new path drawing task detected!"<<endl;
  //  std::cout<<normals<<endl;
     try {
       //get robot position x,y,z
       int myArrayLength=sendback_cmd.size();
       char myArray[myArrayLength];//as 1 char space for null is also required
       strcpy(myArray, sendback_cmd.c_str());
       std::cout << "new path drawing task published!  "<<sendback_cmd<<endl;
       s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator4);
       std::cout<<"a new task has been assigned!"<<endl;
     }
     catch(exception &e) {
      std::cout << "Catch an exception: " << e.what() << std::endl;
   }
 }
}

void signature_data_callback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data=="reset")
    {
       vPtSignature.clear();
       return ;
    }
    vector<string> strokeStrs=split(msg->data, ';');//get different strokes.
    size_t strokesNum=strokeStrs.size();
    if(strokesNum>0)
    {
        vPtSignature.clear();
        vector<vector<Point> >().swap(vPtSignature);
        for(int i=0;i<strokesNum;i++)
        {
          string thisStroke=strokeStrs[i];
          vector<double> points=fromString2Array(thisStroke);
          size_t pointNum=points.size()/2;
          vector<Point> strokePoints;
          for(int j=0;j<pointNum;j++)
              strokePoints.push_back(Point2d(points[2*j],points[2*j+1]));
          vPtSignature.push_back(strokePoints);
        }
        //published=false;
    }
}


//"[0.53196,1.39524,1.09715,0.955755,-4.0458,-1.292023,-0.000904]"
//"[0.1037,1.167,1.3004,1.27976,-4.7181,-1.54276,-0.0039043]"

int main(int argc, char **argv )
{
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "path_drawer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/targetPoints", 1, path_dataCallback);
    ros::Subscriber sub4 = nh.subscribe("/chris/strokes", 1000, signature_data_callback);
    //ros::ServiceClient Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
    //cout << "Press any key to continue..." << endl;
    //getchar();
     // Joint_move_client.call(mv_srv);///////////////////////////////////////////////////

    // cout << "Press any key to continue the servoing loop..." << endl;
    ///  getchar();
    ros::Rate loop_rate(3);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool done=false;
    cout << "UDP client is trying to connect to ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
    while(ros::ok()&&!done)
    {
      ros::spinOnce();
      keyPress = cvWaitKey(1)&255;
    //  cout<<(keyPress)<<endl;
      if(27 == keyPress){//"Esc"
         done=true;
         break;
       }
       try {

       }
      catch(exception &e) {
          std::cout << "Catch an exception: " << e.what() << std::endl;
       }
    }
}
