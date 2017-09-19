
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

#include <unistd.h>
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

#define BUFLEN 272000  //Max length of buffer
#define PORT "30000"   //The port on which to listen for incoming data  12358
#define UDP_SERVER_IP "172.31.1.147"//define the udp server ip address  //"127.0.0.1" "172.31.1.147"/
#define MIN_DISTANCE 0.010 //define min point gap to be 2 mm
using boost::asio::ip::udp;

//enum { max_length = 1024 };


using namespace std;
using namespace cv;
vector< vector <Point2d> > vPtSignature;

boost::asio::io_service io_service;


udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
int samplingGap=2;
int maxPackageSize=1000;
//ros::Publisher pubTask;
//store the previous target points
std::vector<Point3d> position;
std::vector<Point3d> normals;
int data_size=0;
string endTag="@l@";

udp::resolver resolver(io_service);
udp::resolver::query query(udp::v4(), UDP_SERVER_IP , PORT);
udp::resolver::iterator iterator4 = resolver.resolve(query);

double differThreshold=0.05 ;
bool startPublishingFlag=false;

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



std::vector< string > fromString2ArrayStr(string inStr)
{
   std::vector< string > vd;
   string buf; // Have a buffer string
   stringstream ss(inStr); // Insert the string into a stream
   while (ss >> buf)
       vd.push_back((buf.c_str()));
   return vd;
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
// string constructSinglePointStr(geometry_msgs::Point32 pt)
// {
//   return double2str(pt.x)+","+double2str(pt.y)+","+double2str(pt.z);
// }
double roundD(double ind) //round d with only 3 digits
{
  double tempd=ind*10000;
  int intd=floor(tempd);
  double digitd=tempd-(double)intd;
  digitd=digitd/10000;
  return ind-digitd;
}


string constructSinglePointStr(cv::Point3d pt)
{
  return double2str(roundD(pt.x))+","+double2str(roundD(pt.y))+","+double2str(roundD(pt.z));
}

double getPointDistance(Point3d P1, Point3d P2)
{
  Point3d deltaP=P2-P1;
  return cv::norm(deltaP);
}

//PointNum should be larger than 2
vector<double> getPointDistanceInVector(vector<Point3d> inps)
{
  size_t pointNum=inps.size();
  vector<double> distance;
  for(int i=0;i<pointNum-1;i++)
  {
    Point3d thisPoint=inps[i];
    Point3d nextPoint=inps[i+1];
    double dis=getPointDistance(thisPoint, nextPoint);
    distance.push_back(dis);
  }
  return distance;
}



string constructPubStr(vector< vector <Point3d> > ps, vector< vector <Point3d> > ns)
{
  string rtStr="";
  size_t strokesNum=ps.size();

  //remove too close points
  vector< vector <Point3d> > rps;
  vector< vector <Point3d> > rns;
  rps=ps;
  rns=ns;
  // for(int i=0;i<strokesNum;i++)
  // {
  //   size_t pointsNum=ps[i].size();
  //   vector <Point3d> stps;
  //   vector <Point3d> stns;
  //   if(pointsNum>1)//minimum 2
  //   {
  //     vector<double> pointDists=getPointDistanceInVector(ps[i]);
  //     double d=pointDists[0];
  //     stps.push_back(ps[i][0]);
  //     stns.push_back(ns[i][0]);
  //     for(int j=0;j<pointsNum-1;j++)
  //     {
  //       if(d>=MIN_DISTANCE)
  //       {
  //         stps.push_back(ps[i][j+1]);
  //         stns.push_back(ns[i][j+1]);
  //         d=pointDists[j+1];
  //       }
  //       else
  //         d+=pointDists[j+1];
  //     }
  //   }
  //   else
  //   {
  //     stps=ps[i];
  //     stns=ns[i];
  //   }
  //   rps.push_back(stps);
  //   rns.push_back(stns);
  // }

  strokesNum = rps.size();
  for(int i=0;i<strokesNum;i++)
  {
     size_t pointsNum=rps[i].size();
     if(pointsNum>0)
     {
       string thisLineStr="";
       for(int j=0;j<pointsNum-1;j++)
          if(j%samplingGap==0)
            thisLineStr += constructSinglePointStr(rps[i][j])+","+constructSinglePointStr(rns[i][j])+",";
       thisLineStr+=constructSinglePointStr(rps[i][pointsNum-1])+","+constructSinglePointStr(rns[i][pointsNum-1]);
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




bool publishOnce=false;
void path_dataCallback(const normal_surface_calc::targetPoints::ConstPtr& msg)
{
//  double currentPositionSum=sumPositionPoints();
  //position.resize(msg->path_robot.size());
//  normals.resize(msg->path_robot.size());

position.clear();
normals.clear();
  string sendback_cmd;//="Seq_Points:";
  //std::cout<<"topic received!"<<endl;

try{
  path_checker pcheck(msg, vPtSignature);
//   path_checker pcheck(msg);

for (int i = 0; i < msg->path_robot.size(); i++) {
  position.push_back(Point3d(msg->path_robot[i].x, msg->path_robot[i].y,msg->path_robot[i].z));
  normals.push_back(Point3d(msg->normals_robot[i].x, msg->normals_robot[i].y,msg->normals_robot[i].z));//   msg->normals_robot[i]);
}

  vector< vector <Point3d> > ps=pcheck.getStrokesPathPositions();
  vector< vector <Point3d> > ns=pcheck.getStrokesNormalVects();

  data_size=msg->path_robot.size();

  //cout<<"path robot: "<<position<<endl;
  //ROS_INFO_STREAM("topic received! ");
  if(data_size>0)
      sendback_cmd=constructPubStr(ps, ns);
  }
  catch(exception &e) {
   std::cout << "Catch an exception: " << e.what() << std::endl;
}

  sendback_cmd +=endTag;
  sendback_cmd  = "seq_point:"+sendback_cmd;
//  double newPositionSum=sumPositionPoints();
  if(startPublishingFlag)
  {
    ROS_INFO_STREAM("new path drawing task detected!");

    ROS_INFO_STREAM("ConstructPubStr");
    //cout<<sendback_cmd<<endl;
    //ROS_INFO_STREAM("topic received! no NAN.");

  //  std::cout<<normals<<endl;
     try {
       //get robot position x,y,z
       int myArrayLength=sendback_cmd.size();
       char myArray[myArrayLength];//as 1 char space for null is also required
       strcpy(myArray, sendback_cmd.c_str());
       std::cout << "new path drawing task published!  "<<sendback_cmd<<endl;
       boost::asio::socket_base::send_buffer_size option(BUFLEN);//BUFLEN

       s.set_option(option);//set send out buffer size.
       s.get_option(option);
       int ssize = option.value();
       cout<<"size"<<ssize<<endl;
        cout<<"to send length:"<<myArrayLength*8*8<<endl;
        if(myArrayLength*8*8>=212992) //3328
           cout<<"warning: data length beyond UDP datagram."<<endl;
        //if data length is larger the UDP datagram size, split it into several packages.

        int packageNum=myArrayLength/maxPackageSize;
        if(packageNum==0)
            s.send_to(boost::asio::buffer(myArray, myArrayLength), *iterator4);
        else
        {
          for(int i=0;i<packageNum+1;i++)
          {
            //get different package
            int thisPackageSize=maxPackageSize;
            if(i==packageNum)
                thisPackageSize=myArrayLength-maxPackageSize*packageNum;
            char toSendArray[thisPackageSize];
            int startIndex=0+i*maxPackageSize;
            for(int m=0;m<thisPackageSize;m++)
                toSendArray[m]=myArray[startIndex+m];
            s.send_to(boost::asio::buffer(toSendArray, thisPackageSize), *iterator4);
            cout<<toSendArray<<'\0'<<endl;
            usleep(1000);
          }
        }
        std::cout<<"a new task has been assigned!"<<endl;
        startPublishingFlag=false;
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
        vector<vector<Point2d> >().swap(vPtSignature);
        for(int i=0;i<strokesNum;i++)
        {
          string thisStroke=strokeStrs[i];
          vector<double> points=fromString2Array(thisStroke);
          size_t pointNum=points.size()/2;
          vector<Point2d> strokePoints;
          for(int j=0;j<pointNum;j++)
              strokePoints.push_back(Point2d(points[2*j],points[2*j+1]));
          vPtSignature.push_back(strokePoints);
        }
        //published=false;
    }
}

void startCMD_data_callback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data=="start")
    {
       startPublishingFlag=true;
    }
}

//"[0.53196,1.39524,1.09715,0.955755,-4.0458,-1.292023,-0.000904]"
//"[0.1037,1.167,1.3004,1.27976,-4.7181,-1.54276,-0.0039043]"

int main(int argc, char **argv )
{
   if(argc>1)
      sscanf(argv[1],"%d",&samplingGap);
   if(argc>2)
      sscanf(argv[2],"%d",&maxPackageSize);
//sscanf(argv[2],"%d",&power);
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "path_drawer");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/targetPoints", 1, path_dataCallback);
    ros::Subscriber sub4 = nh.subscribe("/chris/strokes", 1000, signature_data_callback);
    ros::Subscriber sub5 = nh.subscribe("/chris/task_cmd", 1000, startCMD_data_callback);
    //ros::ServiceClient Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
    //cout << "Press any key to continue..." << endl;
    //getchar();
     // Joint_move_client.call(mv_srv);///////////////////////////////////////////////////

    // cout << "Press any key to continue the servoing loop..." << endl;
    ///  getchar();
    ros::Rate loop_rate(3);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool done=false;
    //cout<<"press esc to quit."<<endl;
    cout << "UDP client is trying to connect to ...."<< UDP_SERVER_IP<<":"<<PORT<< endl;
    int keyPress;
    while(ros::ok()&&!done)
    {
      ros::spinOnce();
    //   keyPress = cvWaitKey(1)&255;
    // //  cout<<(keyPress)<<endl;
    //   if(27 == keyPress){//"Esc"
    //      ROS_INFO_STREAM("quit app.");
    //      done=true;
    //      break;
    //    }
       try {

       }
      catch(exception &e) {
          std::cout << "Catch an exception: " << e.what() << std::endl;
       }
    }
}
