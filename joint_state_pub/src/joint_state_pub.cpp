#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <sstream>
#include <vector>
#include "robot.h"
#include "robot_client.h"
#define JNUMBER 6
std::vector<float> pos(JNUMBER,0.0);

class SubscribeAndPublish  
{  
public:  
  SubscribeAndPublish(std::string address): robot(address),speed(5),pos_vector{0,90,90,0,-90,0},count(0)
  {  
    robot.start();
    printf("have execute the start function!\n");
    sleep(20);    
    msg.name.resize(JNUMBER);
    msg.name[0]="joint_1";
    msg.name[1]="joint_2";
    msg.name[2]="joint_3";
    msg.name[3]="joint_4";
    msg.name[4]="joint_5";
    msg.name[5]="joint_6";
    msg.position.resize(JNUMBER);
    
    //Topic you want to publish  
    pub_ = n_.advertise<sensor_msgs::JointState>("/joint_states", 1000);  

    //Topic1 you want to subscribe  
    sub_ = n_.subscribe("chatter1", 1000, &SubscribeAndPublish::callback1, this); 
    //Topic2 you want to subscribe  
    sub2_ = n_.subscribe("chatter2", 1000, &SubscribeAndPublish::callback2, this);  
  }  

  void callback1(const std_msgs::String::ConstPtr& msg1)  
  {  
    ++count;
//    robot.getPose(jvalue);
    for(int i=0;i<JNUMBER;i++)
    {
       msg.position[i]=jvalue[i];
    }
    pub_.publish(msg);
    printf("count is %d\n",count);
  }  
  void callback2(const std_msgs::String::ConstPtr& msg2)
  {
    pos_vector[1]+=0.1;
    robot.jointMove(pos_vector,speed);
  }

private:  
  ros::NodeHandle n_;   
  ros::Publisher pub_;  
  ros::Subscriber sub_;
  ros::Subscriber sub2_; 
  std_msgs::String output;
  sensor_msgs::JointState msg;
  int count; 
  Robot robot;
  float speed;
  std::vector<float> pos_vector;
  std::vector<float> jvalue;

};//End of class SubscribeAndPublish  


void updateOringinPos(const sensor_msgs::JointState& msg)
{
  for(int i=0;i<JNUMBER;i++)
  pos[i]=msg.position[i];
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joint_states_pub");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("count", 1000);
 
  int count=0;
  ros::Rate loop_rate(125);
  Robot robot("192.168.1.102:50051");
  
  pos[0]=34.507;
  pos[1]=145.062;
  pos[2]=-56.326;
  pos[3]=164.070;
  pos[4]=-137.357;
  pos[5]=103.926;
  
  robot.start();

 // ros::Subscriber sub=n.subscribe("/joint_states", 1000, updateOringinPos);
 int reply=robot.ServoJEnable(true);
 std::cout<<"servoj enable "<<reply<<std::endl;
  
  // SubscribeAndPublish test("192.168.2.60:50051");  
  // ros::MultiThreadedSpinner s(2);  //多线程
  // ros::spin(s);  
 while (ros::ok())
  {
   std_msgs::String msg;
   pos[1]-=0.06;
  // ++count;
   if(pos[1]>0)
   {
     std::stringstream ss;
     ss << "hello world " << count;
     msg.data = ss.str();
     chatter_pub.publish(msg);
     robot.servoJMove(pos,8);
     ros::spinOnce();
        
     loop_rate.sleep();
   }
   else
    break;
  }
  printf("sleeping!\n");
  sleep(30);
  printf("enable shut down!\n");
  
 reply=robot.ServoJEnable(false);
 std::cout<<"servoj enable "<<reply<<std::endl;
//  robot.stop();
 return 0;
}
