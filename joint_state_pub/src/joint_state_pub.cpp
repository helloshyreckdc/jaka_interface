#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <boost/thread.hpp>
#include <sstream>
#include <vector>
#include "robot.h"
#include "robot_client.h"
#include <cstdio>
#include "cubicSpline.h"
#define JNUMBER 6
#define AXIS_COUNT 6

using namespace std;

vector<vector<double>> all_points;
vector<float> pos(JNUMBER,0.0);

void chatterCB(const trajectory_msgs::JointTrajectory& msg)
{
		cubicSpline spline;
		double time_nsecs=0.0;
		int points_count=0;
		double joint_positions[6] {0.0};
		double right_bound=0;
		ROS_INFO("get the joint_path_command ");
		ofstream originFile;
		originFile.open("origin2.csv",ios::out | ios::trunc);
		originFile<<"time"<<","<<"pos2"<<","<<"vel2"<<","<<"ace2"<<","<<endl;
		std::basic_string<char> jointname=msg.joint_names[0] ;
		points_count= msg.points.size();                        //获取规划后点的个数
		double *Time=new double[points_count];              //划分数组
		double *data_arr=new double[points_count];
		for (int j=0; j < AXIS_COUNT; j++)
		{
				for (int i{}; i < points_count; i++)
				{

						data_arr[i]=msg.points[i].positions[j];
						Time[i]=((double)msg.points[i].time_from_start.toSec());
						ROS_INFO("%f",((double)msg.points[i].time_from_start.toSec()));
						originFile<<msg.points[i].time_from_start.toSec()<<","<<msg.points[i].positions[j]<<","<<msg.points[i].velocities[j]<<","<<msg.points[i].accelerations[j]<<endl;
				} 
				//将各轴数据分开
				originFile<<" "<<","<<" "<<","<<" "<<","<<" "<<endl;
				spline.loadData(Time,data_arr,points_count,0,0, cubicSpline::BoundType_First_Derivative);
				double x_out = 0;
				double y_out = 0;
				ROS_INFO("%f",(double)msg.points[points_count-1].time_from_start.toSec()); 

				vector<double> single_axis_points;

				for(double i=0; i<=(((double)msg.points[points_count-1].time_from_start.toSec())); i=i+0.004)
				{
						x_out = x_out + 0.004;
						spline.getYbyX(x_out, y_out); //x是时间，y是点
						single_axis_points.push_back(y_out); //get points in single axis
				}
				spline.oFile<<" "<<","<<" "<<","<<" "<<","<<" "<<endl;
				all_points.push_back(single_axis_points); //pull all point together. rows represent axis


		}  
}


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
		ros::Subscriber sub=n.subscribe("joint_path_command",10,chatterCB);

		int count=0;
		ros::Rate loop_rate(125);
		Robot robot("192.168.1.102:50051");

		//  pos[0]=34.507;
		//  pos[1]=145.062;
		//  pos[2]=-56.326;
		//  pos[3]=164.070;
		//  pos[4]=-137.357;
		//  pos[5]=103.926;

		robot.start();

		// ros::Subscriber sub=n.subscribe("/joint_states", 1000, updateOringinPos);
		int reply=robot.ServoJEnable(true);
		std::cout<<"servoj enable "<<reply<<std::endl;

		// SubscribeAndPublish test("192.168.2.60:50051");  
		// ros::MultiThreadedSpinner s(2);  //多线程
		// ros::spin(s);  
		int j = 0;
		while (ros::ok())
		{
				std_msgs::String msg;
				//print 6 axis point in same time stamp, should change later. should be sent to the sj interface
				if (j<all_points[0].size())
				{
						for(int i=0; i<all_points.size(); i++)
						{
								pos[i] = all_points[i][j];
						}
						j++;

						++count;
						std::stringstream ss;
						ss << "hello world " << count;
						msg.data = ss.str();
						chatter_pub.publish(msg);
						robot.servoJMove(pos,8);
						ros::spinOnce();

						loop_rate.sleep();
				}
				else
						all_points.clear();
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
