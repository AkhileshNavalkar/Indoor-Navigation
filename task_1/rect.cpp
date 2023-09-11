#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
void move(double speed,double distance); 
void rot(double w,double theta);
void callback(const turtlesim::Pose::ConstPtr & pose_message);
void rect(double d,double sp);
int main(int argc,char **argv)
{
  ros::init(argc,argv,"rectangle");
  ros::NodeHandle n;
  double speed,angspeed;
  double dist,ang;
  bool isForward,clockwise;
  velocity_publisher=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
  pose_subscriber=n.subscribe("/turtle1/pose",10,callback);
  
  ros::Rate loop_rate(10);
  //move(1,5);
  rect(4,1);
  
  ros::spin();
  return 0;

}
void callback(const turtlesim::Pose::ConstPtr & pose_message){
 turtlesim_pose.x=pose_message->x;
 turtlesim_pose.y=pose_message->y;
 turtlesim_pose.theta=pose_message->theta;
 }
 void rect(double d,double sp){
   move(sp,d);
   rot(1,90);
   move(sp,d);
   rot(1,90);
   move(sp,d);
   rot(1,90);
   move(sp,d);
 


}
 void rot(double w,double theta){
 geometry_msgs::Twist vel_msg;
   vel_msg.linear.x= 0  ;
   vel_msg.linear.y=0;
   vel_msg.linear.z=0;
   vel_msg.angular.x=0;
   vel_msg.angular.y=0;
   vel_msg.angular.z=w;
   double b;
   b= (theta*3.14)/180;
   double a=0;
   double t0=ros::Time::now().toSec();
   ros::Rate loop(100);
   
   do{
   double t1=ros::Time::now().toSec();
   velocity_publisher.publish(vel_msg);
   a=w*(t1-t0);
   ros::spinOnce();
   loop.sleep();
   }while(a<b);
   vel_msg.angular.z=0;
   velocity_publisher.publish(vel_msg);
}
 void move(double speed,double distance){
  geometry_msgs::Twist vel_msg;
  double d,t0;
  vel_msg.linear.x= speed  ;
   vel_msg.linear.y=0;
   vel_msg.linear.z=0;
   vel_msg.angular.x=0;
   vel_msg.angular.y=0;
   vel_msg.angular.z=0;
   t0=ros::Time::now().toSec();
   d=0;
   
  
  ros::Rate loop(100);
  do{
   velocity_publisher.publish(vel_msg);
   double t1=ros::Time::now().toSec();
   d=speed*(t1-t0);
   ros::spinOnce();
   loop.sleep();
   }while(d<distance);
   vel_msg.linear.x= 0  ;
   velocity_publisher.publish(vel_msg);
}
 
   
   
  
  
