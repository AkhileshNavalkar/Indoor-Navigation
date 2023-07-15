#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "math.h"
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
void goal(double a,double b);
void callback(const turtlesim::Pose::ConstPtr & pose_message);
int main(int argc,char **argv)
{
  ros::init(argc,argv,"goal");
  ros::NodeHandle n;
  double speed,angspeed;
  double dist,ang;
  bool isForward,clockwise;
  velocity_publisher=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
  pose_subscriber=n.subscribe("/turtle1/pose",10,callback);
  ros::Rate loop_rate(10);
  //spiralclean();
  //circle();
  goal(1,1);
  ros::spin();
  return 0;

}
void callback(const turtlesim::Pose::ConstPtr & pose_message){
 turtlesim_pose.x=pose_message->x;
 turtlesim_pose.y=pose_message->y;
 turtlesim_pose.theta=pose_message->theta;
 }
 void goal(double a,double b){
  geometry_msgs::Twist vel_msg;
  double x,y,theta;
  double d=10;
  
  
  
  ros::Rate loop(1);
  do{
  x=turtlesim_pose.x;
  y=turtlesim_pose.y;
  theta=turtlesim_pose.theta;
  d=sqrt(pow((a-x),2)+pow((b-y),2));
  
  vel_msg.linear.x=1.5*d;
  vel_msg.linear.y=0;
  vel_msg.linear.z=0;
  vel_msg.angular.x=0;
   vel_msg.angular.y=0;
  vel_msg.angular.z=4*((atan2(b-y,a-x))-theta);
  velocity_publisher.publish(vel_msg);
  ros::spinOnce();
  loop.sleep();
  
  }while(d>0.01);
  vel_msg.linear.x=0;
   vel_msg.angular.z=0;
   
   velocity_publisher.publish(vel_msg);
}
  
  
  
  
  
  
  
 
