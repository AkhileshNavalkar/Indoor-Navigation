#!/usr/bin/env python3

import rospy
from math import pi,sqrt
from geometry_msgs.msg import Twist
from std_msgs.msg import String
 
class nav():
    
    def __init__(self):
      rospy.init_node('nav',anonymous=True)
      self.vel = Twist()
      self.cmd_vel_topic = '/cmd_vel'
      self.info = rospy.Subscriber('aruco',String,self.callback)
      self.vel_pub = rospy.Publisher(self.cmd_vel_topic,Twist,queue_size=10)
      self.rate = rospy.Rate(10)
      self.x = None

    def callback(self,a):
      self.x = eval(a.data)[0]
      self.dist = eval(a.data)[1]
      self.centroid = eval(a.data)[2]
    
    def select_marker(self):
      if len(self.x) > 1:
        del self.x[1:]
        del self.dist[1:]
        del self.centroid[1:]

    def marker_config(self,id):
      self.cmd = None
      if id == 0 or id == 2:
        self.cmd = "Right"
      elif id == 1 or id == 3:
        self.cmd = "Left"     
    
    def centering(self):
      if self.centroid[0][0] < 310:
        while self.centroid[0][0] < 310:
          self.vel.angular.z = 0.2
          self.vel_pub.publish(self.vel)

      if self.centroid[0][0] > 330:
        while self.centroid[0][0] > 330:
          self.vel.angular.z = -0.2
          self.vel_pub.publish(self.vel)

      self.vel.angular.z = 0
      self.vel_pub.publish(self.vel)

    def detected(self):
      self.select_marker()
      if self.dist[0] > 70:
        while self.dist[0] > 70:
          self.vel.linear.x = 0.2
          self.vel_pub.publish(self.vel)
          self.centering()
          self.rate.sleep()
        self.vel.linear.x = 0
        self.vel_pub.publish(self.vel)   
      self.marker_config(self.x[0])
      print(self.cmd)
      if self.cmd == "Right":
        a = self.x[0]
        theta=0
        t1=rospy.Time.now().to_sec()
        while theta <= pi:
          self.vel.angular.z = -0.2
          self.vel_pub.publish(self.vel)
          t2=rospy.Time.now().to_sec()
          theta = (t2-t1)*(-1*self.vel.angular.z)
          if self.x[0] != -1 and self.x[0] != a:
            self.detected()
            return
          self.rate.sleep()
        self.vel.angular.z=0
        self.vel_pub.publish(self.vel)
        t=rospy.Time.now().to_sec()
        d=0
        while d <= 1:
          self.vel.linear.x = 0.2
          self.vel_pub.publish(self.vel)
          t2=rospy.Time.now().to_sec()
          d=(t2-t)*(self.vel.linear.x)
          if self.x[0] != -1 and self.x[0] != a:
            self.detected()
            return
          self.rate.sleep()
        self.vel.linear.x=0
        self.vel_pub.publish(self.vel)
      
      if self.cmd == "Left":
        a = self.x[0]
        theta=0
        t1=rospy.Time.now().to_sec()
        while theta <= pi:
          self.vel.angular.z = 0.2
          self.vel_pub.publish(self.vel)
          t2=rospy.Time.now().to_sec()
          theta = (t2-t1)*self.vel.angular.z
          if self.x[0] != -1 and self.x[0] != a:
            self.detected()
            return
          self.rate.sleep()
        self.vel.angular.z=0
        self.vel_pub.publish(self.vel)
        t=rospy.Time.now().to_sec()
        d=0
        while d <= 1:
          self.vel.linear.x = 0.2
          self.vel_pub.publish(self.vel)
          t2=rospy.Time.now().to_sec()
          d=(t2-t)*(self.vel.linear.x)
          if self.x[0] != -1 and self.x[0] != a:
            self.detected()
            return
          self.rate.sleep()
        self.vel.linear.x=0
        self.vel_pub.publish(self.vel)
      
      self.explore()
      
    def explore(self):
      while self.x == None  or self.x[0] == -1:
          dist = 0
          ang = 0
          t0=rospy.Time.now().to_sec()
          while self.x == None  or self.x[0] == -1:
            self.vel.angular.z = 0.2
            self.vel_pub.publish(self.vel)
            t2 = rospy.Time.now().to_sec()
            ang = 0.2*(t2-t0)
            if(ang >= pi/2):
              break
          self.vel.angular.z=0
          self.vel_pub.publish(self.vel)
          t=rospy.Time.now().to_sec()

          while self.x == None  or self.x[0] == -1:
            self.vel.linear.x = 0.2
            t1=rospy.Time.now().to_sec()
            d = 0.2*(t1-t)
            self.vel_pub.publish(self.vel)
            if(d >= 1):
              break
          self.vel.linear.x=0
          self.vel_pub.publish(self.vel)
      self.vel.linear.x=0
      self.vel.angular.z=0
      self.vel_pub.publish(self.vel)

      self.detected()

if __name__=='__main__':
 try:
   nav=nav()
   nav.explore()
   rospy.spin()    
 except rospy.ROSInterruptException:
   rospy.loginfo("node_terminated")
   