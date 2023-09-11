#!/usr/bin/env python3
import rospy
import time
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from array import array
 
class nav:
    
  def _init_(self):
    rospy.init_node('nav',anonymous=True)
    self.vel=Twist()
    self.cmd_vel_topic='/cmd_vel'
    self.info=rospy.Subscriber('aruco',String,self.callback)
    self.vel_pub=rospy.Publisher(self.cmd_vel_topic,Twist,queue_size=10)
    print(1)
    self.rate=rospy.Rate(1)
    self.x = -1
    


  def callback(self,a):
     self.x=eval(a.data)[0]
     self.d=eval(a.data)[1]
     self.cnt=eval(a.data)[2]
     
     
    
    
  def todo(self,a,b,f):
   if len(self.x)>1:
     del self.x[1:]
     del self.d[1:]
     del self.cnt[1:]

  def centering(self):
      self.vel=Twist()
      
      if self.cnt[0][0] < 310:
        while self.cnt[0][0] < 310:
          self.vel.angular.z = 0.2
          self.vel_pub.publish(self.vel)

      if self.cnt[0][0] > 330:
        while self.cnt[0][0] > 330:
          self.vel.angular.z = -0.2
          self.vel_pub.publish(self.vel)

      self.vel.angular.z = 0
      self.vel_pub.publish(self.vel)

    
  def detected(self):
      pi=3.14
      if self.d[0] >70:
        while self.d[0] >70:
          self.vel.linear.x=0.2
          print(self.vel.linear.x)
          self.vel_pub.publish(self.vel)
          self.centering()
          self.rate.sleep()
        self.vel.linear.x=0
        self.vel_pub.publish(self.vel)
      
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
      self.run()

      





  def run(self):
      self.x=-1
      pi=3.14
      while  self.x == -1:
          dist = 0
          ang = 0
          t0=rospy.Time.now().to_sec()
          while self.x == -1:
            self.vel.angular.z = 0.2
            self.vel_pub.publish(self.vel)
            t2 = rospy.Time.now().to_sec()
            ang = 0.2*(t2-t0)
            if(ang >= pi/2):
              break
          self.vel.angular.z=0
          self.vel_pub.publish(self.vel)
          t=rospy.Time.now().to_sec()

          while self.x == -1:
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
   nav._init_()
   nav.run()
   print(11)
   
   
 except rospy.ROSInterruptException:
   rospy.loginfo("node_terminated")
   