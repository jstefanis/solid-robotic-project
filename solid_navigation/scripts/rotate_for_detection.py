#!/usr/bin/env python

#http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
#Node to rotate the turtlebot in place 360degrees

# import required packages
import rospy
from geometry_msgs.msg import Twist
from math import radians
from std_msgs.msg import Bool
from tf import transformations
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
#from ar_track_alvar_msgs.msg import *




PI = 3.1415926535897

class rotate_for_detection():
    def __init__(self):
        #Starts a new node
        rospy.init_node('rotate', anonymous=False)
        rospy.Subscriber("ready_for_nav_second_task",Bool,self.rotate())
        #rotate a second time
        rospy.Subscriber("we_re_at_the_second_goal_nav",Bool,self.rotate_quarter() )
        #self.rotate()
        #keep node running
        rospy.spin

    def rotate(self):
      #http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right

        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        vel_msg = Twist()
        
        # Receiveing the user's input
        print("Let's rotate the turtlebot")

        
        #Converting from angles to radians
        
        speed=10 # degrees/sec       
        angular_speed = speed*2*PI/360
        #relative_angle = angle*2*PI/360
        
        #We won't use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        relative_angle = radians(360) # a full cycle

        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
            
        #Forcing our robot to stop rotating
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        return True
        
      #if rotation_flag:  
        #rate = rospy.Rate(10)
        #rotate_flag = rospy.Publisher('rotation_flag', Bool, queue_size=10)
        #rospy.loginfo("Published /rotation_flag topic")
 
        #while True:
            #rotate_flag.publish(rotation_flag)
            #rate.sleep()
      #else:
          #rospy.loginfo("Failed to rotate turtlebot")

        ## Sleep to give the last log messages time to be sent
        #rospy.sleep(1)
        
    def rotate_quarter(self):
      #http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right

        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        cmd_vel = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        vel_msg = Twist()
        
        # Receiveing the user's input
        print("Let's rotate the turtlebot")
        #speed = input("Input your speed (degrees/sec):")
        #angle = input("Type your distance (degrees):")
        #clockwise = input("Clockwise?: ") #True or false
        
        #Converting from angles to radians
        
        speed=10 # degrees/sec       
        angular_speed = speed*2*PI/360
        #relative_angle = angle*2*PI/360
        
        #We won't use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        relative_angle = radians(270) # a quarter cycle

        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
            
        #Forcing our robot to stop rotating
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

        return True
        
      #if rotation_flag:  
        #rate = rospy.Rate(10)
        #rotate_flag = rospy.Publisher('rotation_flag', Bool, queue_size=10)
        #rospy.loginfo("Published /rotation_flag topic")
 
        #while True:
            #rotate_flag.publish(rotation_flag)
            #rate.sleep()
      #else:
          #rospy.loginfo("Failed to rotate turtlebot")

        ## Sleep to give the last log messages time to be sent
        #rospy.sleep(1)

if __name__ == '__main__':
    try:
        # Initializing our function
        result = rotate_for_detection()
        #rotate
        #result =rotate_for_detection.rotate()

        if result:
          rate = rospy.Rate(10)
          rotate_flag = rospy.Publisher('rotation_flag', Bool, queue_size=10)
          rospy.loginfo("Published /rotation_flag topic")
          rotation_flag = True

          while True:
              rotate_flag.publish(rotation_flag)
              rate.sleep()
 
        else:
          rospy.loginfo("Failed to rotate turtlebot")
          # Sleep to give the last log messages time to be sent
          rospy.sleep(1)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("rotate exception")
