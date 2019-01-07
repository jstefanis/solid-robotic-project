#!/usr/bin/env python

# detecting ar markers, and estimating the pose of each one in the global frame.

import rospy, math,os
import numpy as np
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, Twist
from ar_track_alvar_msgs.msg import *
from std_msgs.msg import Bool

class tag_detection():
       
    def __init__(self):
        rospy.init_node("tag_pose_localisation")
        #rospy.on_shutdown(self.shutdown)
    
        self.counter0 = 0
        self.counter1 = 0
        self.counter2 = 0
        #global t0_flag, t1_flag
        self.t0_flag, self.t1_flag,t2_flag = False, False, False
        
        self.tag0_coords=[0,0,0]
        self.tag1_coords=[0,0,0]
        self.tag2_coords=[0,0,0]

        global tag_0, tag_1, tag_2
        #create PoseStamped variables to store the pose
        tag_0 = PoseStamped()
        tag_1 = PoseStamped()
        tag_2 = PoseStamped()
        

        self.tag_ids = [0,1,2]

        #Publish on /target_pose0 & 1 & 2 ,topic as a PoseStamped message
        self.tag_pub0 = rospy.Publisher("target_pose0", PoseStamped, queue_size=10)
        self.tag_pub1 = rospy.Publisher("target_pose1", PoseStamped, queue_size=10)
        self.tag_pub2 = rospy.Publisher("target_pose2", PoseStamped, queue_size=10)

       #Publish on /t0_flag & 1 & 2 ,topic as a bool message
        self.t0_flag = rospy.Publisher("t0_flag", Bool, queue_size=10)
        self.t1_flag = rospy.Publisher("t1_flag", Bool, queue_size=10)
        self.t2_flag = rospy.Publisher("t2_flag", Bool, queue_size=10)
        
        tag_detection_finished_flag = rospy.Publisher("tag_detection_finished", Bool, queue_size=10)

        #Subscribe pose from amcl_pose
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.get_turtlePose)

        rospy.Subscriber('rotation_flag', Bool, self.rotate_flag_enabled)

        ##subscribe pose from alvar markers
        #rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tag_detection)
        #rospy.loginfo("Publishing marker pose and flags on topic target_pose0 target_pose1 target_pose2 target0_flag target1_flag target2_flag")
       
        rospy.spin() # keep node running
        
    def rotate_flag_enabled:
        #subscribe pose from alvar markers #self
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.get_tag_detection)
        rospy.loginfo("Publishing marker pose and flags on topic target_pose0 target_pose1 target_pose2 target0_flag target1_flag target2_flag")
 
        

    def get_turtlePose(self, msg):
        if self.counter == 3:
            rospy.loginfo("solid_nav localisation: Three tags were detected")
            return
        # get the pose of turtlebot
        global turtlebot_posX, turtlebot_posY, turtlebot_posZ, turtlebot_rotZ, turtlebot_rotW, turtlebot_Angle
        turtlebot_posX = msg.pose.pose.position.x
        turtlebot_posY = msg.pose.pose.position.y
        turtlebot_posZ = msg.pose.pose.position.z
        
        turtlebot_rotZ = msg.pose.pose.orientation.z
        turtlebot_rotW = msg.pose.pose.orientation.w
        turtlebot_Angle = math.atan2(2*turtlebot_rotZ*turtlebot_rotW, 1-2*turtlebot_rotZ*turtlebot_rotZ)
        
       #launch_flag_freenect_start = rospy.Publisher("freenect_nav_start", Bool, queue_size=10)
       #launch_flag_freenect_start.publish(True)

        return
    
    def transform_coords(self,msg):
        
        turtlebot_Angle = math.atan2(2 * turtlebot_rotZ * turtlebot_rotW, 1 - 2 * turtlebot_rotZ * turtlebot_rotZ)

        # 4x4 transform matrix
        a1 = math.cos(turtlebot_Angle)
        a2 = -math.sin(turtlebot_Angle)
        a3 = 0
        a4 = turtlebot_posX
           
        b1 = -a2 #math.sin(turtlebot_Angle)
        b2 = 0
        b3 = a1
        b4 = turtlebot_posY
                
        c1 = 0
        c2 = 0
        c3 = 1
        c4 = turtlebot_posZ
                
        d1 = 0
        d2 = 0
        d3 = 0
        d4 = 1

        """             
        cos(theta) -sin(theta)      0      PosX
        sin(theta)    0        cos(theta)  Posy
        0             0             1      Posz
        0             0             0       1
                
        """
        transform_matrix= [[a1,a2,a3,a4],
                          [b1,b2,b3,b4],
                          [c1,c2,c3,c4],
                          [d1,d2,d3,d4]]
        # return transform_matrix
        return transform_matrix
    
    def get_tag_detection(self, msg):
        #get pose of markers 0 and 1 and 2 and calculate absolute pose
        # publish poses in the respective topics
        
        if self.counter == 3:
            self.tag_detection_finished_flag.publish(True)
            return
        
        self.tag_pub0.publish(tag_0)
        self.tag_pub1.publish(tag_1)
        self.tag_pub2.publish(tag_2)
        
        #rate = rospy.rate(10)
        if self.t0_flag == True and self.t1_flag == True and self.t2_flag == True:
            start_autonomous_navigation = rospy.Publisher("start_autonomous_navigation_flag", Bool, queue_size=10)
            start_autonomous_navigation_flag = True
            i = 1
            while i < 100:
                i += 1
                start_autonomous_navigation.publish(start_autonomous_navigation_flag)
            return
        else:
            start_autonomous_navigation_flag = False
            start_autonomous_navigation = rospy.Publisher("start_autonomous_navigation_flag", Bool, queue_size=10)
            start_autonomous_navigation.publish(start_autonomous_navigation_flag)
            

        #get number of aR_markers detected
        n = len(msg.markers)

        if n == 0:
            return

        for tag in msg.markers:
            if tag.id == 0:
                if self.t0_flag == True:
                    continue
                ####
                
                frame_transformation = self.transform_coords(msg)
                
                ####
                
                
                local_ref_point = [[tag.pose.pose.position.x], [tag.pose.pose.position.y], [tag.pose.pose.position.z], [1]]
                
                global_ref_point = np.matmul(frame_transformation, local_ref_point)

                
                self.tag0_coords = self.tag0_coords + global_ref_point
                self.counter0 += 1
                
                # sample three times the coordinates and take the average
                if self.counter0 == 3:
                    tag_0.pose.position.x = self.tag0_coords[0]/3.0
                    tag_0.pose.position.y = self.tag0_coords[1]/3.0
                    tag_0.pose.position.z = self.tag0_coords[2]/3.0
                   
                    tag_0.pose.orientation.w = 1.0

                    #add time stamp and frameid
                    tag_0.header.stamp = rospy.Time.now()
                    tag_0.header.frame_id = "map"
                   
                    self.t0_flag = True
                    self.counter =+1

            elif tag.id == 1:
                if self.t1_flag == True:
                    continue
                  
                ####
                
                frame_transformation = self.transform_coords(msg)
                
                ####
                
 
                      
                local_ref_point = np.array([[tag.pose.pose.position.x], [tag.pose.pose.position.y], [tag.pose.pose.position.z], [1]])
                global_ref_point = np.matmul(frame_transformation, local_ref_point)
                
                self.tag1_coords = self.tag1_coords + global_ref_point
                self.counter1 += 1
            
                # sample three times the coordinates and take the average
                if self.counter1 == 3:
                    tag_1.pose.position.x = self.tag1_coords[0]/3.0
                    tag_1.pose.position.y = self.tag1_coords[1]/3.0
                    tag_1.pose.position.z = self.tag1_coords[2]/3.0
                   
                    tag_1.pose.orientation.w = 1.0

                    #add time stamp and frameid
                    tag_1.header.stamp = rospy.Time.now()
                    tag_1.header.frame_id = "map"
                   
                    self.t1_flag = True
                    self.counter =+ 1


            elif tag.id == 2:
                if self.t2_flag == True:
                    continue
		  
                ####
                
                frame_transformation = self.transform_coords(msg)
                
                ####
                
 
                      
                local_ref_point = np.array([[tag.pose.pose.position.x], [tag.pose.pose.position.y], [tag.pose.pose.position.z], [1]])
                global_ref_point = np.matmul(frame_transformation, local_ref_point)

                self.tag2_coords = self.tag2_coords + global_ref_point
                self.counter2 += 1

                # sample three times the coordinates and take the average
                if self.counter2 == 3:
                    tag_2.pose.position.x = self.tag2_coords[0]/3.0 
                    tag_2.pose.position.y = self.tag2_coords[1]/3.0                
                    tag_2.pose.position.z = self.tag2_coords[2]/3.0 
                   
                    tag_2.pose.orientation.w = 1.0

                    #add time stamp and frameid
                    tag_2.header.stamp = rospy.Time.now()
                    tag_2.header.frame_id = "map"
                   
                    self.t2_flag = True
                    self.counter =+1

            else:
                continue

if __name__ == '__main__':
    try:
        tag_detection()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("solid_nav localization terminated.")
        



        
        
        
               
        

