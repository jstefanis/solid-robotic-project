#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from tf import transformations


##

import math
from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped

#global 

class goto_starting_pose():
    def __init__(self): 
        
        rospy.init_node('goto_starting_pose', anonymous=False)
        #starting_pose = PoseWithCovarianceStamped()
        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
        ########################################################
        self.goal_sent = False
        starting_pose = PoseWithCovarianceStamped()
        rospy.Subscriber("/pick_finished_alt_flag",Bool,pick_finished_alternative)
        initialPose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        starting_pose.header.stamp = rospy.Time.now()
        
        starting_pose.header.frame_id = "map"
        #starting_pose.pose.pose.position.x = self.pose[0]
        #starting_pose.pose.pose.position.y = self.pose[1]
        if rospy.has_param("/initial_pose_x"):
          alfa = rospy.get_param("/initial_pose_x")
          print(alfa)
        
        print(rospy.get_param_names())
        #get parameters from the launch file
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name("/initial_pose_x"), rospy.get_param("/initial_pose_x") )
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name("initial_pose_y"),rospy.get_param("initial_pose_y") )

        #get initial position from launchfile  map_navigation_stage.launch
        starting_pose.pose.pose.position.x = rospy.get_param("/initial_pose_x") #initial_pose_x
        starting_pose.pose.pose.position.y = rospy.get_param("/initial_pose_y") #initial_pose_x
        starting_pose.pose.pose.position.z = 0 # rospy.get_param('initial_pose_z') #initial_pose_x

# we extract the yaw component using tf.transformations..quaternion_from_euler(rot) 
#that converts a RPY Euler notation into a  quaternion , 
#The yaw is at index 2 of the euler array.
        (starting_pose.pose.pose.orientation.x, 
         starting_pose.pose.pose.orientation.y, 
         starting_pose.pose.pose.orientation.z, 
         starting_pose.pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, rospy.get_param('initial_pose_a'))#self.pose[2])

        #Initial pose covariance (x*x), used to initialize filter with Gaussian distribution.
        starting_pose.pose.covariance[6*0+0] = 0.5 * 0.5
        #Initial pose covariance (y*y), used to initialize filter with Gaussian distribution.
        starting_pose.pose.covariance[6*1+1] = 0.5 * 0.5
        #starting_pose.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0
        #Initial pose covariance (yaw*yaw), used to initialize filter with Gaussian distribution.
        starting_pose.pose.covariance[6*6-1] = math.pi/12.0 * math.pi/12.0
        
        #########################################################
        
        
    def goto(self, pos, quat):
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	      # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 180 seconds to complete task
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(180)) 

        state = self.move_base.get_state()
        result = False

        if finished_within_time and state == GoalStatus.SUCCEEDED:
        # We made it!
            #result = True
            # create a timer of 300 seconds for finishing visual servoing and pick operation
            #finished_vs_and_pick_operation = self.move_base.wait_for_result(rospy.Duration(300))
            #state_vs = self.move_base.get_state()
            #if finished_vs_and_pick_operation and state vs == GoalStatus.SUCCEEDED:
               #goal_pub = rospy.Publisher("pick_finished_alternative", Bool, queue_size=10)
               #pick_finished_alt_flag = True
               #rate = rospy.Rate(10)
               #i = 1
               #while i < 100:
                   #i =+1
                   #goal_pub.publish(pick_finished_alt_flag)
                   #rospy.loginfo("pick finished alternative flag enabled")
                   #rate.sleep()
                   
            result = True
             
                

            
        else:
            self.move_base.cancel_goal()
            rospy.loginfo("Failed or Timed out achieving goal")
            self.goal_sent = False

        return result
	
    #####################################################################
    def shutdown(self):
        # stop turtlebot
        if self.goal_sent:
          self.move_base.cancel_goal()
          rospy.loginfo("Quit program")
          rospy.sleep(1)
 
    # Allow up to 10 seconds for the actionserver to come up
	#?? temporary disabled
	#self.move_base.wait_for_server(rospy.Duration(10))
#######################################################################   	
        	
if __name__ == '__main__':
    try:
        
        navigator = goto_starting_pose()


		# declare the initial coordinates of plan 
		# the first goal is moving at the center of vibot plateu
		
		#self.x_center =  -2.7
		#self.y_center =  -1.7

        # Customize the following values so they are appropriate for your location
        position = {'x': -2.7, 'y' : -1.7}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.580, 'r4' : 0.815}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        result = navigator.goto(position, quaternion)

        if result:
            goal_pub = rospy.Publisher("we_re_at_the_center", Bool, queue_size=10)
            centerReached = True
            rate = rospy.Rate(10)
            i = 1
                      
            while True:
            
                goal_pub.publish(centerReached)
                #i += 1
                rate.sleep()
        else:
            rospy.loginfo("Failed to reach the center of the plateau")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
      rospy.loginfo("Ctrl-C caught. Quitting")
##
