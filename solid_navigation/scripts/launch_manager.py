#!/usr/bin/env python

# http://wiki.ros.org/roslaunch/API%20Usage
# https://is.gd/IuDDu1
#using roslaunch api for controlling kinect controller by using topic messages
import roslaunch
import rospy
class launch_manager():
  def __init__(self):
    rospy.loginfo("launch manager of solid_navigation started")
    rospy.init_node('launch_manager', anonymous=True, disable_signals=True)
    rospy.on_shutdown(self.shutdown)
    
    rospy.Subscriber("ready_for_nav_second_task",Bool,kill_amcl_node())
    #rospy.Subscriber("freenect_nav_start",Bool,start_freenect_node())

    launch_flag_kill_amcl = rospy.Publisher("amcl_killed", Bool, queue_size=10)
    launch_flag_kill_amcl = False
    #launch_flag_freenect_start = rospy.Publisher("freenect_launched", Bool, queue_size=10)
    #launch_flag_freenect_start = False

    uuid_amcl = roslaunch.rlutil.get_or_generate_uuid(None, False)
    uuid_freenect = roslaunch.rlutil.get_or_generate_uuid(None, False)
  
    roslaunch.configure_logging(uuid_amcl)
    roslaunch.configure_logging(uuid_freenect)
    
    launch_amcl = roslaunch.parent.ROSLaunchParent(uuid_amcl,["/home/turtlebot/ros/indigo/catkin_ws/src/solid_navigation/launch/amcl_test.launch"])
    #launch_amcl.start()
    #rospy.loginfo("amcl started")

    launch_freenect = roslaunch.parent.ROSLaunchParent(uuid_freenect,["/home/turtlebot/ros/indigo/catkin_ws/src/solid_navigation/launch/freenect_test.launch"])
    #launch_freenect.start()
    
    #launch amcl script
    amcl_status = start_amcl_node()
    
    rospy.sleep(30)
    
  def start_amcl_node(self):
    self.launch_amcl.start()
    rospy.loginfo("launchmanager: amcl started")
    return True

  def start_freenect_node(self):
    self.launch_freenect.start()
    rospy.loginfo("launchmanager: freenect started")

    
  def kill_amcl_node(self):      
     rospy.loginfo("launchmanager: amcl shutdown")
      self.launch_amcl.shutdown()
      #start freenect after 20 seconds
      rospy.sleep(20)
      
      self.start_freenect_node()
     
  def kill_freenect_node(self):
      rospy.loginfo("launchmanager: freenect shutdown")
      self.launch_freenect.shutdown()
    
  def shutdown(self):
    rospy.loginfo("Stopping the launch file conf script...")
    #destroy()
    rospy.sleep(1)
    reason = "shutdown signal sent"
    rospy.signal_shutdown(reason)

if __name__ == '__main__':
    try:
        manager=launch_manager()
    except rospy.ROSInterruptException:
        rospy.loginfo("launchnode node terminated.")
