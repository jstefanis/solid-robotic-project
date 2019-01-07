#!/usr/bin/env python
# wait for 300 second after the flag we_re_at_the_center has been raised.
# if before this, the flag pick_finished is true, the counter node is killed

# detecting ar markers, and estimating the pose of each one in the global frame.

import rospy, math,os
from std_msgs.msg import Bool

class alt_pick_message():
       
    def __init__(self):
        #https://answers.ros.org/question/218240/python-shutdown-node/
        rospy.init_node("alt_pick_message",disable_signals=True)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber("pick_finished", Bool,self.shutdown )
        rospy.Subscriber("we_re_at_the_center", Bool,self.raise_flag)

        rospy.spin()
        
    def raise_flag(self):
        #wait for 300 seconds
        rospy.sleep(300)
        #publish alternative pick flag topic
        publish_alt_pick_flag = rospy.Publisher("pick_finished_alternative",Bool, queue_size=10)
        flag_alt = True
        publish_alt_pick_flag.publish(flag_alt)
        
    def shutdown(self):
        rospy.loginfo("Quit countdown node")
        rospy.sleep(1)
        reason = "Pick signal received"
        rospy.signal_shutdown(reason)



if __name__ == '__main__':
    try:
        alt_pick_message()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("alt_pick_message terminated.")
