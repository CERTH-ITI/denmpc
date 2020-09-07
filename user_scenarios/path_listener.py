#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

pub = rospy.Publisher('/next_goal', PoseStamped, queue_size=1)

lookahead_distance = 10

def path_callback(data):
    global lookahead_distance
    if lookahead_distance > len(data.poses):
        lookahead_distance = len(data.poses)-1
    print(lookahead_distance)
    pub.publish(data.poses[lookahead_distance])

def listen():
    rospy.init_node('path_listener', anonymous=True)
    rospy.Subscriber(
        '/move_base/TebLocalPlannerROS/local_plan', Path, path_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
