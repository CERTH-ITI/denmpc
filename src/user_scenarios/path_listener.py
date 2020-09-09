#!/usr/bin/env python
# dummy node to handle plans and provide a subgoal - just for testing
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

pub = rospy.Publisher('/next_goal', PoseStamped, queue_size=1)

def path_callback(data):
    pub.publish(data.poses[data.poses.size()-1])


def listen():
    rospy.init_node('path_listener', anonymous=True)
    rospy.Subscriber(
        '/rbcar_global_plan', Path, path_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
