#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point, PoseStamped
from quadrotor_msgs.msg import TakeoffLand 

def publish_cmd():
    rospy.init_node('takeoff_land_publisher', anonymous=True)

    # Publishers
    takeoff_land_pub = rospy.Publisher('/px4ctrl/takeoff_land', TakeoffLand, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        takeoff_land_msg = TakeoffLand()

        takeoff_land_msg.takeoff_land_cmd = TakeoffLand.TAKEOFF

        takeoff_land_pub.publish(takeoff_land_msg)
        
        rate.sleep()

        # takeoff_land_msg.takeoff_land_cmd = TakeoffLand.LAND

        # takeoff_land_pub.publish(takeoff_land_msg)

if __name__ == '__main__':
    try:
        publish_cmd()
    except rospy.ROSInterruptException:
        pass

