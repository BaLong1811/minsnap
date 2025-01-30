#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point, PoseStamped

def publish_odom_and_goal():
    rospy.init_node('uav_odom_and_goal_publisher', anonymous=True)

    # Publishers
    odom_pub = rospy.Publisher('/uav/odom', Odometry, queue_size=10)
    goal_pub = rospy.Publisher('/goal_position', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        odom_msg = Odometry()
        #goal_msg = Point()
        goal_msg = PoseStamped()
        
        # Populate the header with the current time and frame for odometry
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        
        # Set up PoseWithCovariance
        pose = PoseWithCovariance()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        # Assign your values to pose.pose.position, pose.pose.orientation, and pose.covariance
        
        # Set up TwistWithCovariance
        twist = TwistWithCovariance()
        #twist.twist.linear = 0.0;
        # Assign your values to twist.twist.linear, twist.twist.angular, and twist.covariance

        # Attach pose and twist to odometry message
        odom_msg.pose = pose
        odom_msg.twist = twist

        # Set the goal position (example values)
        goal_msg.pose.position.x = 1.0
        goal_msg.pose.position.y = 7.0
        goal_msg.pose.position.z = 6.0
        
        # Publish the odometry message and the goal position
        odom_pub.publish(odom_msg)
        goal_pub.publish(goal_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odom_and_goal()
    except rospy.ROSInterruptException:
        pass

