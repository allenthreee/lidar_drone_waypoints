#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdometryToPoseStamped:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('odometry_to_pose_stamped', anonymous=True)

        # Subscribe to the Odometry topic
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)

        # Publisher for PoseStamped
        self.pose_pub = rospy.Publisher('/uav0/mavros/vision_pose/pose', PoseStamped, queue_size=10)

    def odom_callback(self, msg):
        # Create a new PoseStamped message
        pose_stamped = PoseStamped()
        
        # Fill in the header
        pose_stamped.header = msg.header
        
        # Fill in the pose data
        pose_stamped.pose = msg.pose.pose
        pose_stamped.header.frame_id = "map"
        
        # Publish the PoseStamped message
        self.pose_pub.publish(pose_stamped)
        print(f"\n\nYWY published posestamped at \n{pose_stamped.pose.position}")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OdometryToPoseStamped()
        node.run()
    except rospy.ROSInterruptException:
        pass