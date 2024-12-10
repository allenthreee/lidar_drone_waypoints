#! /usr/bin/python3
import os, rospy
import time
# import serial
import math
# from nav_msgs import Odometry
from uav_ros import UAV_ROS
from utils import *
from geometry_msgs.msg import Point32
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import PointCloud


def odom_callback(data:Odometry, uav_ros):
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z
    uav_ros.last_vio_time = time.time()
    #print(vx, " ", vy, " ", vz)
    if (vx*vx + vy*vy + vz*vz > 3.0*3.0):
        uav_ros.global_flag = -1
        #ser.write("05905911".encode('utf-8'))

def rc_callback(data:RCIn):
    channel6 = data.channels[5]
    if channel6 < 1500:
        print("CH6")
        uav_ros.global_flag = -1
        #ser.write("05905911".encode('utf-8'))

if __name__ == "__main__":
    #ser = serial.Serial("/dev/ttyUSB0", 9600, timeout = 1)

    rospy.init_node("uav0_control_single")
    
    # dt = rospy.get_param('~dt')  # 采样时间
    # time_max = rospy.get_param('~time_max')  # 最大仿真时间
    # use_gazebo = rospy.get_param('~use_gazebo')
    
    uav_ros = UAV_ROS(use_ros_param=True, name='~uav0_parameters')

    #rospy.Subscriber("/lv2/paths_msg", PointCloud, path_points_callback, queue_size=1)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, odom_callback, uav_ros, queue_size=1)
    #rospy.Subscriber("/uav0/mavros/rc/in", RCIn, rc_callback, queue_size=1)

    uav_ros.connect()  # 连接
    uav_ros.offboard_arm()  # OFFBOARD 模式 + 电机解锁

    path_points = np.array([[1, 1, 1, np.pi*0.25], [-1, 1, 1, np.pi], [-1, -1, 1, -np.pi*0.5], [1, -1, 1, 0], [0, 0, 1, np.pi*0.75]])
    cur_point_index = 0
    # path_points = []
    
    print('Approaching...')
    if uav_ros.global_flag != -1:
        uav_ros.global_flag = 1
    
    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        t_acc = t - t0
        t_now = round(t - t0, 4)
        uav_ros.n += 1
        if uav_ros.n % 100 == 0:
            print("MODE: ", uav_ros.global_flag, 'time: ', t_now, \
                  " X: ", uav_ros.uav_odom.pose.pose.position.x, \
                    " Y: ", uav_ros.uav_odom.pose.pose.position.y, \
                        "Z: ", uav_ros.uav_odom.pose.pose.position.z, \
                            "Angle: ", uav_ros.att[2], \
                                "Index: ", cur_point_index)
            
        if time.time() - uav_ros.last_vio_time > 0.5 and uav_ros.last_vio_time > 1e-9:
            print("VIO lost")
            uav_ros.global_flag = -1
            #ser.write("05905911".encode('utf-8'))
        
        if uav_ros.global_flag == -1:
            uav_ros.switch_stabilized()
        elif uav_ros.global_flag == 1:  # approaching
            uav_ros.pos0 = [0.0, 0.0, 1.0]
            uav_ros.euler0 = [0.0, 0.0, 0.0]
            if uav_ros.approaching() == 1 and len(path_points) != 0 and uav_ros.global_flag != -1:
                uav_ros.global_flag = 2
        elif uav_ros.global_flag == 2:  # control
            uav_ros.pos0 = path_points[cur_point_index][0:3]
            uav_ros.euler0 = [0.0, 0.0, path_points[cur_point_index][3]]
            if uav_ros.approaching() == 1 and uav_ros.global_flag != -1:
                cur_point_index += 1
                if cur_point_index == len(path_points):
                    uav_ros.global_flag = 3
                    #ser.write("99999911".encode('utf-8'))
        elif uav_ros.global_flag == 3:  # finish, back to position
            #uav_ros.land()
            break
        else:
            #uav_ros.global_flag = -1
            print('working mode error...')
            break
        uav_ros.rate.sleep()
