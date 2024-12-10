#! /usr/bin/python3
import rospy
from mavros_msgs.msg import State, AttitudeTarget, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamSet, ParamSetResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray

from utils import *


class UAV_ROS:
    def __init__(self,
                 m: float = 1.5,
                 dt: float = 0.01,
                 time_max: float = 30.,
                 pos0: np.ndarray = np.zeros(3),
                 euler0: np.ndarray = np.zeros(3),
                 offset: np.ndarray = np.zeros(3),
                 group='/uav0',
                 use_ros_param: bool = False,
                 name: str = '~uav0_parameters'):

        self.pos0 = pos0
        self.euler0 = euler0
        self.offset = offset
        self.dt = dt
        self.time_max = time_max  # 每回合最大时间
        self.group = group
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.att = np.zeros(3)
        self.pqr = np.zeros(3)
        
        self.n = 0  # 记录走过的拍数
        self.time = 0.  # 当前时间
        self.last_vio_time = 0.
        
        '''control'''
        #self.throttle = self.m * self.g  # 油门
        #elf.phi_d = 0.
        #self.theta_d = 0.
        '''control'''
        
        self.current_state = State()  # monitor uav status
        self.ctrl_param = Float32MultiArray(data=[0., 0., 0., 0., 0., 0., 0., 0., 0.])  # 9 维
        self.nn_input = Float32MultiArray(data=[0., 0., 0., 0., 0., 0.]) # 6 维
        self.pose = PoseStamped()  # publish offboard [x_d y_d z_d] cmd
        self.uav_odom = Odometry()  # subscribe uav state x y z vx vy vz phi theta psi p q r
        self.ctrl_cmd = AttitudeTarget()  # publish offboard expected [phi_d theta_d psi_d throttle] cmd
        self.voltage = 11.4  # subscribe voltage from the battery
        self.global_flag = 0  # UAV working mode monitoring
        
        self.state_sub = rospy.Subscriber(self.group + "/mavros/state", State, callback=self.state_cb)
        self.ctrl_param_sub = rospy.Subscriber(self.group + "/ctrl_param", Float32MultiArray, callback=self.ctrl_param_cb)
        
        self.uav_vel_sub = rospy.Subscriber(self.group + "/mavros/local_position/odom", Odometry, callback=self.uav_odom_cb)
        self.uav_battery_sub = rospy.Subscriber(self.group + "/mavros/battery", BatteryState, callback=self.uav_battery_cb)
        '''topic subscribe'''
        
        self.local_pos_pub = rospy.Publisher(self.group + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.nn_input_state_pub = rospy.Publisher(self.group + "/nn_input_rl", Float32MultiArray, queue_size=10)
        self.uav_att_throttle_pub = rospy.Publisher(self.group + "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
        '''Publish 位置指令给 UAV'''
        
        '''arming service'''
        rospy.wait_for_service(self.group + "/mavros/cmd/arming")  # 等待解锁电机的 service 建立
        self.arming_client = rospy.ServiceProxy(self.group + "/mavros/cmd/arming", CommandBool)
        
        '''working mode service'''
        rospy.wait_for_service(self.group + "/mavros/set_mode")  # 等待设置 UAV 工作模式的 service 建立
        self.set_mode_client = rospy.ServiceProxy(self.group + "/mavros/set_mode", SetMode)

        rospy.wait_for_service(self.group + "/mavros/param/set")  # 等待设置 UAV 工作模式的 service 建立
        self.set_param_client = rospy.ServiceProxy(self.group + "/mavros/param/set", ParamSet)
        
        self.rate = rospy.Rate(1 / self.dt)
        self.offb_set_mode = SetModeRequest()  # 先设置工作模式为 offboard
        self.arm_cmd = CommandBoolRequest()

        self.set_param("MPC_XY_VEL_MAX", 0.6)
        self.set_param("MPC_Z_VEL_MAX_UP", 0.5)
        self.set_param("MPC_Z_VEL_MAX_DN", 0.5)
        self.set_param("MC_YAWRATE_MAX", 90.0)
        self.set_param("MPC_LAND_SPEED", 0.5)

    def set_param(self, param_name, value):
        param_val = ParamValue()
        param_val.integer = 0
        param_val.real = value
        resp = self.set_param_client(param_id=param_name, value=param_val)
        print("SET ", param_name, " STATE: ", resp.success)
    
    def ctrl_param_cb(self, msg):
        self.ctrl_param = msg
    
    def state_cb(self, msg):
        self.current_state = msg
    
    def uav_odom_cb(self, msg: Odometry):
        self.uav_odom.pose.pose.position.x = msg.pose.pose.position.x + self.offset[0]
        self.uav_odom.pose.pose.position.y = msg.pose.pose.position.y + self.offset[1]
        self.uav_odom.pose.pose.position.z = msg.pose.pose.position.z + self.offset[2]
        self.uav_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.uav_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.uav_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.uav_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w
        
        self.uav_odom.twist.twist.linear.x = msg.twist.twist.linear.x
        self.uav_odom.twist.twist.linear.y = msg.twist.twist.linear.y
        self.uav_odom.twist.twist.linear.z = msg.twist.twist.linear.z
        self.uav_odom.twist.twist.angular.x = msg.twist.twist.angular.x
        self.uav_odom.twist.twist.angular.y = msg.twist.twist.angular.y
        self.uav_odom.twist.twist.angular.z = msg.twist.twist.angular.z
    
    def uav_battery_cb(self, msg: BatteryState):
        self.voltage = msg.voltage
    
    def set_state(self, xx: np.ndarray):
        self.pos[:] = xx[0:3]
        self.vel[:] = xx[3:6]
        self.att[:] = xx[6:9]
        self.pqr[:] = xx[9:12]
    
    def approaching(self):
        self.pose.pose.position.x = self.pos0[0] - self.offset[0]
        self.pose.pose.position.y = self.pos0[1] - self.offset[1]
        self.pose.pose.position.z = self.pos0[2] - self.offset[2]
        
        cmd_q = euler2quaternion(self.euler0)
        self.pose.pose.orientation.x = cmd_q[0]
        self.pose.pose.orientation.y = cmd_q[1]
        self.pose.pose.orientation.z = cmd_q[2]
        self.pose.pose.orientation.w = cmd_q[3]

        self.local_pos_pub.publish(self.pose)
        
        self.set_state(uav_odom_2_uav_state(self.uav_odom))
        

        if ((np.linalg.norm(self.pos0 - self.pos) < 0.4) and  # 位置误差
                (np.linalg.norm(self.vel) < 0.3) and  # 速度
                (np.linalg.norm(self.att[2] - self.euler0[2]) < deg2rad(20)) and # 偏航角
                self.global_flag != -1):  
            #self.global_flag = 2
            return 1
        return 0
    
    def connect(self):
        while (not rospy.is_shutdown()) and (not self.current_state.connected):
            self.rate.sleep()
        
        self.pose.pose.position.x = self.uav_odom.pose.pose.position.x
        self.pose.pose.position.y = self.uav_odom.pose.pose.position.y
        self.pose.pose.position.z = self.uav_odom.pose.pose.position.z
        
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
    
    def offboard_arm(self):
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd.value = True  # 通过指令将电机解锁
        
        while (self.current_state.mode != "OFFBOARD") and (not rospy.is_shutdown()):  # 等待
            if self.set_mode_client.call(self.offb_set_mode).mode_sent:
                print('Switching to OFFBOARD mode is available...waiting for 1 seconds')
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
        
        while (not self.current_state.armed) and (not rospy.is_shutdown()):
            if self.arming_client.call(self.arm_cmd).success:
                print('UAV is armed now.')
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

    def land(self):
        self.offb_set_mode.custom_mode = 'AUTO.LAND'
        while (self.current_state.mode != "AUTO.LAND") and (not rospy.is_shutdown()):
            if self.set_mode_client.call(custom_mode='AUTO.LAND').mode_sent:
                #print('Switching to AUTO.LAND mode is available')
                break

    def switch_stabilized(self):
        self.offb_set_mode.custom_mode = 'STABILIZED'
        while (self.current_state.mode != "STABILIZED") and (not rospy.is_shutdown()):
            if self.set_mode_client.call(custom_mode='STABILIZED').mode_sent:
                #print('Switching to STABILIZED mode is available')
                break

