#! /usr/bin/python3
import os, rospy

from control.uav_ros import UAV_ROS
from control.lk_smc import lk_smc_param, lk_smc
from control.lk_obs import lk_obs
from control.collector import data_collector
from control.utils import *

cur_path = os.path.dirname(os.path.abspath(__file__))

if __name__ == "__main__":
    rospy.init_node("control_lk_smc")

    t_miemie = rospy.get_param('~t_miemie')  # 轨迹跟踪前的初始化等待时间
    dt = rospy.get_param('~dt')  # 采样时间
    time_max = rospy.get_param('~time_max')  # 最大仿真时间
    TOTAL_SEQ = round((time_max + t_miemie) / dt)  # 参考序列长度
    use_gazebo = rospy.get_param('~use_gazebo')
    CONTROLLER = rospy.get_param('~controller')
    use_obs = rospy.get_param('~use_obs')

    uav_ros = UAV_ROS(use_ros_param=True, name='~parameters')
    uav_ros.connect()  # 连接
    uav_ros.offboard_arm()  # OFFBOARD 模式 + 电机解锁

    print('Approaching...')
    uav_ros.global_flag = 1

    '''define controllers and observers'''
    obs = lk_obs('~obs')
    pos_ctrl_param = lk_smc_param('~lk_smc_parameters')
    controller = lk_smc(pos_ctrl_param)

    data_record = data_collector(N=TOTAL_SEQ)
    ctrl_param_record = None
    '''define controllers and observers'''

    ra = np.array([1.5, 1.5, 0.2, deg2rad(0)])
    rp = np.array([8, 8, 10, 10])  # xd yd zd psid 周期
    rba = np.array([0.0, 0.0, 1.0, deg2rad(0)])  # xd yd zd psid 幅值偏移
    rbp = np.array([np.pi / 2, 0, 0, 0])  # xd yd zd psid 相位偏移

    e = np.zeros(3).astype(float)
    de = np.zeros(3).astype(float)

    ref_all, dot_ref_all, dot2_ref_all = ref_uav_sequence_with_dead(dt, uav_ros.time_max, t_miemie, ra, rp, rba, rbp)
    
    print('hhh', ref_all.shape)

    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        if uav_ros.global_flag == 1:  # approaching
            uav_ros.approaching()
            t0 = rospy.Time.now().to_sec()
        elif uav_ros.global_flag == 2:  # control
            t_now = round(t - t0, 4)
            if uav_ros.n % 100 == 0:
                print('time: ', t_now)

            '''1. generate reference command and uncertainty'''
            ref, dot_ref, dot2_ref = ref_all[uav_ros.n], dot_ref_all[uav_ros.n], dot2_ref_all[uav_ros.n]

            '''2. generate outer-loop reference signal 'eta_d' and its 1st, 2nd, and 3rd-order derivatives'''
            eta_d, dot_eta_d, dot2_eta_d = ref[0: 3], dot_ref[0: 3], dot2_ref[0: 3]
            e = uav_ros.eta() - eta_d
            de = uav_ros.dot_eta() - dot_eta_d
            psi_d = ref[3]

            if use_obs:
                syst_dynamic = -uav_ros.kt / uav_ros.m * uav_ros.dot_eta() + uav_ros.A()
                observe = obs.observe(de=uav_ros.dot_eta(), dy=syst_dynamic)
            else:
                observe = np.zeros(3)

            '''3. Update the parameters of FNTSMC if RL is used'''
            if CONTROLLER == 'PX4-PID':
                uav_ros.pose.pose.position.x = ref[0] - uav_ros.offset[0]
                uav_ros.pose.pose.position.y = ref[1] - uav_ros.offset[1]
                uav_ros.pose.pose.position.z = ref[2] - uav_ros.offset[2]
                uav_ros.local_pos_pub.publish(uav_ros.pose)
                phi_d, theta_d, uf = 0., 0., 0.
            else:
                '''3. generate phi_d, theta_d, throttle'''
                controller.control_update_outer(e_eta=e,
                                                dot_e_eta=de,
                                                dot_eta=uav_ros.vel,
                                                kt=uav_ros.kt,
                                                m=uav_ros.m,
                                                dd_ref=dot2_eta_d,
                                                obs=observe,
                                                e_m=0.2,
                                                de_m=0.5,
                                                delta_est=True)
                phi_d, theta_d, uf = uav_ros.publish_ctrl_cmd(controller.control_out, psi_d, use_gazebo)

            '''5. get new uav states from Gazebo'''
            uav_ros.rk44(action=[phi_d, theta_d, uf], uav_state=uav_odom_2_uav_state(uav_ros.uav_odom))

            '''6. data storage'''
            data_block = {'time': uav_ros.time,  # simulation time
                          'throttle': uf,
                          'thrust': uav_ros.ctrl_cmd.thrust,
                          'ref_angle': np.array([phi_d, theta_d, psi_d]),
                          'ref_pos': ref[0: 3],
                          'ref_vel': dot_ref[0: 3],
                          'd_out_obs': observe,
                          'state': uav_ros.uav_state_call_back(),
                          'dot_angle': uav_ros.uav_dot_att()}
            data_record.record(data_block)

            if data_record.index == data_record.N:
                print('Data collection finish. Switching to offboard position...')
                save_path = cur_path + '/datasave/uav0/'
                if not os.path.exists(save_path):
                    os.mkdir(save_path)
                data_record.package2file(path=save_path)
                obs.save_adap_obs_param(save_path, flag='pos')
                controller.save_ctrl_adaptive(save_path, flag='pos')
                uav_ros.global_flag = 3
        elif uav_ros.global_flag == 3:  # finish, back to position
            uav_ros.pose.pose.position.x = 0
            uav_ros.pose.pose.position.y = 0
            uav_ros.pose.pose.position.z = 0.5
            uav_ros.local_pos_pub.publish(uav_ros.pose)
        else:
            uav_ros.pose.pose.position.x = 0
            uav_ros.pose.pose.position.y = 0
            uav_ros.pose.pose.position.z = 0.5
            uav_ros.local_pos_pub.publish(uav_ros.pose)
            print('working mode error...')
        uav_ros.rate.sleep()
