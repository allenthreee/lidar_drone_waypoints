import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os, sys


def plot_xi():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, xi[:, 0], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('xi_x')

    plt.subplot(1, 3, 2)
    plt.plot(time, xi[:, 1], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('xi_y')

    plt.subplot(1, 3, 3)
    plt.plot(time, xi[:, 2], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('xi_z')


def plot_var_theta():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, var_theta[:, 0], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('var_theta_x')

    plt.subplot(1, 3, 2)
    plt.plot(time, var_theta[:, 1], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('var_theta_y')

    plt.subplot(1, 3, 3)
    plt.plot(time, var_theta[:, 2], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('var_theta_z')


def plot_sigma():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, sigma[:, 0], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('sigma_x')

    plt.subplot(1, 3, 2)
    plt.plot(time, sigma[:, 1], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('sigma_y')

    plt.subplot(1, 3, 3)
    plt.plot(time, sigma[:, 2], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('sigma_z')


def plot_delta():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, delta[:, 0], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('delta_x')

    plt.subplot(1, 3, 2)
    plt.plot(time, delta[:, 1], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('delta_y')

    plt.subplot(1, 3, 3)
    plt.plot(time, delta[:, 2], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('delta_z')


if __name__ == '__main__':
    """
    Data formation:
        pos_adap_obs_param.csv:
    """
    path = os.path.dirname(os.path.abspath(__file__)) + '/'
    data = pd.read_csv(path + '/uav0/pos_adap_obs_param.csv', header=0).to_numpy()
    
    L = data.shape[0]
    time = data[0: L - 2, 0]    # 时间
    
    xi = data[0: L - 2, 1:4]  # 速度观测误差
    d_var_theta = data[0: L - 2, 4:7]
    var_theta = data[0: L - 2, 7:10]
    d_sigma = data[0: L - 2, 10:13]
    sigma = data[0: L - 2, 13:16]
    delta = data[0: L - 2, 16:19]
    
    plot_xi()
    plot_var_theta()
    plot_sigma()
    plot_delta()
    plt.show()
