import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os, sys


def plot_ctrl_delta():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, delta[:, 0], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('ctrl_delta_x')

    plt.subplot(1, 3, 2)
    plt.plot(time, delta[:, 1], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('ctrl_delta_y')

    plt.subplot(1, 3, 3)
    plt.plot(time, delta[:, 2], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('ctrl_delta_z')


if __name__ == '__main__':
    """
    Data formation:
        pos_ctrl_adaptive.csv:
    """
    path = os.path.dirname(os.path.abspath(__file__)) + '/'
    data = pd.read_csv(path + '/uav0/pos_ctrl_adaptive.csv', header=0).to_numpy()
    
    L = data.shape[0]
    time = data[0: L - 2, 0]    # 时间
    d_delta = data[0: L - 2, 1:4]
    delta = data[0: L - 2, 4:7]
    
    plot_ctrl_delta()
    plt.show()
