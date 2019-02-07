# !/usr/bin/python
#
# Example code to read and plot the odometry data.
#
# To call:
#
#   python read_odom.py odometry.csv
#

import sys
from sys import argv
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def main(args):

    if len(sys.argv) < 3:
        print 'Please specify odometry files, <odometry calculated.csv> <ground_truth.csv>'
        return 1

    odom = pd.read_csv(argv[-1])
    ground_truth = pd.read_csv(argv[-2])

    t_odom = odom['%time']

    x_odom = odom['field.pose.pose.position.x']
    y_odom = odom['field.pose.pose.position.y']
    z_odom = odom['field.pose.pose.position.z']

    r_odom = odom['field.pose.pose.orientation.x']
    p_odom = odom['field.pose.pose.orientation.y']
    h_odom = odom['field.pose.pose.orientation.z']

    t_gt = ground_truth['%time']

    x_gt = ground_truth['field.pose.pose.position.x']
    y_gt = ground_truth['field.pose.pose.position.y']
    z_gt = ground_truth['field.pose.pose.position.z']

    r_gt = ground_truth['field.pose.pose.orientation.x']
    p_gt = ground_truth['field.pose.pose.orientation.y']
    h_gt = ground_truth['field.pose.pose.orientation.z']

    plt.figure()
    plt.subplot(1, 2, 1)
    plt.plot(x_odom, y_odom, 'g', label='Estimated Position')
    plt.plot(x_gt, y_gt, 'r', label='Ground truth')
    plt.title('Odometry position')
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(t_odom, h_odom, 'g')
    plt.plot(t_gt, h_gt, 'r')
    plt.legend(['Estimated Position','Ground truth'])
    plt.title('Odometry yaw')
    plt.show()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
