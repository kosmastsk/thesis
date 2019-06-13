'''
Copyright (c) 2019 Kosmas Tsiakas

GNU GENERAL PUBLIC LICENSE
  Version 3, 29 June 2007

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''
# python evaluate_localization.py

# INSTRUCTIONS
# You need a rosbag that contains the recording of /amcl_pose and /ground_truth/state topics
# To run the script just run: python evaluate_localization.py <truth>.csv <amcl>.csv
# Always adjust the paths to your files and folders
# Don't forget to run chmod +x evaluate_localization.py before

import sys
import os
import pandas as pd
import numpy as np
import math
import csv
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

# Global Variables that will be used with the metric
position = pd.DataFrame(
    columns=['Mean', 'Median', 'Min', 'Max', 'SSE', 'STD', 'RMSE'])
orientation = pd.DataFrame(
    columns=['Mean', 'Median', 'Min', 'Max', 'SSE', 'STD', 'RMSE'])
# Initialize
attempt = 0
path = '/home'
speed = 'slow'
world = 'warehouse'
movement = 'line'


# Define a function to convert Quaternion x,y,z,w to yaw
# double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
# double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
# yaw = atan2(siny_cosp, cosy_cosp);
def quaternionToYaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


# Mean value
def calculateMean(dataframe):
    global position
    global orientation
    position.at[attempt, 'Mean'] = dataframe['rms_error'].mean()
    orientation.at[attempt, 'Mean'] = dataframe['error_yaw'].mean()


def calculateMedian(dataframe):
    global position
    global orientation
    position.at[attempt, 'Median'] = dataframe['rms_error'].median()
    orientation.at[attempt, 'Median'] = dataframe['error_yaw'].median()


# Min value
def calculateMin(dataframe):
    global position
    global orientation
    position.at[attempt, 'Min'] = dataframe['rms_error'].min()
    orientation.at[attempt, 'Min'] = dataframe['error_yaw'].min()


# Max value
def calculateMax(dataframe):
    global position
    global orientation
    position.at[attempt, 'Max'] = dataframe['rms_error'].max()
    orientation.at[attempt, 'Max'] = dataframe['error_yaw'].max()


# Root Mean Squared Errors
def calculateRMSE(dataframe):
    global position
    global orientation
    position.at[attempt, 'RMSE'] = (dataframe['rms_error']**2).mean()**.5
    orientation.at[attempt, 'RMSE'] = (dataframe['error_yaw']**2).mean()**.5


# Sum of Squared Errors
def calculateSSE(dataframe):
    global position
    global orientation
    position.at[attempt, 'SSE'] = np.sum((dataframe['rms_error']**2))
    orientation.at[attempt, 'SSE'] = np.sum((dataframe['error_yaw']**2))


# Standard deviation
def calculateSTD(dataframe):
    global position
    global orientation
    position.at[attempt, 'STD'] = dataframe['rms_error'].std()
    orientation.at[attempt, 'STD'] = dataframe['error_yaw'].std()


def plot(amcl, truth):
    params = {
        'legend.fontsize': 'x-large',
        'figure.figsize': (25, 15),
        'axes.labelsize': 'x-large',
        'axes.titlesize': 'x-large',
        'xtick.labelsize': 'x-large',
        'ytick.labelsize': 'x-large'
    }
    pylab.rcParams.update(params)

    fig = plt.figure()

    ############## x position ##############
    plt.subplot(2, 2, 1)
    plt.plot(
        'field.header.stamp',
        'field.pose.position.x',
        data=amcl,
        marker='',
        color='#414a4c',
        linewidth=2,
        linestyle='dashed',
        label="MCL estimation")
    plt.plot(
        'field.header.stamp',
        'field.pose.pose.position.x',
        data=truth,
        marker='',
        color='#232b2b',
        linewidth=2,
        linestyle='solid',
        label="Ground truth")
    plt.legend()
    plt.xlabel('time (seconds)')
    plt.ylabel('x-position')
    plt.title('Position in X axis')

    ############## y position ##############
    plt.subplot(2, 2, 2)
    plt.plot(
        'field.header.stamp',
        'field.pose.position.y',
        data=amcl,
        marker='',
        color='#414a4c',
        linewidth=2,
        linestyle='dashed',
        label="MCL estimation")
    plt.plot(
        'field.header.stamp',
        'field.pose.pose.position.y',
        data=truth,
        marker='',
        color='#232b2b',
        linewidth=2,
        linestyle='solid',
        label="Ground truth")
    plt.legend()
    plt.xlabel('time (seconds)')
    plt.ylabel('y-position')
    plt.title('Position in Y axis')

    ############## z position ##############
    plt.subplot(2, 2, 3)
    plt.plot(
        'field.header.stamp',
        'field.pose.position.z',
        data=amcl,
        marker='',
        color='#414a4c',
        linewidth=2,
        linestyle='dashed',
        label="MCL estimation")
    plt.plot(
        'field.header.stamp',
        'field.pose.pose.position.z',
        data=truth,
        marker='',
        color='#232b2b',
        linewidth=2,
        linestyle='solid',
        label="Ground truth")
    plt.legend()
    plt.xlabel('time (seconds)')
    plt.ylabel('z-position')
    plt.title('Position in Z axis')

    ############## yaw orientation ##############
    plt.subplot(2, 2, 4)
    plt.plot(
        'field.header.stamp',
        'field.pose.orientation.yaw',
        data=amcl,
        marker='',
        color='#414a4c',
        linewidth=2,
        linestyle='dashed',
        label="MCL estimation")
    plt.plot(
        'field.header.stamp',
        'field.pose.pose.orientation.yaw',
        data=truth,
        marker='',
        color='#232b2b',
        linewidth=2,
        linestyle='solid',
        label="Ground truth")
    plt.legend()
    plt.axis([None, None, None, None])
    plt.xlabel('time (seconds)')
    plt.ylabel('yaw')
    plt.title('Orientation')

    ############## General figure settings ##############
    fig.suptitle(
        'Ground truth vs MCL estimates : ' + world.capitalize() + ', ' + speed
        + ' speed, ' + movement + ' movement',
        size=22)
    fig.subplots_adjust(hspace=0.2)
    # plt.savefig(path + '/' + world + '_' + speed + '_' + movement + '_' +
    # attempt + '.png')
    plt.show()


def calculateMetrics(dataframe):
    calculateMean(dataframe)
    calculateMedian(dataframe)
    calculateMin(dataframe)
    calculateMax(dataframe)
    calculateSSE(dataframe)
    calculateSTD(dataframe)
    calculateRMSE(dataframe)


def main():

    # ROSbag path
    if len(sys.argv) != 3:
        print "[USAGE] python evaluate_localization <file_with_ground_truth_values.csv> <file_with_amcl_values.csv>"
        sys.exit()

    truth_path = sys.argv[-2]
    amcl_path = sys.argv[-1]

    # Get the number of attempt /path/to/file/X_truth.csv, where X is what we ask for
    global path
    path = os.path.dirname(truth_path)

    path_list = truth_path.split(os.sep)

    global attempt
    attempt = path_list[-1][0]

    # Get the characteristics of each case
    global world
    global speed
    global movement
    world = path_list[-2]
    speed = path_list[-3]
    movement = path_list[-4]

    print
    print world.capitalize(
    ) + ', ' + speed + ' speed, ' + movement + ' movement : attempt #' + attempt

    # Read CSV files
    truth = pd.read_csv(truth_path)
    amcl = pd.read_csv(amcl_path)

    # Create and fill a column with yaw values, instead of quaternion z, y, z, w
    truth['field.pose.pose.orientation.yaw'] = pd.Series()
    amcl['field.pose.orientation.yaw'] = pd.Series()

    for index, row in amcl.iterrows():
        amcl.at[index, 'field.pose.orientation.yaw'] = quaternionToYaw(
            amcl['field.pose.orientation.x'][index],
            amcl['field.pose.orientation.y'][index],
            amcl['field.pose.orientation.z'][index],
            amcl['field.pose.orientation.w'][index])

    for index, row in truth.iterrows():
        truth.at[index, 'field.pose.pose.orientation.yaw'] = quaternionToYaw(
            truth['field.pose.pose.orientation.x'][index],
            truth['field.pose.pose.orientation.y'][index],
            truth['field.pose.pose.orientation.z'][index],
            truth['field.pose.pose.orientation.w'][index])

    # Convert time stamps to float and then to seconds from nsecs
    truth['field.header.stamp'] = pd.to_numeric(
        truth['field.header.stamp'], downcast='float') * 10**(-9)
    amcl['field.header.stamp'] = pd.to_numeric(
        amcl['field.header.stamp'], downcast='float') * 10**(-9)

    # Create a df for saving error values with initial size the size of AMCL df
    error_df = pd.DataFrame(
        columns=['timestamp', 'error_x', 'error_y', 'error_z', 'error_yaw'])

    starting_index = 0
    for amcl_index in range(0, len(amcl)):

        # Keeping the message that is closest in timestamp with the amcl message
        for truth_index in range(starting_index, len(truth)):
            if truth['field.header.stamp'][truth_index] <= amcl[
                    'field.header.stamp'][amcl_index]:
                starting_index = truth_index
            else:
                break

        # Ground truth value minus AMCL estimation
        error_df.at[amcl_index,
                    'timestamp'] = amcl['field.header.stamp'][amcl_index]
        error_df.at[amcl_index, 'error_x'] = abs(
            truth['field.pose.pose.position.x'][starting_index] -
            amcl['field.pose.position.x'][amcl_index])
        error_df.at[amcl_index, 'error_y'] = abs(
            truth['field.pose.pose.position.y'][starting_index] -
            amcl['field.pose.position.y'][amcl_index])
        error_df.at[amcl_index, 'error_z'] = abs(
            truth['field.pose.pose.position.z'][starting_index] -
            amcl['field.pose.position.z'][amcl_index])
        error_df.at[amcl_index, 'error_yaw'] = abs(
            truth['field.pose.pose.orientation.yaw'][starting_index] -
            amcl['field.pose.orientation.yaw'][amcl_index])

    # Calculate the RMS error and save it in a new column
    for index in range(0, len(error_df)):
        error_df.at[index, 'rms_error'] = np.sqrt(
            error_df['error_x'][index]**2 + error_df['error_y'][index]**2 +
            error_df['error_z'][index]**2)

    # print(error_df.head())

    calculateMetrics(error_df)

    # Write the results in a .csv file
    print position.head()
    # with open(
    # path + '/position' + '_' + world + '_' + speed + '_' + movement +
    # '.csv', 'a') as f:
    # position.to_csv(f, header=False)

    print orientation.head()
    # with open(
    # path + '/orientation' + '_' + world + '_' + speed + '_' + movement
    # + '.csv', 'a') as f:
    # orientation.to_csv(f, header=False)

    plot(amcl, truth)


if __name__ == '__main__':
    main()
