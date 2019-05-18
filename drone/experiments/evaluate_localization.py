# python plot.py

# INSTRUCTIONS
# You need a rosbag that contains the recording of /amcl_pose and /ground_truth/state topics
# Convert this rosbag to 2 different csv files using:
# rostopic echo -b file.bag -p /topic > filename.csv
# to run the script just run: python amcl_vs_ground_truth.py
# Always adjust the paths to your files and folders
# Don't forget to run chmod +x amcl_vs_ground_truth.py before

import sys
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt


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
    print
    print '################## mean ##################'
    print 'Mean error in X: ' + str(dataframe['error_x'].mean()) + ' (metres)'
    print 'Mean error in Y: ' + str(dataframe['error_y'].mean()) + ' (metres)'
    print 'Mean error in Z: ' + str(dataframe['error_z'].mean()) + ' (metres)'
    print 'Mean error: ' + str(dataframe['rms_error'].mean()) + ' (metres)'
    print 'Mean error in Yaw: ' + str(dataframe['error_yaw'].mean()) + ' (rad)'


def calculateMedian(dataframe):
    print
    print '################## median ##################'
    print 'Median error in X: ' + str(
        dataframe['error_x'].median()) + ' (metres)'
    print 'Median error in Y: ' + str(
        dataframe['error_y'].median()) + ' (metres)'
    print 'Median error in Z: ' + str(
        dataframe['error_z'].median()) + ' (metres)'
    print 'Median error: ' + str(dataframe['rms_error'].median()) + ' (metres)'
    print 'Median error in Yaw: ' + str(
        dataframe['error_yaw'].median()) + ' (rad)'


# Min value
def calculateMin(dataframe):
    print
    print '################## min ##################'
    print 'Min error in X: ' + str(dataframe['error_x'].min()) + ' (metres)'
    print 'Min error in Y: ' + str(dataframe['error_y'].min()) + ' (metres)'
    print 'Min error in Z: ' + str(dataframe['error_z'].min()) + ' (metres)'
    print 'Min error: ' + str(dataframe['rms_error'].min()) + ' (metres)'
    print 'Min error in Yaw: ' + str(dataframe['error_yaw'].min()) + ' (rad)'


# Max value
def calculateMax(dataframe):
    print
    print '################## max ##################'
    print 'Max error in X: ' + str(dataframe['error_x'].max()) + ' (metres)'
    print 'Max error in Y: ' + str(dataframe['error_y'].max()) + ' (metres)'
    print 'Max error in Z: ' + str(dataframe['error_z'].max()) + ' (metres)'
    print 'Max error: ' + str(dataframe['rms_error'].max()) + ' (metres)'
    print 'Max error in Yaw: ' + str(dataframe['error_yaw'].max()) + ' (rad)'


# Root Mean Squared Errors
def calculateRMSE(dataframe):
    print
    print '################## RMSE ##################'
    print 'RMSE in X: ' + str(
        (dataframe['error_x']**2).mean()**.5) + ' (metres)'
    print 'RMSE in Y: ' + str(
        (dataframe['error_y']**2).mean()**.5) + ' (metres)'
    print 'RMSE in Z: ' + str(
        (dataframe['error_z']**2).mean()**.5) + ' (metres)'
    print 'RMSE: ' + str((dataframe['rms_error']**2).mean()**.5) + ' (metres)'
    print 'RMSE in Yaw: ' + str(
        (dataframe['error_yaw']**2).mean()**.5) + ' (rad)'


# Sum of Squared Errors
def calculateSSE(dataframe):
    print
    print '################## SSE ##################'
    print 'SSE in X: ' + str(np.sum((dataframe['error_x']**2))) + ' (metres)'
    print 'SSE in Y: ' + str(np.sum((dataframe['error_y']**2))) + ' (metres)'
    print 'SSE in Z: ' + str(np.sum((dataframe['error_z']**2))) + ' (metres)'
    print 'SSE: ' + str(np.sum((dataframe['rms_error']**2))) + ' (metres)'
    print 'SSE in Yaw: ' + str(np.sum((dataframe['error_yaw']**2))) + ' (rad)'


# Standard deviation
def calculateSTD(dataframe):
    print
    print '################## STD ##################'
    print 'STD error in X: ' + str(dataframe['error_x'].std()) + ' (metres)'
    print 'STD error in Y: ' + str(dataframe['error_y'].std()) + ' (metres)'
    print 'STD error in Z: ' + str(dataframe['error_z'].std()) + ' (metres)'
    print 'STD error: ' + str(dataframe['rms_error'].std()) + ' (metres)'
    print 'STD error in Yaw: ' + str(dataframe['error_yaw'].std()) + ' (rad)'


def plot(amcl, truth):
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
    plt.xlabel('time (seconds)')
    plt.ylabel('yaw')
    plt.title('Orientation')

    ############## General figure settings ##############
    fig.suptitle('Ground truth - MCL estimates', size=16)
    fig.subplots_adjust(hspace=0.2)
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
        counter = 0
        accumulated_position_x = 0
        accumulated_position_y = 0
        accumulated_position_z = 0
        accumulated_position_yaw = 0

        # Interpolate the measurements between timestamps
        for truth_index in range(starting_index, len(truth)):
            if truth['field.header.stamp'][truth_index] <= amcl[
                    'field.header.stamp'][amcl_index]:
                starting_index = truth_index
                counter += 1
                accumulated_position_x += truth['field.pose.pose.position.x'][
                    truth_index]
                accumulated_position_y += truth['field.pose.pose.position.y'][
                    truth_index]
                accumulated_position_z += truth['field.pose.pose.position.z'][
                    truth_index]
                accumulated_position_yaw += truth[
                    'field.pose.pose.orientation.yaw'][truth_index]
            else:
                break

        # Mean ground truth value minus AMCL estimation
        error_df.at[amcl_index,
                    'timestamp'] = amcl['field.header.stamp'][amcl_index]
        error_df.at[amcl_index, 'error_x'] = abs(
            (accumulated_position_x / counter) -
            amcl['field.pose.position.x'][amcl_index])
        error_df.at[amcl_index, 'error_y'] = abs(
            (accumulated_position_y / counter) -
            amcl['field.pose.position.y'][amcl_index])
        error_df.at[amcl_index, 'error_z'] = abs(
            (accumulated_position_z / counter) -
            amcl['field.pose.position.z'][amcl_index])
        error_df.at[amcl_index, 'error_yaw'] = abs(
            (accumulated_position_yaw / counter) -
            amcl['field.pose.orientation.yaw'][amcl_index])

    # Calculate the RMS error and save it in a new column
    for index in range(0, len(error_df)):
        error_df.at[index, 'rms_error'] = np.sqrt(
            error_df['error_x'][index]**2 + error_df['error_y'][index]**2 +
            error_df['error_z'][index]**2)

    # print(error_df.head())

    calculateMetrics(error_df)
    plot(amcl, truth)


if __name__ == '__main__':
    main()
