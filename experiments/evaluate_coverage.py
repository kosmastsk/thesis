# python evaluate_coverage.py

# INSTRUCTIONS
# You need a rosbag that contains the recording of /octomap_covered/volume
# Convert this rosbag to s csv file using:
# create_coverage_csv /path/to/rosbag/filename (without extension)
# to run the script just run: python evaluate_coverage.py <volume>.csv
# Always adjust the paths to your files and folders
# Don't forget to run chmod +x evaluate_coverage.py before

from __future__ import division
import sys
import os
import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.pylab as pylab

# Initialize
path = '/home'
type = 'narrow'
method = 'slice'
world = 'corridor'
volume = 1


def plot(percentage):
    params = {
        'legend.fontsize': 'x-large',
        'figure.figsize': (18, 10),
        'axes.labelsize': 'x-large',
        'axes.titlesize': 'x-large',
        'xtick.labelsize': 'x-large',
        'ytick.labelsize': 'x-large'
    }
    pylab.rcParams.update(params)
    fig = plt.figure()
    plt.plot(
        'timestamp',
        'percentage',
        data=percentage,
        marker='',
        color='#414a4c',
        linewidth=3,
        linestyle='solid')
    plt.axis([None, None, None, 100])
    plt.grid()
    plt.xlabel('time (seconds)')
    plt.ylabel('% of area covered')
    plt.title(
        'Percentage of area covered : ' + world.capitalize() + ', ' + type +
        ' RFID type, ' + method,
        size=20)
    # plt.savefig(path + '/' + world + '_' + type + '_' + method + '.png')
    plt.show()


def main():

    # ROSbag path
    if len(sys.argv) != 2:
        print "[USAGE] python evaluate_coverage <file_with_coverage_values.csv>"
        sys.exit()

    global path
    path = sys.argv[-1]

    # Read CSV files
    coverage = pd.read_csv(path)

    path_list = path.split(os.sep)
    global type
    global method
    global world

    type = path_list[-2]
    method = path_list[-3]
    world = path_list[-4]

    path = os.path.dirname(path)

    global volume
    if world == "corridor":
        volume = 3.221308
    elif world == "warehouse":
        volume = 10.146877

    # Convert time stamps to float and then to seconds from nsecs
    coverage['field.header.stamp'] = pd.to_numeric(
        coverage['field.header.stamp'], downcast='float') * 10**(-9)

    percentage_df = pd.DataFrame(columns=['timestamp', 'percentage'])

    # Find the non-zero values - coverage had not started
    for index in range(0, len(coverage)):
        if coverage['field.data'][index] != 0:
            starting_index = index
            break

    for index in range(0, len(coverage) - starting_index):
        old_index = index + starting_index
        percentage_df.at[index, 'timestamp'] = coverage['field.header.stamp'][
            old_index]
        percentage_df.at[index, 'percentage'] = 100 * (
            coverage['field.data'][old_index] / volume)

    print
    print '##### ' + world + '  ' + type + '  ' + method + '##### '
    print 'Coverage duration: ' + str(percentage_df['timestamp'][
        len(percentage_df) - 1] - percentage_df['timestamp'][0])
    print 'Percentage covered: ' + str(
        percentage_df['percentage'][len(percentage_df) - 1])
    print

    # print(percentage_df.head())

    plot(percentage_df)


if __name__ == '__main__':
    main()
