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
type = 'narrow'
world = 'corridor'
# Volume of the whole environment that is already known before
volume = 1


def plot(lift_percentage_df, slice_percentage_df):
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
        data=slice_percentage_df,
        marker='',
        color='#131616',
        linewidth=3,
        linestyle='dashed',
        label='Slice')
    plt.plot(
        'timestamp',
        'percentage',
        data=lift_percentage_df,
        marker='',
        color='#131616',
        linewidth=3,
        linestyle='solid',
        label='Lift')
    plt.legend()
    plt.axis([None, None, None, 100])
    plt.grid()
    plt.xlabel('time (seconds)')
    plt.ylabel('% of area covered')
    plt.title(world.capitalize() + ', ' + type + ' RFID sensor', size=20)
    plt.savefig(world + '_' + type + '.png')
    # plt.show()


def main():

    # ROSbag path
    if len(sys.argv) != 3:
        print "[USAGE] python evaluate_coverage <file_with_coverage_values_slice.csv> <file_with_coverage_values_lift.csv>"
        sys.exit()

    lift_path = sys.argv[-1]
    slice_path = sys.argv[-2]

    # Read CSV files
    lift_coverage = pd.read_csv(lift_path)
    slice_coverage = pd.read_csv(slice_path)

    path_list = lift_path.split(os.sep)
    global type
    global world

    type = path_list[-2]
    world = path_list[-4]
    global volume
    if world == "corridor":
        volume = 3.221308
    elif world == "warehouse":
        volume = 10.146877

    # Convert time stamps to float and then to seconds from nsecs
    lift_coverage['field.header.stamp'] = pd.to_numeric(
        lift_coverage['field.header.stamp'], downcast='float') * 10**(-9)
    slice_coverage['field.header.stamp'] = pd.to_numeric(
        slice_coverage['field.header.stamp'], downcast='float') * 10**(-9)

    lift_percentage_df = pd.DataFrame(columns=['timestamp', 'percentage'])
    slice_percentage_df = pd.DataFrame(columns=['timestamp', 'percentage'])

    for index in range(0, len(lift_coverage)):
        lift_percentage_df.at[index, 'timestamp'] = lift_coverage[
            'field.header.stamp'][index]
        lift_percentage_df.at[index, 'percentage'] = 100 * (
            lift_coverage['field.data'][index] / volume)

    for index in range(0, len(slice_coverage)):
        slice_percentage_df.at[index, 'timestamp'] = slice_coverage[
            'field.header.stamp'][index]
        slice_percentage_df.at[index, 'percentage'] = 100 * (
            slice_coverage['field.data'][index] / volume)

    print
    print '##### ' + world + ' ' + type + ' #####'
    print

    # print(percentage_df.head())

    plot(lift_percentage_df, slice_percentage_df)


if __name__ == '__main__':
    main()
