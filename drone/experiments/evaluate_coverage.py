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


def plot(coverage):
    fig = plt.figure()
    plt.plot(
        'field.header.stamp',
        'field.data',
        data=coverage,
        marker='',
        color='#414a4c',
        linewidth=1,
        linestyle='solid')
    plt.axis([0, None, 0, 100])
    plt.xlabel('time (seconds)')
    plt.ylabel('Percentage of nodes covered')
    plt.title('Κάλυψη χώρου συναρτήσει του χρόνου')

    plt.show()


def main():

    # ROSbag path
    if len(sys.argv) != 2:
        print "[USAGE] python evaluate_localization <file_with_coverage_values.csv>"
        sys.exit()

    coverage_path = sys.argv[-1]

    # Read CSV files
    coverage = pd.read_csv(coverage_path)

    # Convert time stamps to float and then to seconds from nsecs
    coverage['field.header.stamp'] = pd.to_numeric(
        coverage['field.header.stamp'], downcast='float') * 10**(-9)

    print(coverage.head())

    plot(coverage)


if __name__ == '__main__':
    main()
