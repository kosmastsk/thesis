# Experiments

## Overview

This directory contains the scripts that were used to evaluate the localization and the coverage methods that have been developed. It requires the data to be available in .csv format. In order to convert ROS bag files to .csv, you need to run for every topic that exists in the bag, the following:

	rostopic echo -b filename.bag -p /topic/name > topicname.csv

### Localization
Run the script with

	python evaluation_localization.py <true_pose>.csv <amcl_pose>.csv

The first .csv file contains a recording of the **`/ground_truth/state`** *(nav_msgs/Odometry)* topic and the second a recording of **`/amcl_pose`** *(geometry_msgs/PoseStamped)*. It calculated the Mean, Median, Min, Max, SSE, STD, RMSE of the localization error. Also, plots the true and the estimated trajectory of the drone with regards to time.

The **`iterator.sh`** file is running the above script for every .csv file that contains the essential files in the following paths: */type/speed/world/attempt*, where:
* **`type`** can be meander, spiral or line
* **`speed`** can be slow, fast or normal
* **`world`** can be box, corridor or warehouse
* **`attempt`** can be 1 up to 5

### Coverage
Run the script with

	python evaluate_coverage.py <volume>.csv

The .csv file contains the recording of the **`/octomap_covered/volume`** *(drone_gazebo/Float64Stamped)* topic. The volume of the total environment is considered known and is defined as a variable inside the code. The output is a plot of the percentage covered with regards to time.

The **compare_coverage.py** can be used to compare the percentage covered when 2 different recordings are available.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/kosmastsk/thesis/issues).
