# python plot.py home|warehouse
# files are saved in home OR warehouse folder

# INSTRUCTIONS
# You need a rosbag that contains the recording of /amcl_pose and /ground_truth/state topics
# Convert this rosbag to 2 different csv files using:
# rostopic echo -b file.bag -p /topic > filename.csv
# to run the script just run: python amcl_vs_ground_truth.py
# Always adjust the paths to your files and folders
# Don't forget to run chmod +x amcl_vs_ground_truth.py before

import plotly
import plotly.graph_objs as go
import sys
import pandas as pd

# Environment
environment = sys.argv[-2] # home OR warehouse
truth_vs_odom = sys.argv[-1] # truth OR odom

# Import the ground truth values
truth = pd.read_csv(environment + '/' + environment + '_' + truth_vs_odom +  '_ground_truth.csv')
truth_time = truth['%time']
truth_pose_x = truth['field.pose.pose.position.x']
truth_pose_y = truth['field.pose.pose.position.y']

# Import amcl values
amcl = pd.read_csv(environment + '/' + environment + '_' + truth_vs_odom + '_amcl_pose.csv')
amcl_time = amcl['%time']
amcl_pose_x = amcl['field.pose.pose.position.x']
amcl_pose_y = amcl['field.pose.pose.position.y']

# Ground Truth values trace
trace_truth = go.Scatter(
    x = truth_pose_x,
    y = truth_pose_y,
    name = 'Ground truth',
    mode = 'markers'
)

# AMCL values trace
trace_amcl = go.Scatter(
    x = amcl_pose_x,
    y = amcl_pose_y,
    name = 'AMCL',
    mode = 'markers'
)

data = [trace_truth, trace_amcl]

layout = go.Layout(
    title= environment + ' given ' + truth_vs_odom,
    xaxis=dict(
        title='x'
    ),
    yaxis=dict(
        title='y'
    )
)

fig = dict(data=data, layout=layout)
plotly.offline.plot(fig, filename=environment + '/' 'compare_' + truth_vs_odom + '_' + environment + '.html', auto_open=True)
