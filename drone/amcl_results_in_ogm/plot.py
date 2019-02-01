# python plot.py home|warehouse
# files are saved in home OR warehouse folder

import plotly
import plotly.graph_objs as go
import sys
import pandas as pd

# Environment
environment = sys.argv[-1]

# Import the ground truth values
truth = pd.read_csv(environment + '/' + environment +  '_ground_truth.csv')
truth_time = truth['%time']
truth_pose_x = truth['field.pose.pose.position.x']
truth_pose_y = truth['field.pose.pose.position.y']

# Import amcl values
amcl = pd.read_csv(environment + '/' + environment + '_amcl_pose.csv')
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
    title= environment,
    xaxis=dict(
        title='x'
    ),
    yaxis=dict(
        title='y'
    )
)

fig = dict(data=data, layout=layout)
plotly.offline.plot(fig, filename=environment + '/' 'compare_' + environment + '.html', auto_open=True)
