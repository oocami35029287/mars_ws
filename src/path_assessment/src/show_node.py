#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from path_assessment.msg import Assessment
import rospy


# Create a figure and axis object
fig, ax = plt.subplots()
# Set axis limits
ax.set_xlim(0, 10)
ax.set_ylim(0, 100)

# Initialize an empty line object
line, = ax.plot([], [], lw=1)

# Initialize data
x_data = []
y_data = []

# Update function, updates data and line every frame
def update(frame):
    # Update line data
    line.set_data(x_data[:frame], y_data[:frame])
    
    # Set x-axis range
    ax.set_xlim(frame-50, frame )
    
    return line,

# Callback function, receives Assessment message and extracts data
def assessment_callback(msg):
    global x_data, y_data
    t = msg.t
    idx_h = msg.idx_h
    
    # Do some data processing, here is just a simple demonstration, you can modify as needed
    if t > 0 and idx_h is not None:
        x_data.append(t)
        y_data.append(idx_h)
    
    # Update animation
    ani.event_source.stop()
    ani.event_source.start()
    

# Initialize ROS node
rospy.init_node('assessment_subscriber')

# Create a Subscriber to subscribe to the 'assessment_topic' topic with message type Assessment
rospy.Subscriber('assessment_topic', Assessment, assessment_callback)

# Create animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 100), interval=200)

# Set x-axis ticks and labels
seconds_per_frame = 0.1  # seconds per frame
frame_per_second = 1 / seconds_per_frame
num_ticks = 10  # number of ticks
ax.set_xticks(np.linspace(0, 100, num_ticks))
ax.set_xticklabels([str(int(i)) for i in range(num_ticks)])
ax.axhline(y=20, color='r', linestyle='--', label='Threshold')

# Show animation
plt.show()

# Keep waiting for ROS messages
rospy.spin()
