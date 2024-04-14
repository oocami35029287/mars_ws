#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class GlobalMapInflator:
    def __init__(self):
        rospy.init_node('global_map_inflator')
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        self.inflated_map_pub = rospy.Publisher("inflated_map", OccupancyGrid, queue_size=1)
        self.inflated_map = None

    def map_callback(self, msg):
        self.inflated_map = msg

    def inflate_map(self, max_inflation, decay_rate):
        if self.inflated_map is not None:
            inflated_map = self.inflated_map
            width = inflated_map.info.width
            height = inflated_map.info.height
            data_list = list(inflated_map.data)

            for x in range(width):
                for y in range(height):
                    index = x + y * width
                    if inflated_map.data[index] == 100:  # Check if the cell is occupied
                        weight = max_inflation
                        for dx in range(-max_inflation, max_inflation+1):
                            for dy in range(-max_inflation, max_inflation+1):
                                nx = x + dx
                                ny = y + dy
                                if 0 <= nx < width and 0 <= ny < height:
                                    nIndex = nx + ny * width
                                    d = np.sqrt(dx**2 + dy**2)
                                    weight *= 1 / (1 + decay_rate * d)
                        data_list[index] = min(100, data_list[index] * weight)  # Apply weight to map data

            inflated_map.data = tuple(data_list)
            # Publish the inflated map
            self.inflated_map_pub.publish(inflated_map)

if __name__ == '__main__':
    try:
        global_map_inflator = GlobalMapInflator()
        max_inflation = 100  # Maximum inflation value
        decay_rate = 0.1  # Decay rate for inflation
        rate = rospy.Rate(1)  # Adjust the publishing rate as needed
        while not rospy.is_shutdown():
            global_map_inflator.inflate_map(max_inflation, decay_rate)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
