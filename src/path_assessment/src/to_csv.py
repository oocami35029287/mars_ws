#!/usr/bin/env python

import rospy
import signal
import pandas as pd
from std_msgs.msg import Header
from path_assessment.msg import Assessment  # replace "your_package" with the actual package name

class TopicSubscriber:
    def __init__(self):
        rospy.init_node('topic_subscriber', anonymous=True)
        rospy.Subscriber("path_assessment", Assessment, self.callback)
        self.data = []

        # Set up signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

    def callback(self, data):
        row = {
            'Header': data.header,
            'stop': data.stop,
            't': data.t,
            'robot_x': data.robot_x,
            'robot_y': data.robot_y,
            'eva_cur_h': data.eva_cur_h,
            'idx_h': data.idx_h,
            'eva_avg_h': data.eva_avg_h,
            'colli_t_h': data.colli_t_h,
            'colli_h': data.colli_h,
            'eva_cur_o': data.eva_cur_o,
            'idx_o': data.idx_o,
            'eva_avg_o': data.eva_avg_o,
            'colli_t_o': data.colli_t_o,
            'colli_o': data.colli_o
        }
        self.data.append(row)
        print("adding")

    def write_to_csv(self, filename):
        df = pd.DataFrame(self.data)
        df.to_csv(filename, index=False)

    def signal_handler(self, sig, frame):
        print("Ctrl+C detected, writing to CSV...")
        self.write_to_csv('./mars_ws/src/path_assessment/src/data/assessment_data.csv')
        rospy.signal_shutdown("Shutdown signal received")

if __name__ == '__main__':
    subscriber = TopicSubscriber()
    rospy.spin()

