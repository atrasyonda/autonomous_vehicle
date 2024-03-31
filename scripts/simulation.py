#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import threading
initial_value = 0

def callback(data):
    global initial_value
    initial_value = data.data

def publisher():
    global initial_value
    pub = rospy.Publisher('topic_a', Int32, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(1) # Publish setiap 1 detik

    while not rospy.is_shutdown():
        pub.publish(initial_value)
        rospy.loginfo("Publishing value: %d", initial_value)
        rate.sleep()

def subscriber():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber('topic_b', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        # Start subscriber
        subscriber_thread = threading.Thread(target=subscriber)
        subscriber_thread.start()

        # Start publisher
        publisher()
    except rospy.ROSInterruptException:
        pass
