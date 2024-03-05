#!/usr/bin/env python3
import rospy
import random
from autonomous_vehicle.msg import state
from constants import *





if __name__=='__main__':
    rospy.init_node("state_publisher")
    rospy.loginfo("Node has been started")
    pub = rospy.Publisher("/car/state", state, queue_size=10 )
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        car = state()
        car.xr_dot = round(random.uniform(xr_dot_min, xr_dot_max),2)
        car.psi_dot = round(random.uniform(psi_dot_min, psi_dot_max),2)
        car.psi = round(random.uniform(psi_min, psi_max),2)

        car.x_dot=round(random.uniform(x_dot_min, x_dot_max),2)
        car.y_dot=round(random.uniform(y_dot_min, y_dot_max),2)
        car.delta=random.randint(delta_min, delta_max) # sudut steering didapat dari data servo

        pub.publish(car)

        log_message = "xr_dot: %s, psi_dot: %s, psi: %s,x_dot: %s, y_dot: %s, delta: %s" % (car.xr_dot, car.psi_dot,car.psi, car.x_dot,car.y_dot,car.delta)
        rospy.loginfo(log_message)
        rate.sleep()