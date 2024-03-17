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
        # === kinematic control =====
        car.x_e = round(random.uniform(0, 5),2)
        car.y_e = round(random.uniform(0, 5),2)
        car.psi_e = round(random.uniform(psi_min, psi_max),2) # also for kinematic scheduling variable
        # for reference r_c
        car.x_dot_ref = round(random.uniform(xr_dot_min, xr_dot_max),2) # also for kinematic scheduling variable
        car.psi_dot_ref = round(random.uniform(0, 5),2)
        # === dynamic control ======
        car.x_dot=round(random.uniform(x_dot_min, x_dot_max),2) # also for dynamic scheduling variable
        car.y_dot=round(random.uniform(y_dot_min, y_dot_max),2) # also for dynamic scheduling variable
        car.psi_dot = round(random.uniform(psi_dot_min, psi_dot_max),2) # also for kinematic scheduling variable
        
        car.delta=round(random.uniform(delta_min, delta_max),2) # also for dynamic scheduling variable

        pub.publish(car)

        log_message = "x_e: %s, y_e: %s, psi_e: %s,x_dot_ref: %s, psi_dot_ref: %s, x_dot: %s, y_dot: %s, psi_dot: %s, delta: %s" % (car.x_e, car.y_e,car.psi_e, car.x_dot_ref,car.psi_dot_ref,car.x_dot,car.y_dot,car.psi_dot,car.delta)
        rospy.loginfo(log_message)
        rate.sleep()