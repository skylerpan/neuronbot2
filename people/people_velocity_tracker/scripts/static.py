#!/usr/bin/python

import roslib; roslib.load_manifest('people_velocity_tracker')
import rospy
import sys
from people_msgs.msg import Person, People

class VelocityTracker(object):
    def __init__(self):
        self.ppub = rospy.Publisher('/people', People, queue_size=10)

    def spin(self):
        if len(sys.argv) != 6:
            print("Usage: ./static.py frame_id pos_x pos_y vel_x vel_y\n")
            print("For example: ./static.py map 2.0 1.0 3.0 3.0\n")
            sys.exit()


        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pv = Person()
            pl = People()
            pl.header.stamp = rospy.Time.now()

            # determine which frame's coordinator you rely on, usually set to map or base_link
            pl.header.frame_id = sys.argv[1]
            pv.position.x = float(sys.argv[2])
            pv.position.y = float(sys.argv[3])
            pv.position.z = .5
            pv.velocity.x = float(sys.argv[4])
            pv.velocity.y = float(sys.argv[5])
            pv.name = 'asdf'
            pv.reliability = .90
            pl.people.append(pv)

            self.ppub.publish(pl)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("people_velocity_tracker")
    vt = VelocityTracker()
    try:
        vt.spin()
    except rospy.ROSInterruptException: pass

