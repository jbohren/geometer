#!/usr/bin/env python

from math import sin

import rospy

from geometer import Geometer

def main():
    rospy.init_node('geometer')

    g = Geometer()

    g.point('world', [1,1,1], 0.01)
    g.line('world', [1,1,1], [1,1,1], [0.0,2.0])
    g.plane('world', [1,1,1], [1,1,1], 1, key='disc')


    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        g.plane('world', [2,2,2], [1,1,1], 2+sin(rospy.Time.now().to_sec()), key='disco')
        g.publish()
        r.sleep()

if __name__ == '__main__':
    main()

