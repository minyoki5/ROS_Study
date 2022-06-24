#!/usr/bin/env python

from __future__ import print_function

from mid_project.srv import waypoint,waypointResponse
import rospy

def handle_cal_two_ints(req):
    print("WayPoint Busan[%s]"%( (req.a * req.b)))
    return waypointResponse(req.a * req.b)

def cal_two_ints_server():
    rospy.init_node('GPS')
    service= rospy.Service('waypoint_check', waypoint, handle_cal_two_ints)
    print("Ready")
    rospy.spin()

if __name__ == "__main__":
    cal_two_ints_server()
