#!/usr/bin/env python

from __future__ import print_function

import sys
from math import atan2
import numpy as np
import roslib; roslib.load_manifest('dynamic_graph_bridge')
import rospy

import tf
from geometry_msgs.msg import TransformStamped, Vector3Stamped

from tf.transformations import quaternion_from_matrix, \
    translation_from_matrix, quaternion_from_matrix

rospy.init_node('merge_errors')

errors = {}

pub = rospy.Publisher('error', Vector3Stamped)

def cb(var, data):
    global errors
    errors[var] = data

subAr = rospy.Subscriber('/ar/error', Vector3Stamped,
                         lambda data: cb("ar", data))
subSlam = rospy.Subscriber('/slam/error', Vector3Stamped,
                           lambda data: cb("slam", data))

while not rospy.is_shutdown():
    if "ar" in errors or "slam" in errors:
        if not "ar" in errors:
            pub.publish(errors["slam"])
        elif not "slam" in errors:
            pub.publish(errors["ar"])
        elif np.linalg.norm(errors["ar"].vector) < np.linalg.norm(errors["slam"].vector):
            pub.publish(errors["ar"])
        else:
            pub.publish(errors["slam"])
    rospy.sleep(0.1)
