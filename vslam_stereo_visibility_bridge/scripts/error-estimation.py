#!/usr/bin/env python
#
# This script computes the error drift, i.e. the planned robot position w.r.t
# to the real one.
#
# Practically speaking, this uses three transformations:
# (1) /plan -> /base_link - planning
# (2) /map -> /base_link - map
# (3) /world -> /base_link - control
#
# (1) is the initial movement computed during the planning phase.
# We want to follow this plan as precisely as possible.
#
# (2) is the perceived movement computed by the localization node.
# It takes into account the feedback on the movement execution
# and the drift / errors in the execution.
#
# (3) is the robot position in the control framework, it takes into
# account the correction but not the drift.
# If no correction is applied, it is equal to the planning frame.
#
# We use tf to retrieve these information and realize the following
# computation:
#
# \hat{bl} M bl = \hat{bl} M w . w M bl
#
# \hat{bl} M w is (2)^{-1} and w M bl is (1)
#
# From \hat{bl} M bl, the X, Y and raw components are extracted and
# streamed.


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

rospy.init_node('error_estimator')


# Frame ids.
baseLinkMapFrameId = rospy.get_param(
    '~base_link_map_frame_id', '/mocap_world/left_foot/origin') #FIXME:
baseLinkPlanFrameId = rospy.get_param(
    '~base_link_plan_frame_id', '/plan_left_ankle') #FIXME:
mapFrameId = rospy.get_param('~map_frame_id', '/world')
planFrameId = rospy.get_param('~plan_frame_id', '/world')
timeOffset = rospy.get_param('~offset', 0.) #FIXME:
nullifyErrorAtStartup = rospy.get_param('~nullify_error_at_startup', False)

banner  =  "Starting error estimation.\n"
banner += "* base link (map):   {0}\n".format(baseLinkMapFrameId)
banner += "* base link (world): {0}\n".format(baseLinkPlanFrameId)
banner += "* map frame:         {0}\n".format(mapFrameId)
banner += "* plan frame:        {0}\n".format(planFrameId)
banner += "* time offset:       {0}\n".format(timeOffset)
banner += "* nullify error at startup: {0}\n".format(nullifyErrorAtStartup)
rospy.loginfo(banner)

tl = tf.TransformListener(True, rospy.Duration(10.))

pub = rospy.Publisher('error', Vector3Stamped)

pubDbgMap = rospy.Publisher('error_mapPose', TransformStamped)
pubDbgPlan = rospy.Publisher('error_planPose', TransformStamped)

error = Vector3Stamped()
error.header.seq = 0
error.header.frame_id = baseLinkPlanFrameId

dbgMap = TransformStamped()
dbgMap.header.seq = 0
dbgMap.header.frame_id = mapFrameId
dbgPlan = TransformStamped()
dbgPlan.header.seq = 0
dbgPlan.header.frame_id = planFrameId

initialError = np.identity(4)


ok = False
rospy.loginfo("Waiting for frames...")
rate = rospy.Rate(.1)
t = rospy.Time(0)
while not ok and not rospy.is_shutdown():
    try:
        tl.waitForTransform(
            planFrameId, baseLinkPlanFrameId,
            t, rospy.Duration(0.1))
        tl.waitForTransform(
            mapFrameId, baseLinkMapFrameId,
            t, rospy.Duration(0.1))
        ok = True
    except tf.Exception as e:
        rospy.logwarn("error while waiting for frames: {0}".format(e))
        ok = False
        rate.sleep()
if rospy.is_shutdown():
    sys.exit(0)

rospy.loginfo("started")

# This delay has been experimentally setup for the LAAS motion capture
# system.
offsetPlan = rospy.Duration(timeOffset)

rate = rospy.Rate(10.)
while not rospy.is_shutdown():
    rate.sleep()

    tMap = tl.getLatestCommonTime(mapFrameId,
                                  baseLinkMapFrameId)
    tPlan = tl.getLatestCommonTime(planFrameId,
                                   baseLinkPlanFrameId)

    # Take the min to make sure that we have data for both.
    tPlan = tMap = min(tMap, tPlan)
    # Then subscribe an optional offset.
    tPlan += offsetPlan
    try:
        (wMhbl_t, wMhbl_q) = tl.lookupTransform(
            mapFrameId, baseLinkMapFrameId, tMap)
        (wMbl_t, wMbl_q) = tl.lookupTransform(
            planFrameId, baseLinkPlanFrameId, tPlan)
    except Exception as e:
        rospy.logwarn(e)
        continue

    wMhbl = np.matrix(tl.fromTranslationRotation(wMhbl_t, wMhbl_q))

    wMbl = np.matrix(tl.fromTranslationRotation(wMbl_t, wMbl_q))

    hblMbl = np.linalg.inv(wMhbl) * wMbl

    # first time the error is computed, we nullify it if required.
    if nullifyErrorAtStartup:
        if error.header.seq == 0:
            initialError = np.linalg.inv(hblMbl)
        hblMbl = initialError * hblMbl

    error.header.seq += 1
    error.header.stamp = tMap # Here we used tMap to ignore offset.
    error.vector.x = hblMbl[0, 3]
    error.vector.y = hblMbl[1, 3]
    error.vector.z = atan2(hblMbl[1, 0], hblMbl[0, 0])

    pub.publish(error)


    dbgMap.header.seq += 1
    dbgMap.header.stamp = tMap
    dbgMap.transform.translation.x = wMhbl_t[0]
    dbgMap.transform.translation.y = wMhbl_t[1]
    dbgMap.transform.translation.z = wMhbl_t[2]
    dbgMap.transform.rotation.x = wMhbl_q[0]
    dbgMap.transform.rotation.y = wMhbl_q[1]
    dbgMap.transform.rotation.z = wMhbl_q[2]
    dbgMap.transform.rotation.w = wMhbl_q[3]

    dbgPlan.header.seq += 1
    dbgPlan.header.stamp = tPlan
    dbgPlan.transform.translation.x = wMbl_t[0]
    dbgPlan.transform.translation.y = wMbl_t[1]
    dbgPlan.transform.translation.z = wMbl_t[2]
    dbgPlan.transform.rotation.x = wMbl_q[0]
    dbgPlan.transform.rotation.y = wMbl_q[1]
    dbgPlan.transform.rotation.z = wMbl_q[2]
    dbgPlan.transform.rotation.w = wMbl_q[3]

    pubDbgMap.publish(dbgMap)
    pubDbgPlan.publish(dbgPlan)
