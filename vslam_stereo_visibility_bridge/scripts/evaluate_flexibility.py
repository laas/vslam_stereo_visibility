#!/usr/bin/env python
import roslib; roslib.load_manifest('vslam_stereo_visibility_bridge')
import rospy

from geometry_msgs.msg import TransformStamped
import tf
import tf.transformations

import numpy as np

transforms = {}

def callbackMocap(transformName, msg):
    global transforms
    transforms[transformName] = msg

def main():
    rospy.init_node('evaluate_flexibility')
    rate = rospy.Rate(10)

    commandFrameId = rospy.get_param('command_frame_id', '/world')

    mocapMcommandMsg = TransformStamped()
    mocapMcommandMsg.header.frame_id = '/camera_bottom_top_left_optical'

    tl = tf.TransformListener(True, rospy.Duration(10.))

    rospy.Subscriber("/evart/head/origin", TransformStamped,
                     lambda x: callbackMocap("head", x))
    rospy.Subscriber("/evart/left_foot/origin", TransformStamped,
                     lambda x: callbackMocap("foot", x))

    pub = rospy.Publisher('flexibility_estimation', TransformStamped)

    while not tl.frameExists("/camera_bottom_left_optical") and \
            not rospy.is_shutdown():
        rospy.loginfo("waiting for frames camera_bottom_left_optical")
        rate.sleep()
    while not tl.frameExists("/left_ankle") and \
            not rospy.is_shutdown():
        rospy.loginfo("waiting for frames left_ankle")
        rate.sleep()
    while (not "head" in transforms or \
               transforms["head"].header.seq == 0) and \
            not rospy.is_shutdown():
        rospy.loginfo("waiting for frames head mocap")
        rate.sleep()
    while (not "foot" in transforms or \
        transforms["foot"].header.seq == 0) and \
        not rospy.is_shutdown():
        rospy.loginfo("waiting for frames foot mocap")
        rate.sleep()

    rospy.loginfo("started")
    while not rospy.is_shutdown():
        try:
            tHeadCommand = tl.getLatestCommonTime(
                commandFrameId, "/camera_bottom_left_optical")
            tFootCommand = tl.getLatestCommonTime(
                commandFrameId, "/left_ankle")

            tHeadMocap = transforms["head"].header.stamp
            tFootMocap = transforms["foot"].header.stamp

            t = min(tHeadCommand, tFootCommand)
            t = min(tHeadMocap, t)
            t = min(tFootMocap, t)

            headCommandTransformation = tl.lookupTransform(
                commandFrameId, "/camera_bottom_left_optical", t)
            footCommandTransformation = tl.lookupTransform(
                commandFrameId, "/left_ankle", t)

            wMheadMocap = np.matrix(
                tl.fromTranslationRotation(
                    (transforms["head"].transform.translation.x,
                     transforms["head"].transform.translation.y,
                     transforms["head"].transform.translation.z),
                    (transforms["head"].transform.rotation.x,
                     transforms["head"].transform.rotation.y,
                     transforms["head"].transform.rotation.z,
                     transforms["head"].transform.rotation.w)))
            wMfootMocap = np.matrix(
                tl.fromTranslationRotation(
                    (transforms["foot"].transform.translation.x,
                     transforms["foot"].transform.translation.y,
                     transforms["foot"].transform.translation.z),
                    (transforms["foot"].transform.rotation.x,
                     transforms["foot"].transform.rotation.y,
                     transforms["foot"].transform.rotation.z,
                     transforms["foot"].transform.rotation.w)))

            wMheadCommand = np.matrix(
                tl.fromTranslationRotation(headCommandTransformation[0],
                                           headCommandTransformation[1]))
            wMfootCommand = np.matrix(
                tl.fromTranslationRotation(footCommandTransformation[0],
                                           footCommandTransformation[1]))


            # Compute head relative position w.r.t foot (command)
            headMfootMocap = \
                np.linalg.inv(wMheadMocap) * wMfootMocap
            # Compute head relative position w.r.t foot (mocap)
            headMfootCommand = \
                np.linalg.inv(wMheadCommand) * wMfootCommand

            mocapMcommand = headMfootMocap * np.linalg.inv(headMfootCommand)
            mocapMcommand_q = \
                tf.transformations.quaternion_from_matrix(mocapMcommand)

            mocapMcommandMsg.header.stamp = t
            mocapMcommandMsg.transform.translation.x = mocapMcommand[0,3]
            mocapMcommandMsg.transform.translation.y = mocapMcommand[1,3]
            mocapMcommandMsg.transform.translation.z = mocapMcommand[2,3]
            mocapMcommandMsg.transform.rotation.x = mocapMcommand_q[0]
            mocapMcommandMsg.transform.rotation.y = mocapMcommand_q[1]
            mocapMcommandMsg.transform.rotation.z = mocapMcommand_q[2]
            mocapMcommandMsg.transform.rotation.w = mocapMcommand_q[3]

            pub.publish(mocapMcommandMsg)
            rate.sleep()
        except tf.Exception, e:
            rate.sleep()

main()
