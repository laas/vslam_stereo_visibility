#!/usr/bin/env python
import roslib; roslib.load_manifest('vslam_stereo_visibility_bridge')
import rospy

from geometry_msgs.msg import TransformStamped
import tf
import tf.transformations

import numpy as np

transforms = {}

def callback(transformName, msg):
    global transforms
    transforms[transformName] = msg

def main():
    rospy.init_node('evaluate_slam_error')
    rate = rospy.Rate(10)

    worldTransformation = None

    worldTransformation = np.matrix([
            [0., 1., 0., 0.],
            [-1., 0., 0., 0.],
            [0., 0., 1., 0.],
            [0., 0., 0., 1.],
            ])

    mocapMslamMsg = TransformStamped()
    mocapMslamMsg.header.frame_id = '/camera_bottom_top_left_optical'

    tl = tf.TransformListener(True, rospy.Duration(10.))

    rospy.Subscriber("/evart/head/origin", TransformStamped,
                     lambda x: callback("head", x))
    rospy.Subscriber("/slam_node/camera_position", TransformStamped,
                     lambda x: callback("headSlam", x))

    pub = rospy.Publisher('slam_error_estimation', TransformStamped)

    while (not "head" in transforms or \
               transforms["head"].header.seq == 0) and \
            not rospy.is_shutdown():
        rospy.loginfo("waiting for frames head mocap")
        rate.sleep()
    while (not "headSlam" in transforms or \
        transforms["headSlam"].header.seq == 0) and \
        not rospy.is_shutdown():
        rospy.loginfo("waiting for frames headSlam")
        rate.sleep()

    rospy.loginfo("started")
    while not rospy.is_shutdown():
        try:
            wMheadMocap = np.matrix(
                tl.fromTranslationRotation(
                    (transforms["head"].transform.translation.x,
                     transforms["head"].transform.translation.y,
                     transforms["head"].transform.translation.z),
                    (transforms["head"].transform.rotation.x,
                     transforms["head"].transform.rotation.y,
                     transforms["head"].transform.rotation.z,
                     transforms["head"].transform.rotation.w)))

            wMheadSlam = np.matrix(
                tl.fromTranslationRotation(
                    (transforms["headSlam"].transform.translation.x,
                     transforms["headSlam"].transform.translation.y,
                     transforms["headSlam"].transform.translation.z),
                    (transforms["headSlam"].transform.rotation.x,
                     transforms["headSlam"].transform.rotation.y,
                     transforms["headSlam"].transform.rotation.z,
                     transforms["headSlam"].transform.rotation.w)))

#            if worldTransformation == None:
#                worldTransformation = np.linalg.inv(wMheadMocap) * wMheadSlam
#                print(worldTransformation)

#            mocapMslam = \
#                np.linalg.inv(worldTransformation * wMheadMocap) * wMheadSlam

            mocapMslam = \
                worldTransformation * wMheadMocap

            mocapMslam_q = \
                tf.transformations.quaternion_from_matrix(mocapMslam)

            mocapMslamMsg.header.stamp = rospy.Time()
            mocapMslamMsg.transform.translation.x = mocapMslam[0,3]
            mocapMslamMsg.transform.translation.y = mocapMslam[1,3]
            mocapMslamMsg.transform.translation.z = mocapMslam[2,3]
            mocapMslamMsg.transform.rotation.x = mocapMslam_q[0]
            mocapMslamMsg.transform.rotation.y = mocapMslam_q[1]
            mocapMslamMsg.transform.rotation.z = mocapMslam_q[2]
            mocapMslamMsg.transform.rotation.w = mocapMslam_q[3]

            pub.publish(mocapMslamMsg)
            rate.sleep()
        except tf.Exception, e:
            rate.sleep()

main()
