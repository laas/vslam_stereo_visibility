#!/usr/bin/env python

import sys
import numpy as np

import roslib
roslib.load_manifest('vslam_stereo_visibility_bridge')
import rospy

import tf
from tf.transformations import translation_from_matrix, \
    quaternion_from_matrix, euler_from_matrix, quaternion_from_euler
import geometry_msgs.msg
import vslam_stereo_visibility_bridge.srv

trans = None
rot = None
listener = None
transformer = None

def computeTransformation(world, slam, camera):
    # Here we do not care about the slamMworld transformation
    # timing as it is constant.
    tWorld = listener.getLatestCommonTime(world, camera)
    tMap = listener.getLatestCommonTime(slam, camera)
    t = min(tWorld, tMap)

    # Current pose given by the SLAM.
    (slamMcam_T, slamMcam_Q) = listener.lookupTransform(camera, slam, t)
    slamMcam = np.matrix(transformer.fromTranslationRotation(slamMcam_T,
                                                             slamMcam_Q))

    # Estimation of the camera position given by control.
    #FIXME: order is weird but it works.
    (worldMcam_T, worldMcam_Q) = listener.lookupTransform(world, camera, t)
    worldMcam = np.matrix(transformer.fromTranslationRotation(worldMcam_T,
                                                              worldMcam_Q))

    (slamMworld_T, slamMworld_Q) = listener.lookupTransform(slam, world, t)
    slamMworld = np.matrix(transformer.fromTranslationRotation(slamMworld_T,
                                                               slamMworld_Q))
    slamMcamEstimated = slamMworld * worldMcam

    # Inverse frames.
    camMslam = np.linalg.inv(slamMcam)
    camMslamEstimated = np.linalg.inv(slamMcamEstimated)

    # Split.
    camMslam_T = translation_from_matrix(camMslam)
    camMslam_Q = quaternion_from_matrix(camMslam)
    camMslam_E = euler_from_matrix(camMslam)

    camMslamEstimated_T = translation_from_matrix(camMslamEstimated)
    camMslamEstimated_Q = quaternion_from_matrix(camMslamEstimated)
    camMslamEstimated_E = euler_from_matrix(camMslamEstimated)

    # Compute correction.
    camMslamCorrected_T = [camMslam_T[0],
                           camMslamEstimated_T[1],
                           camMslam_T[2]]
    camMslamCorrected_E = [camMslamEstimated_E[0],
                           camMslam_E[1],
                           camMslamEstimated_E[2]]

    camMslamCorrected_Q = quaternion_from_euler(*camMslamCorrected_E)

    return (camMslamCorrected_T, camMslamCorrected_Q, t)


if __name__ == '__main__':
    rospy.init_node('tf_combined')
    listener = tf.TransformListener()
    transformer = tf.TransformerROS()

    # Parameters
    camera = rospy.get_param('~camera_frame_id', '/camera_bottom_left_optical')

    world = rospy.get_param('~world_frame_id', '/world')
    slam = rospy.get_param('~map_world_frame_id', '/slam')
    camera_corrected = rospy.get_param('~camera_corrected',
                                       '/camera_corrected')

    rospy.loginfo("camera frame: {0}".format(camera))
    rospy.loginfo("world frame: {0}".format(world))
    rospy.loginfo("SLAM world frame: {0}".format(slam))
    rospy.loginfo("corrected camera frame: {0}".format(camera_corrected))

    t = rospy.Time(0)
    ok = False
    rospy.loginfo("Waiting for frames...")
    rate = rospy.Rate(10.0)
    while not ok and not rospy.is_shutdown():
        try:
            listener.waitForTransform(world, camera, t, rospy.Duration(0.1))
            listener.waitForTransform(slam, camera, t, rospy.Duration(0.1))
            listener.waitForTransform(world, slam, t, rospy.Duration(0.1))
            ok = True
        except tf.Exception as e:
            rospy.logdebug("error while waiting for frames: {0}".format(e))
            ok = False
            rate.sleep()
    if rospy.is_shutdown():
        sys.exit(0)

    if rospy.is_shutdown():
        sys.exit(0)

    tfPub = tf.TransformBroadcaster()
    rate = rospy.Rate(20)
    rospy.loginfo("Streaming frame...")
    while not rospy.is_shutdown():
        try:
            (trans, rot, t) = computeTransformation(world, slam, camera)
            tfPub.sendTransform(trans, rot, t,
                                camera_corrected, slam)
            ok = True
        except tf.ExtrapolationException:
            pass
        rate.sleep()
