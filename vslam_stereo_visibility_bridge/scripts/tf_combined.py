#!/usr/bin/env python

import sys
import numpy as np

import roslib
roslib.load_manifest('vslam_stereo_visibility_bridge')
import rospy

import tf
from tf.transformations import translation_from_matrix, quaternion_from_matrix
import geometry_msgs.msg
import vslam_stereo_visibility_bridge.srv

trans = None
rot = None
listener = None
transformer = None

def computeTransformation(world, map_world, camera):
    tWorld = listener.getLatestCommonTime(world, camera)
    tMap = listener.getLatestCommonTime(map_world, camera)
    t = min(tWorld, tMap)

    (worldMcam_T, worldMcam_Q) = listener.lookupTransform(camera, world, t)
    worldMcam = np.matrix(transformer.fromTranslationRotation(worldMcam_T, worldMcam_Q))

    (mapMcam_T, mapMcam_Q) = listener.lookupTransform(camera, map_world, t)
    mapMcam = np.matrix(transformer.fromTranslationRotation(mapMcam_T, mapMcam_Q))

    mapCombinedMcamera_T = [mapMcam_T[0], mapMcam_T[1], worldMcam_T[2]]
    mapCombinedMcamera_Q = mapMcam_Q

    return (mapCombinedMcamera_T, mapCombinedMcamera_Q, t)

if __name__ == '__main__':
    rospy.init_node('tf_combined')
    listener = tf.TransformListener()
    transformer = tf.TransformerROS()

    # Parameters
    world = rospy.get_param('~world_frame_id', '/world')
    map_world = rospy.get_param('~map_world_frame_id', '/map')
    map_combined_world = rospy.get_param('~map_combined_world_frame_id',
                                         '/map_combined')
    camera = rospy.get_param('~camera_frame_id', '/camera_bottom_left_optical')

    #map_combined_world = '/lol_combined'

    rospy.logdebug("world frame: {0}".format(world))
    rospy.logdebug("map world frame: {0}".format(map_world))
    rospy.logdebug("map combined world frame: {0}".format(map_combined_world))

    t = rospy.Time(0)
    ok = False
    rospy.loginfo("Waiting for frames...")
    rate = rospy.Rate(10.0)
    while not ok and not rospy.is_shutdown():
        try:
            listener.waitForTransform(world, camera, t, rospy.Duration(0.1))
            listener.waitForTransform(map_world, camera, t, rospy.Duration(0.1))
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
            (trans, rot, t) = computeTransformation(world, map_world,camera)
            tfPub.sendTransform(trans, rot, t,
                                map_combined_world, camera)
            ok = True
        except tf.ExtrapolationException:
            pass
        rate.sleep()
