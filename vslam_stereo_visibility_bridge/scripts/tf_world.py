#!/usr/bin/env python
#
# This nodes stream the relative position between the  map world frame and
# the SLAM world frame.
#
# I.e. /world -> control world frame (i.e. dynamic-graph)
#      /slam  -> SLAM world frame (i.e. map dependent)
#      /map   -> localization world frame
#
# We make the assumpation that when this node is started (or when the transformation
# is evaluated), the execution error is null.

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

def computeTransformation(world, slam_world):
    t = listener.getLatestCommonTime(world, slam_world)
    (mapMslam_T, mapMslam_Q) = listener.lookupTransform(slam_world, world, t)
    mapMslam = np.matrix(transformer.fromTranslationRotation(mapMslam_T, mapMslam_Q))
    rospy.loginfo("mapMslam: {0} {1}\n{2}".format(mapMslam_T, mapMslam_Q, mapMslam))
    rospy.loginfo("Slam world to world tf computed: {0} {1}\n{2}".format(
            mapMslam_T, mapMslam_Q, mapMslam))
    return (mapMslam_T, mapMslam_Q)

def evaluate_world_position(world, slam_world, req):
    try:
        (trans, rot) = computeTransformation(world, slam_world)
    except:
        return vslam_stereo_visibility_bridge.srv.EvaluateWorldPositionResponse()
    return vslam_stereo_visibility_bridge.srv.EvaluateWorldPositionResponse()

def evaluate_world_position_bind(world, slam_world):
    return lambda req: evaluate_world_position(world, slam_world, req)

if __name__ == '__main__':
    rospy.init_node('tf_world_broadcaster')
    listener = tf.TransformListener()
    transformer = tf.TransformerROS()

    # Parameters
    world = rospy.get_param('~world_frame_id', '/world')
    slam_world = rospy.get_param('~slam_world_frame_id', '/slam')
    map_world = rospy.get_param('~map_world_frame_id', '/map')

    rospy.loginfo("world frame: {0}".format(world))
    rospy.logdebug("SLAM world frame: {0}".format(slam_world))

    # First transformation evaluation (blocking, unlike the service)
    t = rospy.Time(0)
    ok = False
    rospy.loginfo("Waiting for frames...")
    rate = rospy.Rate(10.0)
    while not ok and not rospy.is_shutdown():
        try:
            listener.waitForTransform(world, slam_world, t, rospy.Duration(0.1))
            ok = True
        except tf.Exception as e:
            rospy.logdebug("error while waiting for frames: {0}".format(e))
            ok = False
            rate.sleep()
    if rospy.is_shutdown():
        sys.exit(0)

    ok = False
    while not ok and not rospy.is_shutdown():
        try:
            (trans, rot) = computeTransformation(world, slam_world)
            ok = True
        except tf.ExtrapolationException:
            ok = False

    if rospy.is_shutdown():
        sys.exit(0)
    s = rospy.Service('evaluate_world_position',
                      vslam_stereo_visibility_bridge.srv.EvaluateWorldPosition,
                      evaluate_world_position_bind(world, slam_world))
    tfPub = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if trans != None and rot != None:
            tfPub.sendTransform(trans, rot, rospy.Time.now(), map_world, slam_world)
        rate.sleep()
