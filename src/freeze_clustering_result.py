#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    rospy.init_node('freeze_clustering_result')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    br = tf2_ros.tf2_ros.StaticTransformBroadcaster()
    object_base_name = "object"
    object_num = 1
    object_name = object_base_name + "_" + str(object_num)
    objects_trans = {}

    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform(
                "world", object_name, rospy.Time(0), rospy.Duration(1.0))
            objects_trans[object_name] = trans
            print("{} found.".format(object_name))
            object_num += 1
            object_name = object_base_name + "_" + str(object_num)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            break

        for name, trans in objects_trans.items():
            trans.header.child_frame_id = name + "_" + "static"
            br.sendTransform(trans)
        rospy.spin()
