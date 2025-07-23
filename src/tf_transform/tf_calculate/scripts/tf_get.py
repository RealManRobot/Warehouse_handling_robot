#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

import rospy    # ROS模块
import tf
import tf.transformations    # tf坐标处理模块


def get_tf_coor(base_frame_id, child_frame_id):

    # 监听target对于base的相对关系
    tf_listener = tf.TransformListener()

    tuple_frame_id = (base_frame_id, child_frame_id)
    # 等待tf树中的数据变得可用
    tf_listener.waitForTransform(*tuple_frame_id, rospy.Time(0), rospy.Duration(1.0))
    print(base_frame_id, "---->", child_frame_id)
    (trans, rot) = tf_listener.lookupTransform(*tuple_frame_id, rospy.Time(0))

    euler = tf.transformations.euler_from_quaternion(rot)
    print("tf----->:euler", euler)
    print("tf----->:trans", trans)
    print("tf----->:rot", rot)


def main(args=None):
    rospy.init_node('tf_node')
    get_tf_coor("top_middle", "test_middle")
    rospy.spin()


if __name__ == '__main__':
    main()
