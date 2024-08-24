#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching px4vision_0 and ball_hinge_0")
    req = AttachRequest()
    req.model_name_1 = "px4vision_0"
    req.link_name_1 = "base_link"
    req.model_name_2 = "slung_load"
    req.link_name_2 = "ball_hinge_0::base_link"

    attach_srv.call(req)
    # From the shell:
    """
rosservice call /link_attacher_node/attach "model_name_1: 'iris0'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
    """

    rospy.loginfo("Attaching px4vision_1 and ball_hinge_1")
    req = AttachRequest()
    req.model_name_1 = "px4vision_1"
    req.link_name_1 = "base_link"
    req.model_name_2 = "slung_load"
    req.link_name_2 = "ball_hinge_1::base_link"

    attach_srv.call(req)

    # rospy.loginfo("Attaching cube3 and iris0")
    # req = AttachRequest()
    # req.model_name_1 = "cube3"
    # req.link_name_1 = "link"
    # req.model_name_2 = "iris0"
    # req.link_name_2 = "link"

    # attach_srv.call(req)
