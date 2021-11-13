#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('task3_gazebo_sim')
    rospy.sleep(2)
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
  
    print("******************************************************")
    # Link them
    rospy.loginfo("Applying brakes to ebot")
    req = AttachRequest()
    req.model_name_1 = "ebot"
    req.link_name_1 = "ebot_base"
    req.model_name_2 = "green_house"
    req.link_name_2 = "link"

    attach_srv.call(req)


