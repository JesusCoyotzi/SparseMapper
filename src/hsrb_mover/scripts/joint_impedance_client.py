#!/usr/bin/python
# Copyright (C) 2017 Toyota Motor Corporation
import dynamic_reconfigure.client
import rospy

if __name__ == "__main__":
    rospy.init_node("joint_impedance_client")

    # connect to dynamic_reconfigure server and return a client object
    client = dynamic_reconfigure.client.Client(
        "/hsrb/joint_impedance_control_server/arm_flex_joint",
        timeout=30)

    # change the server's configuration
    client.update_configuration({"enable": True,
                                 "inertia": 2.0,
                                 "damping": 4.0})

    # retrieve a parameter from the param server
    damping = rospy.get_param(
        "/hsrb/joint_impedance_control_server/arm_flex_joint/damping")
    rospy.loginfo("damping: %s", damping)
