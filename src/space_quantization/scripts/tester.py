#!/usr/bin/env python
# license removed for brevity
import rospy
from space_quantization.msg import codebook
from geometry_msgs.msg import Point

if __name__ == '__main__':
    pub = rospy.Publisher('/hsrb/codebook', codebook, queue_size=1)
    rospy.init_node('custom_talker', anonymous=True)
    r = rospy.Rate(2)  # 10hz

    msg = codebook()
    msg.header.frame_id = "base_link"
    msg.header.stamp = rospy.get_rostime()
    # test_points = [[1, 1, 0], [1, 2, 0], [1, 3, 0],
    #                [2, 1, 0], [2, 2, 0], [2, 3, 0],
    #                [1.5, 2, 0.2]]
    test_points = [[1, 1, 0], [1, 2, 0],
                   [2, 1, 0],
                   [1.5, 2, 0.2]]
    #[0.5, 0.5, 0.2], [1.5, 2, 0.2], [2.5, 3.5, 0.2]]

    for p in test_points:
        a = Point()
        a.x = p[0]
        a.y = p[1]
        a.z = p[2]
        msg.centroids.append(a)
    r.sleep()
#    print(msg)
    pub.publish(msg)
