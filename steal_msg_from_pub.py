#!/usr/bin/env python

import time
import rospy
from rospy import AnyMsg

"""
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>

Get all the information of a topic that has a publisher.
It does not need traffic in the topic, but there must a publisher.
So if a topic has only a Subscriber you cannot get the
information as there is no connection header.
"""


def get_topic_connection_dict(topic_name):
    sub = rospy.Subscriber(topic_name, AnyMsg, None, queue_size=1)
    print("Waiting for connections...")
    while not sub.impl.connections and not rospy.is_shutdown():
        time.sleep(0.01)

    print("Waiting for connection information...")
    conn = sub.impl.connections[0]
    while conn is None and not rospy.is_shutdown():
        time.sleep(0.01)
        conn = sub.impl.connections[0]

    print("Waiting for header information...")
    while conn.header is None and not rospy.is_shutdown():
        time.sleep(0.01)

    print("Got it!")

    return conn.header


if __name__ == '__main__':
    import sys
    print("Initializing node...")
    rospy.init_node('get_msg_description')
    # In another shell
    # rostopic pub /testpub aruco_msgs/MarkerArray [TAB] -r1
    argv = rospy.myargv(sys.argv)
    if len(argv) > 1:
        d = get_topic_connection_dict(argv[1])
    else:
        d = get_topic_connection_dict('/testpub')
    print d

# You get something like 
# {'callerid': '/test',
#  'latching': '0',
#  'md5sum': '9d486b76ee1f72a8b0d33e8c66a97306',
#  'message_definition': "Header header\naruco_msgs/Marker[] markers\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\n# 0: no frame\n# 1: global frame\nstring frame_id\n\n================================================================================\nMSG: aruco_msgs/Marker\nHeader header\nuint32 id\ngeometry_msgs/PoseWithCovariance pose\nfloat64 confidence\n\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of postion and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n",
#  'topic': '/testpub',
#  'type': 'aruco_msgs/MarkerArray'}