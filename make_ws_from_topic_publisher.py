#!/usr/bin/env python

import rospy
import sys
from steal_msg_from_pub import get_topic_connection_dict
from recreate_msgs import create_packages_and_messages_from_definition_and_type

"""
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
Give a topic (that has a publisher) to this script and it will
generate a workspace to be able to interact with that topic.
"""

if __name__ == '__main__':
    rospy.init_node('topic_ws_creator')
    argv = rospy.myargv(sys.argv)
    topic_name = argv[1]
    d = get_topic_connection_dict(topic_name)
    path, _ = create_packages_and_messages_from_definition_and_type(d['type'],
        d['message_definition'])
    print("Play with topic: " + topic_name + " with the ready to compile workspace: " + path)
    print("Just do:")
    print("  cd " + path)
    print("  catkin_make")
    print("  source devel/setup.bash")
    print("  rostopic pub " + topic_name + " " + d['type'] + " [TAB]")
    print("Or just spy it:")
    print("  rostopic echo " + topic_name)