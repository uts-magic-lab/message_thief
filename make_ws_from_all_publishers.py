#!/usr/bin/env python

import sys
import os
import tempfile
import rospy
import rosgraph
from steal_msg_from_pub import get_topic_connection_dict
from recreate_msgs import create_packages_and_messages_from_definition_and_type

"""
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
Get all topics with a publisher and create a workspace
with all stolen message packages. You can also provide
a folder to add the packages there.
"""

if __name__ == '__main__':
    rospy.init_node('all_topics_ws_creator')
    argv = rospy.myargv(sys.argv)
    if len(argv) > 1:
        already_existing_ws = argv[1]
    else:
        # Create a temporal workspace
        tmpdir = tempfile.mkdtemp(prefix="catkin_ws")
        tmpdir = tmpdir + '/src'
        os.mkdir(tmpdir)
        already_existing_ws = tmpdir
    master = rosgraph.masterapi.Master('/topic_pub_fetcher')
    published_topics = master.getPublishedTopics('')
    all_pkgs_and_msgs = {}
    for topic_name, type in published_topics:
        d = get_topic_connection_dict(topic_name)
        path, pkgs_msgs = create_packages_and_messages_from_definition_and_type(d['type'],
                                                                     d['message_definition'],
                                                                     already_existing_ws=already_existing_ws)
        for pkg in pkgs_msgs:
            if pkg in all_pkgs_and_msgs:
                all_pkgs_and_msgs[pkg].extend(pkgs_msgs[pkg])
                # Remove duplicates
                all_pkgs_and_msgs[pkg] = list(set(all_pkgs_and_msgs[pkg]))
            else:
                all_pkgs_and_msgs[pkg] = pkgs_msgs[pkg]

    print("\nDone, your workspace is at: " + already_existing_ws)
    print("We created " + str(len(all_pkgs_and_msgs)) + " packages, ")
    count_msgs = 0
    for pkg in all_pkgs_and_msgs:
        count_msgs += len(all_pkgs_and_msgs[pkg])
    print("with " + str(count_msgs) + " messages in them:")
    for pkg in all_pkgs_and_msgs:
        print("Package: " + pkg)
        print("    Messages: " + str(all_pkgs_and_msgs[pkg]))
