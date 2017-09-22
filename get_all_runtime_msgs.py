#!/usr/bin/env python

import rospy
from rospkg import RosPack
from rospkg.common import ResourceNotFound
import rosgraph.masterapi
from rostopic import get_topic_type
from rosservice import get_service_type
from subprocess import check_output
from subprocess import CalledProcessError
import tempfile
import os
from shutil import copyfile
import tarfile

"""
Get all ROS msgs necessary from what
is currently running.

Author: Sammy Pfeiffer
"""


def get_message_dependencies(message_file_path, rp=RosPack()):
    """
    Given a message .msg file path,
    return the list of message file paths this
    message depends on."""

    basic_types = ["Bool", "Byte", "ByteMultiArray", "Char",
                   "ColorRGBA", "Duration", "Empty", "Float32",
                   "Float32MultiArray", "Float64", "Float64MultiArray",
                   "Header", "Int16", "Int16MultiArray", "Int32",
                   "Int32MultiArray", "Int64", "Int64MultiArray",
                   "Int8", "Int8MultiArray", "MultiArrayDimension",
                   "MultiArrayLayout", "String", "Time", "UInt16",
                   "UInt16MultiArray", "UInt32", "UInt32MultiArray",
                   "UInt64", "UInt64MultiArray", "UInt8", "UInt8MultiArray"]
    basic_types_lower = [bt.lower() for bt in basic_types]

    _message_file_paths = []
    with open(message_file_path, 'r') as f:
        text = f.read()
        for line in text.splitlines():
            line = line.strip()
            # ignore comments and empty lines
            # constant declarations are fine, they are basic types
            if not line.startswith('#') and len(line) > 2:
                line_parts = line.split(' ')
                if len(line_parts) >= 2:
                    msg_pkg_and_type = line_parts[0]

                    # In case it's an array, get rid of that part as it's
                    # not part of a message file name
                    if '[' in msg_pkg_and_type:
                        idx = msg_pkg_and_type.index('[')
                        msg_pkg_and_type = msg_pkg_and_type[:idx]

                    # With '/' it's from another package...
                    if '/' in msg_pkg_and_type:
                        pkg_name, msg_name = msg_pkg_and_type.split('/')
                        if msg_name.lower() in basic_types_lower:
                            idx = basic_types_lower.index(msg_name.lower())
                            msg_name = basic_types[idx]
                    # Without, it may be a local message from the same package
                    # or std_msgs
                    else:
                        # Check if it is of base type
                        if msg_pkg_and_type.lower() in basic_types_lower:
                            pkg_name = 'std_msgs'
                            idx = basic_types_lower.index(
                                msg_pkg_and_type.lower())
                            msg_name = basic_types[idx]
                        # Otherwise it's from the same package
                        else:
                            pkg_name, _ = get_pkg_and_msg_from_msg_path(
                                message_file_path)
                            msg_name = msg_pkg_and_type

                    pkg_path = rp.get_path(pkg_name)
                    msg_full_path = pkg_path + "/msg/" + msg_name + ".msg"
                    if not os.path.isfile(msg_full_path):
                        print "File: " + msg_full_path + " does not exist!"
                    if msg_full_path not in _message_file_paths:
                        _message_file_paths.append(msg_full_path)

    return _message_file_paths


def get_pkg_and_msg_from_msg_path(msg_path):
    # A msg_path looks like:
    # /opt/ros/indigo/share/actionlib_msgs/msg/GoalID.msg
    split_path = msg_path.split('/')
    msg_name = split_path[-1]
    msg_name = msg_name.replace('.msg', '')
    msg_name = msg_name.replace('.srv', '')
    msg_name = msg_name.replace('.action', '')
    pkg_name = split_path[-3]
    return pkg_name, msg_name


def get_message_file_paths_via_gendeps(types_list, rp=RosPack()):
    """
    Given a list of types as ['geometry_msgs/PoseStamped', ...]
    return the list of messages that are dependend with their full
    file path. Also returns a list of non processed files as gendeps
    sometimes fails parsing them.
    """
    message_file_paths = []
    non_processed_files = []
    for t in types_list:
        pkg_name, msg_name = t.split('/')
        pkg_path = rp.get_path(pkg_name)
        msg_full_path = pkg_path + "/msg/" + msg_name + ".msg"
        if msg_full_path not in message_file_paths:
            message_file_paths.append(msg_full_path)
        # Get list of files that this message depends on
        try:
            out = check_output(["rosrun", "roslib", "gendeps", msg_full_path])
        except CalledProcessError as e:
            print "\n   Exception when checking message: " + msg_full_path
            print "   " + str(e) + "\n"
            non_processed_files.append(msg_full_path)
        out = out.replace('\n', '')
        files = out.split(' ')
        for f in files:
            if f != '':
                if f not in message_file_paths:
                    message_file_paths.append(f)
        # if files != ['']:
        #     print "Message: " + t + " depends on: "
        #     print "         " + str(files)
        # else:
        #     print "Message: " + t + " has no dependencies."

    return message_file_paths, non_processed_files


def make_tarfile(output_filename, source_dir):
    print "tar-gz'ing dir: " + source_dir
    tar = tarfile.open(output_filename, "w:gz")
    tar.add(source_dir, arcname=os.path.basename(source_dir))
    tar.close()


std_msgs_list = ["std_msgs/Bool", "std_msgs/Byte", "std_msgs/ByteMultiArray",
                 "std_msgs/Char", "std_msgs/ColorRGBA",
                 "std_msgs/Duration", "std_msgs/Empty",
                 "std_msgs/Float32", "std_msgs/Float32MultiArray",
                 "std_msgs/Float64", "std_msgs/Float64MultiArray",
                 "std_msgs/Header", "std_msgs/Int16",
                 "std_msgs/Int16MultiArray", "std_msgs/Int32",
                 "std_msgs/Int32MultiArray", "std_msgs/Int64",
                 "std_msgs/Int64MultiArray", "std_msgs/Int8",
                 "std_msgs/Int8MultiArray", "std_msgs/MultiArrayDimension",
                 "std_msgs/MultiArrayLayout", "std_msgs/String",
                 "std_msgs/Time", "std_msgs/UInt16",
                 "std_msgs/UInt16MultiArray", "std_msgs/UInt32",
                 "std_msgs/UInt32MultiArray", "std_msgs/UInt64",
                 "std_msgs/UInt64MultiArray", "std_msgs/UInt8",
                 "std_msgs/UInt8MultiArray"]

if __name__ == '__main__':
    rospy.init_node('messages_fetcher')
    rp = RosPack()

    print "Getting currently listening topics and their types..."
    master = rosgraph.masterapi.Master('/package_fetcher')
    # Returns a list of topics and their subscribers
    listening_topics = master.getSystemState()[1]
    listening_topic_names = [lt[0] for lt in listening_topics]
    srvs_qtty = len(listening_topic_names)
    print "Found " + str(srvs_qtty) + " listening topics."

    topics_and_types_listening = []
    for t_name in listening_topic_names:
        topic_type, _, _ = get_topic_type(t_name, blocking=True)
        topics_and_types_listening.append([t_name, topic_type])

    print "Getting currently published topics and their types..."
    topics_and_types = rospy.get_published_topics()
    print "Found " + str(len(topics_and_types)) + " topics."

    print "Merging both topics lists..."
    for tt in topics_and_types_listening:
        if tt not in topics_and_types:
            topics_and_types.append(tt)

    topics_and_types_qtty = len(topics_and_types)
    print "Found " + str(topics_and_types_qtty) + " total topics."

    # Deal with action servers
    possible_action_servers = {}
    for topic, type_ in topics_and_types:
        if topic.endswith('/goal'):
            as_name = topic.replace('/goal', '')
            if as_name not in possible_action_servers:
                possible_action_servers[as_name] = {}
            possible_action_servers[as_name]['goal'] = True
        if topic.endswith('/result'):
            as_name = topic.replace('/result', '')
            if as_name not in possible_action_servers:
                possible_action_servers[as_name] = {}
            possible_action_servers[as_name]['result'] = True
        if topic.endswith('/feedback'):
            as_name = topic.replace('/feedback', '')
            if as_name not in possible_action_servers:
                possible_action_servers[as_name] = {}
            possible_action_servers[as_name]['feedback'] = True
        if topic.endswith('/cancel'):
            as_name = topic.replace('/cancel', '')
            if as_name not in possible_action_servers:
                possible_action_servers[as_name] = {}
            possible_action_servers[as_name]['cancel'] = True
        if topic.endswith('/status'):
            as_name = topic.replace('/status', '')
            if as_name not in possible_action_servers:
                possible_action_servers[as_name] = {}
            possible_action_servers[as_name]['status'] = True

    pas_qtty = len(possible_action_servers)
    print "Found " + str(pas_qtty) + " posible action servers..."

    confirmed_action_servers = []
    for pas, d in possible_action_servers.iteritems():
        if 'goal' in d and 'result' in d and 'feedback' in d and 'cancel' in d and 'status' in d:
            if pas not in confirmed_action_servers:
                confirmed_action_servers.append(pas)

    conf_as_qtty = len(confirmed_action_servers)
    print "Confirmed " + str(conf_as_qtty) + " action servers."

    # Construct a list of non-repeated message types
    # not taking into account action server topics
    types_list = []
    # Force std_msgs to be there
    types_list.extend(std_msgs_list)
    skipped_topics = 0
    for topic, type_ in topics_and_types:
        # print "Topic: '" + str(topic) + "' has type: '" + str(type_) + "'"
        is_as = False
        for cas in confirmed_action_servers:
            if topic.startswith(cas):
                if topic.endswith('/goal') or \
                        topic.endswith('/result') or \
                        topic.endswith('/feedback') or \
                        topic.endswith('/cancel') or \
                        topic.endswith('/status'):
                    skipped_topics += 1
                    is_as = True
                    break

        if not is_as:
            if type_ not in types_list:
                types_list.append(type_)

    unique_msgs_qtty = len(types_list)
    print "Found " + str(unique_msgs_qtty) + " messages used."
    if skipped_topics > 0:
        print "(We skipped " + str(skipped_topics) + " topics as they were part of action servers)."

    pkgs_and_msgs_dict = {}
    for t in types_list:
        pkg_name, msg_name = t.split('/')
        if not pkgs_and_msgs_dict.get(pkg_name):
            pkgs_and_msgs_dict[pkg_name] = []
        pkgs_and_msgs_dict[pkg_name].append(msg_name)

    pkgs_qtty = len(pkgs_and_msgs_dict)
    print "Found " + str(pkgs_qtty) + " message packages to be fetch."

    # # Print resume of list
    # print "Resume of packages and their necessary messages:"
    # for pkg, msgs in pkgs_and_msgs_dict.iteritems():
    #     print "Package: " + pkg
    #     print "Has:         " + str(msgs)

    # Add dependencies until exhausting
    message_file_paths = []
    for t in types_list:
        if '/' in t:
            pkg, msg_name = t.split('/')
            try:
                pkg_path = rp.get_path(pkg)
            except ResourceNotFound as e:
                print("Error: " + str(e))
                print("This means you dont have that package and you need it in the robot!")
                continue
            msg_full_path = pkg_path + "/msg/" + msg_name + ".msg"
            if msg_full_path not in message_file_paths:
                message_file_paths.append(msg_full_path)
            msg_files = get_message_dependencies(msg_full_path)
            for m in msg_files:
                if m not in message_file_paths:
                    message_file_paths.append(m)

    msg_f_paths_qtty = len(message_file_paths)
    print "We found " + str(msg_f_paths_qtty) + " message files that we directly depend."

    # Go thru all the list manually checking every msg file again
    # so we make sure we didn't forget any

    for f in message_file_paths:
        msg_f_paths = get_message_dependencies(f, rp=rp)
        for new_f in msg_f_paths:
            if new_f not in message_file_paths:
                # If we found a new dependency (damn gendeps)
                # we'll process it in the end
                message_file_paths.append(new_f)

    final_f_paths_qtty = len(message_file_paths)
    print "We are sure that " + str(final_f_paths_qtty) + " message files are depended."

    # Get services
    print "Getting current services..."
    # Returns a list of services and their node providing it
    services = master.getSystemState()[2]
    service_names = [s[0] for s in services]
    srvs_qtty = len(service_names)
    print "Found " + str(srvs_qtty) + " services."
    services_and_types = []
    for name in service_names:
        type_ = get_service_type(name)
        services_and_types.append([name, type_])
        # print "Service: '" + name + "' has type: '" + type_ + "'"

    srv_types_list = []
    for service, type_ in services_and_types:
        if type_ not in srv_types_list:
            srv_types_list.append(type_)

    unique_srvs_qtty = len(srv_types_list)
    print "Found " + str(unique_srvs_qtty) + " service messages used."

    pkgs_and_srvs_dict = {}
    srvs_msg_paths = []
    for t in srv_types_list:
        pkg_name, msg_name = t.split('/')
        if not pkgs_and_srvs_dict.get(pkg_name):
            pkgs_and_srvs_dict[pkg_name] = []
        srv_pkg_path = rp.get_path(pkg_name)
        srv_msg_full_path = srv_pkg_path + "/srv/" + msg_name + ".srv"
        pkgs_and_srvs_dict[pkg_name].append(srv_msg_full_path)
        if srv_msg_full_path not in srvs_msg_paths:
            srvs_msg_paths.append(srv_msg_full_path)

    srv_pkgs_qtty = len(pkgs_and_msgs_dict.keys())
    print "Found " + str(srv_pkgs_qtty) + " service message packages to be fetch."

    # Get messages that services need
    msg_paths_from_srvs = []
    # first run with gendeps
    for srv_path in srvs_msg_paths:
        msgs = get_message_dependencies(srv_path, rp=rp)
        for m in msgs:
            if m not in msg_paths_from_srvs:
                msg_paths_from_srvs.append(m)

    msg_from_srvs_qtty = len(msg_paths_from_srvs)
    print "Found we need " + str(msg_from_srvs_qtty) + " messages for the services."

    for msg_file in msg_paths_from_srvs:
        if msg_file not in message_file_paths:
            message_file_paths.append(msg_file)

    new_total_qtty = len(message_file_paths)
    print "Adding to what we had, now found we need " + str(new_total_qtty) + " message files."

    # Get the action type from the goal topic
    action_server_and_types = []
    action_description_file_paths = []
    for topic, type_ in topics_and_types:
        for cas in confirmed_action_servers:
            as_and_type = [cas]
            if cas + '/goal' == topic:
                action_description_file = type_
                action_description_file = action_description_file.replace(
                    'ActionGoal', '')
                as_and_type.append(action_description_file)
                pkg_name, action_file = action_description_file.split('/')
                pkg_path = rp.get_path(pkg_name)
                full_as_file_path = pkg_path + '/action/' + action_file + '.action'
                if full_as_file_path not in action_description_file_paths:
                    action_description_file_paths.append(full_as_file_path)
                    action_server_and_types.append(as_and_type)

    for adfp in action_description_file_paths:
        dep_file_paths = get_message_dependencies(adfp, rp=rp)
        for dfp in dep_file_paths:
            if dfp not in message_file_paths:
                message_file_paths.append(dfp)

    # Make sure we don't have duplicates
    message_file_paths = list(set(message_file_paths))

    # Final re-check that we aren't missing messages
    # By looping checking if we get additional files
    prev_message_file_paths_qtty = len(message_file_paths)
    new_message_file_paths_qtty = 0
    loops = 0
    while prev_message_file_paths_qtty != new_message_file_paths_qtty:
        prev_message_file_paths_qtty = len(message_file_paths)
        for msg_path in message_file_paths:
            deps = get_message_dependencies(msg_path, rp=rp)
            for d in deps:
                if d not in message_file_paths:
                    message_file_paths.append(d)
        new_message_file_paths_qtty = len(message_file_paths)
        loops += 1

    print "We looped " + str(loops) + " times to find more messages..."

    # Form dictionary of
    # pkg_name:
    #           msg_files: []
    #           srv_files: []
    #           action_files: []
    #           depends: []
    # And create fake replica of pkgs
    # to be able to compile all the messages and srv's

    generate_dict = {}
    for msg_path in message_file_paths:
        pkg_name, msg_name = get_pkg_and_msg_from_msg_path(msg_path)
        if pkg_name not in generate_dict:
            generate_dict[pkg_name] = {'msg_files': [],
                                       'depends': []}
        generate_dict[pkg_name]['msg_files'].append(msg_path)
        deps_msgs_paths = get_message_dependencies(msg_path, rp=rp)

        for d in deps_msgs_paths:
            pkg, _ = get_pkg_and_msg_from_msg_path(d)
            # Don't add as dependence the own package
            # and don't add multiple times dependences
            if pkg not in generate_dict[pkg_name]['depends'] and pkg != pkg_name:
                generate_dict[pkg_name]['depends'].append(pkg)

    for srv_path in srvs_msg_paths:
        pkg_name, msg_name = get_pkg_and_msg_from_msg_path(srv_path)
        if pkg_name not in generate_dict:
            generate_dict[pkg_name] = {'srv_files': [],
                                       'depends': []}
        elif 'srv_files' not in generate_dict[pkg_name]:
            generate_dict[pkg_name]['srv_files'] = []
        generate_dict[pkg_name]['srv_files'].append(srv_path)
        deps_msgs_paths = get_message_dependencies(srv_path, rp=rp)

        for d in deps_msgs_paths:
            pkg, _ = get_pkg_and_msg_from_msg_path(d)
            # Don't add as dependence the own package
            # and don't add multiple times dependences
            if pkg not in generate_dict[pkg_name]['depends'] and pkg != pkg_name:
                generate_dict[pkg_name]['depends'].append(pkg)

    for action_path in action_description_file_paths:
        pkg_name, msg_name = get_pkg_and_msg_from_msg_path(action_path)
        if pkg_name not in generate_dict:
            generate_dict[pkg_name] = {'action_files': [],
                                       'depends': []}
        elif 'action_files' not in generate_dict[pkg_name]:
            generate_dict[pkg_name]['action_files'] = []
        generate_dict[pkg_name]['action_files'].append(action_path)
        deps_msgs_paths = get_message_dependencies(action_path, rp=rp)

        for d in deps_msgs_paths:
            pkg, _ = get_pkg_and_msg_from_msg_path(d)
            # Don't add as dependence the own package
            # and don't add multiple times dependences
            if pkg not in generate_dict[pkg_name]['depends'] and pkg != pkg_name:
                generate_dict[pkg_name]['depends'].append(pkg)

    print "------------"
    print "Final resume:"
    print "------------"

    num_pkgs = len(generate_dict)
    num_msg_files = len(message_file_paths)
    num_srv_files = len(srvs_msg_paths)
    num_as_files = len(action_description_file_paths)

    # We have still stored
    # topics_and_types: [['/joint_states', 'sensor_msgs/JointState'], ...]
    # services_and_types: [['/rosout/get_loggers', 'roscpp/GetLoggers'], ...]
    # action_server_and_types: [['/l_arm_controller/follow_joint_trajectory', 'control_msgs/FollowJointTrajectory'], ...]
    #
    # With this we can generate dynamically a Python class
    # that enables the interaction with the robot like...
    #
    # Topic example:
    # js = robot.topics.joint_states
    # js.wait_for_message()
    # js.pub(js.msg())
    #
    # Service example:
    # get_log = robot.services.rosout-get_loggers
    # get_log.call(get_log.request())
    #
    # Action server example:
    # l_arm = robot.action_servers.l_arm_controller-follow_joint_trajectory
    # g = l_arm.goal()
    # l_arm.send_goal(g)
    #
    # TODO: params and dynparams in the same style

    print "We'll generate " + str(num_pkgs) + " packages."
    print "With " + str(num_msg_files) + " msg definition files."
    print "And " + str(num_srv_files) + " srv service definition files."
    print "And " + str(num_as_files) + " action definition files."

    print "Current dictionary to generate fake pkgs:"
    import pprint
    pp = pprint.PrettyPrinter(indent=2)
    pp.pprint(generate_dict)

    # Having the full list, now we steal all the messages
    # and create a workspace to compile all the messages
    # with generated packages that only include these
    # messages/services/actionservers

    # TODO: Optionally, we can also create a folder
    # to have readily-available all the messages in python
    # as we can steal the already generated from the robot...

    cmakelists_template = """cmake_minimum_required(VERSION 2.8.3)
project(__PACKAGE_NAME__)

# Message package generated from a running system
# THIS IS NOT THE ORIGINAL PACKAGE

find_package(catkin REQUIRED COMPONENTS message_generation __DEPENDENCIES_LIST__)

__MESSAGE_BLOCK__

__SERVICE_BLOCK__

__ACTION_BLOCK__

generate_messages(DEPENDENCIES __DEPENDENCIES_LIST__)

catkin_package(CATKIN_DEPENDS message_runtime __DEPENDENCIES_LIST__)
"""

    message_tmp = """add_message_files(
  DIRECTORY msg
  FILES
    __MSG_FILENAMES__)"""

    service_tmp = """add_service_files(
  DIRECTORY srv
  FILES
    __SRV_FILENAMES__)"""

    action_tmp = """add_action_files(
  DIRECTORY action
  FILES
    __ACTION_FILENAMES__)"""

    package_template = """<package>
  <name>__PACKAGE_NAME__</name>
  <version>0.0.1</version>
  <description>
    This package is autogenerated from a running system to get all the necessary messages.
    THIS IS NOT THE REAL PACKAGE.
  </description>
  <maintainer email="nomaintainer@nomaintainer.com">No maintainer</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>message_generation</build_depend>
__BUILD_DEPEND_BLOCK__

  <run_depend>message_runtime</run_depend>
__RUN_DEPEND_BLOCK__

</package>
    """
    build_tmp = "  <build_depend>__MSGS__</build_depend>"
    run_tmp = "  <run_depend>__MSGS__</run_depend>"

    tmpdir = tempfile.mkdtemp(prefix="catkin_ws")
    tmpdir = tmpdir + '/src'
    os.mkdir(tmpdir)
    print "Working on temporal dir: " + tmpdir
    for pkg_name, pkg_d in generate_dict.iteritems():
        # Create folder of pkg
        print "Creating pkg: " + pkg_name
        this_pkg_dir = tmpdir + '/' + pkg_name
        os.mkdir(this_pkg_dir)

        # Check if it has msgs / srvs / actionlibs
        msg_files = pkg_d.get('msg_files', [])
        msg_names = []
        for m in msg_files:
            pkg, msg = get_pkg_and_msg_from_msg_path(m)
            msg_names.append(msg + ".msg")
        srv_files = pkg_d.get('srv_files', [])
        srv_names = []
        for m in srv_files:
            pkg, msg = get_pkg_and_msg_from_msg_path(m)
            srv_names.append(msg + ".srv")
        action_files = pkg_d.get('action_files', [])
        action_names = []
        for m in action_files:
            pkg, msg = get_pkg_and_msg_from_msg_path(m)
            action_names.append(msg + ".action")

        # Create list of dependencies
        dependencies = pkg_d.get('depends', [])

        # Create package.xml
        packagexml = package_template
        packagexml = packagexml.replace('__PACKAGE_NAME__', pkg_name)

        # build depends
        build_deps_str = ''
        for dep in dependencies:
            build_deps_str += build_tmp.replace('__MSGS__', dep) + "\n"
        if action_files:
            build_deps_str += build_tmp.replace('__MSGS__',
                                                'actionlib_msgs') + '\n'
        packagexml = packagexml.replace('__BUILD_DEPEND_BLOCK__',
                                        build_deps_str)
        # run depends
        run_deps_str = ''
        for dep in dependencies:
            run_deps_str += run_tmp.replace('__MSGS__', dep) + "\n"
        if action_files:
            run_deps_str += run_tmp.replace('__MSGS__',
                                            'actionlib_msgs') + '\n'
        packagexml = packagexml.replace('__RUN_DEPEND_BLOCK__',
                                        run_deps_str)

        print "   ...creating package.xml"
        with open(this_pkg_dir + '/package.xml', 'w') as f:
            f.write(packagexml)

        # Create CMakeLists.txt
        cmakeliststxt = cmakelists_template

        cmakeliststxt = cmakeliststxt.replace('__PACKAGE_NAME__', pkg_name)

        deps = ' '.join(dependencies)
        if action_files:
            deps += " actionlib_msgs"
        cmakeliststxt = cmakeliststxt.replace('__DEPENDENCIES_LIST__', deps)

        if msg_files:
            msg_tmp = message_tmp
            msg_str = ''
            for idx, f in enumerate(msg_names):
                if idx > 0:
                    msg_str += "    "
                msg_str += f + "\n"
            msg_tmp = msg_tmp.replace('__MSG_FILENAMES__', msg_str)
            cmakeliststxt = cmakeliststxt.replace('__MESSAGE_BLOCK__', msg_tmp)
            os.mkdir(this_pkg_dir + '/msg')
            for mfile in msg_files:
                _, msg_name = get_pkg_and_msg_from_msg_path(mfile)
                copyfile(mfile, this_pkg_dir + '/msg/' + msg_name + '.msg')
            print "   ...creating msg folder."
        else:
            cmakeliststxt = cmakeliststxt.replace('__MESSAGE_BLOCK__', '')

        if srv_files:
            srv_tmp = service_tmp
            msg_str = ''
            for idx, f in enumerate(srv_names):
                if idx > 0:
                    msg_str += "    "
                msg_str += f + "\n"
            srv_tmp = srv_tmp.replace('__SRV_FILENAMES__', msg_str)
            cmakeliststxt = cmakeliststxt.replace('__SERVICE_BLOCK__', srv_tmp)
            os.mkdir(this_pkg_dir + '/srv')
            for mfile in srv_files:
                _, msg_name = get_pkg_and_msg_from_msg_path(mfile)
                copyfile(mfile, this_pkg_dir + '/srv/' + msg_name + '.srv')
            print "   ...creating srv folder."
        else:
            cmakeliststxt = cmakeliststxt.replace('__SERVICE_BLOCK__', '')

        if action_files:
            as_tmp = action_tmp
            msg_str = ''
            for idx, f in enumerate(action_names):
                if idx > 0:
                    msg_str += "    "
                msg_str += f + "\n"
            as_tmp = as_tmp.replace('__ACTION_FILENAMES__', msg_str)
            cmakeliststxt = cmakeliststxt.replace(
                '__ACTION_BLOCK__', as_tmp)
            os.mkdir(this_pkg_dir + '/action')
            for mfile in action_files:
                _, msg_name = get_pkg_and_msg_from_msg_path(mfile)
                copyfile(mfile, this_pkg_dir +
                         '/action/' + msg_name + '.action')
            print "   ...creating action folder."
        else:
            cmakeliststxt = cmakeliststxt.replace('__ACTION_BLOCK__', '')

        print "   ...creating CMakeLists.txt"
        with open(this_pkg_dir + "/CMakeLists.txt", 'w') as f:
            f.write(cmakeliststxt)

    print "We are done!"
    print "\n\n"
    print "Try to compile: "
    parent_tmpdir = tmpdir.replace('/src', '')
    print "cd " + parent_tmpdir
    print "catkin_make"

    make_tarfile(parent_tmpdir + '/all_msgs.tar.gz', parent_tmpdir)
    print "Find a all_msgs.tar.gz at " + parent_tmpdir

    # Do hacky Python copy
    from shutil import copytree
    tmpdirpy = tempfile.mkdtemp(prefix="python_msgs")
    for pkg in generate_dict:
        python_lib = '/opt/ros/indigo/lib/python2.7/dist-packages/' + pkg
        copytree(python_lib, tmpdirpy + '/' + pkg)
        # TODO: check if we need to delete .pyc files

    print "Adding to sys.path : " + str(tmpdirpy)
    print "You can use all messages."