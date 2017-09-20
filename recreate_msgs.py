#!/usr/bin/env python

import os
import tempfile
import copy
import re

# Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
# Recreate packages and messsages based
# on a message_definition and type
# TODO: Do the same for services


cmakelists_template = """cmake_minimum_required(VERSION 2.8.3)
project(__PACKAGE_NAME__)

# Message package generated from a running system
# THIS IS NOT THE ORIGINAL PACKAGE

find_package(catkin REQUIRED COMPONENTS message_generation 
#__TEMPLATE_DEPENDENCIES_START__
#__TEMPLATE_DEPENDENCIES_END__
)

__MESSAGE_BLOCK__

__SERVICE_BLOCK__

__ACTION_BLOCK__

generate_messages(DEPENDENCIES 
#__TEMPLATE_DEPENDENCIES_START__
#__TEMPLATE_DEPENDENCIES_END__
)

catkin_package(CATKIN_DEPENDS message_runtime 
#__TEMPLATE_DEPENDENCIES_START__
#__TEMPLATE_DEPENDENCIES_END__
)
"""

message_tmp = """add_message_files(
  DIRECTORY msg
  FILES
    #__TEMPLATE_MSGS_START__
    __MSG_FILENAMES__
    #__TEMPLATE_MSGS_END__
    )"""

service_tmp = """add_service_files(
  DIRECTORY srv
  FILES
    __SRV_FILENAMES__
    #__TEMPLATE_SRVS__
    )"""

action_tmp = """add_action_files(
  DIRECTORY action
  FILES
    __ACTION_FILENAMES__
    #__TEMPLATE_ACTIONS__
    )"""

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
  <!-- __TEMPLATE_BUILD_DEPENDS_START__ -->
  <!-- __TEMPLATE_BUILD_DEPENDS_END__ -->

  <run_depend>message_runtime</run_depend>
  <!-- __TEMPLATE_RUN_DEPENDS_START__ -->
  <!-- __TEMPLATE_RUN_DEPENDS_END__ -->

</package>
    """
build_tmp = "  <build_depend>__MSGS__</build_depend>"
run_tmp = "  <run_depend>__MSGS__</run_depend>"


def create_dir(path):
    try:
        os.mkdir(path)
    except OSError as e:
        if 'File exists' in e:
            print("Directory " + path + ' already exists... continuing.')
        else:
            raise


def create_pkg_path_and_msg_file(pkg_name, msg_name, text, workspace_path):
    print("Creating package: " + pkg_name + ' with message: ' + msg_name)
    path = workspace_path + '/' + pkg_name
    # Create package name folder
    create_dir(path)
    # Create subfolder msg
    create_dir(path + '/msg')
    # Write the actual message
    with open(path + '/msg/' + msg_name + '.msg', 'w') as f:
        f.write(text)


def add_cmakelists_and_packagexml(pkg_name, workspace_path,
                                  dependencies, pkg_msgs_names):
    """
    Given an already made workspace with a pkg_name, add to it the
    CMakeLists.txt and package.xml files, taking into account that
    if they already exist, the new dependencies must be appended.
    We don't deal with duplications, too much work.
    """
    print("    Creating or updating cmakelists and packagexml for package: " + pkg_name)
    print("    It has dependencies: " + str(dependencies))
    add_packagexml(pkg_name, workspace_path, dependencies)
    add_cmakeliststxt(pkg_name, workspace_path, dependencies, pkg_msgs_names)


def add_packagexml(pkg_name, workspace_path, dependencies):
    this_pkg_dir = workspace_path + '/' + pkg_name
    clean_dep_list = copy.deepcopy(dependencies)
    while pkg_name in clean_dep_list:
        clean_dep_list.remove(pkg_name)

    # Check if package.xml already exists
    if os.path.isfile(this_pkg_dir + '/package.xml'):
        # Deal with appending dependencies
        print "   ...adding dependencies to package.xml"
        with open(this_pkg_dir + '/package.xml', 'r') as f:
            curr_xml = f.read()

        startidx = curr_xml.index(
            "  <!-- __TEMPLATE_BUILD_DEPENDS_START__ -->")
        startidx_deps = startidx + \
            len("  <!-- __TEMPLATE_BUILD_DEPENDS_START__ -->")
        endidx = curr_xml.index("  <!-- __TEMPLATE_BUILD_DEPENDS_END__ -->")
        endidx_block = endidx + \
            len("  <!-- __TEMPLATE_BUILD_DEPENDS_END__ -->")

        curr_deps = curr_xml[startidx_deps:endidx]
        curr_deps = curr_deps.replace('<build_depend>', '')
        curr_deps = curr_deps.replace('</build_depend>', '')
        curr_deps = curr_deps.strip()
        curr_deps = curr_deps.split(' ')

        for d in curr_deps:
            clean_dep_list.append(d)
        # Remove duplicates
        clean_dep_list = list(set(clean_dep_list))

        # build depends
        build_deps_str = ''
        for dep in clean_dep_list:
            build_deps_str += build_tmp.replace('__MSGS__', dep) + "\n"
        # if action_files:
        #     build_deps_str += build_tmp.replace('__MSGS__',
        #                                         'actionlib_msgs') + '\n'
        new_block = "  <!-- __TEMPLATE_BUILD_DEPENDS_START__ -->\n" + \
            build_deps_str + "\n  <!-- __TEMPLATE_BUILD_DEPENDS_END__ -->"

        curr_xml = curr_xml.replace(curr_xml[startidx:endidx_block],
                                    new_block)
        # run depends
        run_deps_str = ''
        for dep in clean_dep_list:
            run_deps_str += run_tmp.replace('__MSGS__', dep) + "\n"

        startidx = curr_xml.index("  <!-- __TEMPLATE_RUN_DEPENDS_START__ -->")
        startidx_deps = startidx + \
            len("  <!-- __TEMPLATE_RUN_DEPENDS_START__ -->")
        endidx = curr_xml.index("  <!-- __TEMPLATE_RUN_DEPENDS_END__ -->")
        endidx_block = endidx + len("  <!-- __TEMPLATE_RUN_DEPENDS_END__ -->")
        # We dont need to check the current deps again on run
        # as they are the same than build

        new_block = "  <!-- __TEMPLATE_RUN_DEPENDS_START__ -->\n" + \
            run_deps_str + "\n  <!-- __TEMPLATE_RUN_DEPENDS_END__ -->"

        curr_xml = curr_xml.replace(curr_xml[startidx:endidx_block],
                                    new_block)

        packagexml = curr_xml

    else:
        print "   ...creating package.xml"
        # Create a new package.xml
        packagexml = package_template
        packagexml = packagexml.replace('__PACKAGE_NAME__', pkg_name)

        # build depends
        build_deps_str = ''
        for dep in clean_dep_list:
            build_deps_str += build_tmp.replace('__MSGS__', dep) + "\n"
        # if action_files:
        #     build_deps_str += build_tmp.replace('__MSGS__',
        #                                         'actionlib_msgs') + '\n'
        build_deps_str = "  <!-- __TEMPLATE_BUILD_DEPENDS_START__ -->\n" + \
            build_deps_str + "\n  <!-- __TEMPLATE_BUILD_DEPENDS_END__ -->"""
        packagexml = packagexml.replace('''  <!-- __TEMPLATE_BUILD_DEPENDS_START__ -->
  <!-- __TEMPLATE_BUILD_DEPENDS_END__ -->''',
                                        build_deps_str)
        # run depends
        run_deps_str = ''
        for dep in clean_dep_list:
            run_deps_str += run_tmp.replace('__MSGS__', dep) + "\n"
        # if action_files:
        #     run_deps_str += run_tmp.replace('__MSGS__',
        #                                     'actionlib_msgs') + '\n'

        run_deps_str = "  <!-- __TEMPLATE_RUN_DEPENDS_START__ -->\n" + \
            run_deps_str + "\n  <!-- __TEMPLATE_RUN_DEPENDS_END__ -->"""
        packagexml = packagexml.replace('''  <!-- __TEMPLATE_RUN_DEPENDS_START__ -->
  <!-- __TEMPLATE_RUN_DEPENDS_END__ -->''',
                                        run_deps_str)

    with open(this_pkg_dir + '/package.xml', 'w') as f:
        f.write(packagexml)


def add_cmakeliststxt(pkg_name, workspace_path, dependencies, pkg_msgs_names):
    this_pkg_dir = workspace_path + '/' + pkg_name
    clean_dep_list = copy.deepcopy(dependencies)
    while pkg_name in clean_dep_list:
        clean_dep_list.remove(pkg_name)

    deps = ' '.join(clean_dep_list)
    # Check if CMakeLists.tx already exists
    if os.path.isfile(this_pkg_dir + '/CMakeLists.txt'):
        # Append to the existing one
        print("   ...adding dependencies to CMakeLists.txt")

        with open(this_pkg_dir + '/CMakeLists.txt', 'r') as f:
            curr_cmakelists = f.read()

        if len(deps) > 0:
            # Grab current dependencies already there
            # first instance should be the same everywhere
            startidx = curr_cmakelists.index(
                "#__TEMPLATE_DEPENDENCIES_START__")
            startidx_deps = startidx + len("#__TEMPLATE_DEPENDENCIES_START__")
            endidx = curr_cmakelists.index("#__TEMPLATE_DEPENDENCIES_END__")
            endidx_block = endidx + len("#__TEMPLATE_DEPENDENCIES_END__")
            curr_deps = curr_cmakelists[startidx_deps:endidx].strip()
            curr_deps = curr_deps.split(' ')

            for d in clean_dep_list:
                if d not in curr_deps:
                    curr_deps.append(d)
            # Remove duplicates
            curr_deps = list(set(curr_deps))
            deps = ' '.join(curr_deps)

            full_block = "#__TEMPLATE_DEPENDENCIES_START__\n" + \
                deps + "\n#__TEMPLATE_DEPENDENCIES_END__"

            curr_cmakelists = curr_cmakelists.replace(
                curr_cmakelists[startidx:endidx_block], full_block)

            # Do it twice more, as this is present three times
            last_endidx_block = endidx_block
            first_part = curr_cmakelists[:last_endidx_block]
            second_part = curr_cmakelists[last_endidx_block:]
            startidx = second_part.index("#__TEMPLATE_DEPENDENCIES_START__")
            startidx_deps = startidx + len("#__TEMPLATE_DEPENDENCIES_START__")
            endidx = second_part.index("#__TEMPLATE_DEPENDENCIES_END__")
            endidx_block = endidx + len("#__TEMPLATE_DEPENDENCIES_END__")

            second_part = second_part.replace(
                second_part[startidx:endidx_block], full_block)

            last_endidx_block = endidx_block
            third_part = second_part[last_endidx_block:]
            second_part = second_part[:last_endidx_block]
            startidx = third_part.index("#__TEMPLATE_DEPENDENCIES_START__")
            startidx_deps = startidx + len("#__TEMPLATE_DEPENDENCIES_START__")
            endidx = third_part.index("#__TEMPLATE_DEPENDENCIES_END__")
            endidx_block = endidx + len("#__TEMPLATE_DEPENDENCIES_END__")

            third_part = third_part.replace(
                third_part[startidx:endidx_block], full_block)

            curr_cmakelists = first_part + second_part + third_part

        if len(pkg_msgs_names) > 0:
            # Grab current messages already there
            startidx = curr_cmakelists.index("    #__TEMPLATE_MSGS_START__")
            startidx_msgs = startidx + len("    #__TEMPLATE_MSGS_START__")
            endidx = curr_cmakelists.index("    #__TEMPLATE_MSGS_END__")
            endidx_block = endidx + len("    #__TEMPLATE_MSGS_END__")
            curr_msgs = curr_cmakelists[startidx_msgs:endidx]
            curr_msgs_clean = curr_msgs.strip()
            curr_msgs = curr_msgs_clean.split(' ')

            full_names = []
            for msg in pkg_msgs_names:
                full_names.append(msg + '.msg')
            for msg in curr_msgs:
                full_names.append(msg)
            # Remove duplicates
            full_names = list(set(full_names))
            msg_names = '    '
            msg_names += ' '.join(full_names)
            full_block = "    #__TEMPLATE_MSGS_START__\n" + \
                msg_names + "\n    #__TEMPLATE_MSGS_END__"

            curr_cmakelists = curr_cmakelists.replace(
                curr_cmakelists[startidx:endidx_block], full_block)

        cmakeliststxt = curr_cmakelists
    else:
        # Create CMakeLists.txt
        print("   ...creating CMakeLists.txt")
        cmakeliststxt = cmakelists_template

        cmakeliststxt = cmakeliststxt.replace('__PACKAGE_NAME__', pkg_name)

        # if action_files:
        #     deps += " actionlib_msgs"
        deps = "#__TEMPLATE_DEPENDENCIES_START__\n" + \
            deps + "\n#__TEMPLATE_DEPENDENCIES_END__"
        cmakeliststxt = cmakeliststxt.replace('''#__TEMPLATE_DEPENDENCIES_START__
#__TEMPLATE_DEPENDENCIES_END__''', deps)

        # if msg_files:
        msg_tmp = message_tmp
        full_names = []
        for name in pkg_msgs_names:
            full_names.append(name + '.msg')

        msg_str = ' '.join(full_names)
        msg_tmp = msg_tmp.replace('__MSG_FILENAMES__', msg_str)
        cmakeliststxt = cmakeliststxt.replace('__MESSAGE_BLOCK__', msg_tmp)
        # else:
        #     cmakeliststxt = cmakeliststxt.replace('__MESSAGE_BLOCK__', '')

        # if srv_files:
        #     srv_tmp = service_tmp
        #     msg_str = ''
        #     for idx, f in enumerate(srv_names):
        #         if idx > 0:
        #             msg_str += "    "
        #         msg_str += f + "\n"
        #     srv_tmp = srv_tmp.replace('__SRV_FILENAMES__', msg_str)
        #     cmakeliststxt = cmakeliststxt.replace('__SERVICE_BLOCK__', srv_tmp)
        #     os.mkdir(this_pkg_dir + '/srv')
        #     for mfile in srv_files:
        #         _, msg_name = get_pkg_and_msg_from_msg_path(mfile)
        #         copyfile(mfile, this_pkg_dir + '/srv/' + msg_name + '.srv')
        #     print "   ...creating srv folder."
        # else:
        cmakeliststxt = cmakeliststxt.replace('__SERVICE_BLOCK__\n', '')

        cmakeliststxt = cmakeliststxt.replace('__ACTION_BLOCK__\n', '')

    with open(this_pkg_dir + "/CMakeLists.txt", 'w') as f:
        f.write(cmakeliststxt)


def get_dependencies_from_message_definition(msg_def):
    # Check if any subfield has another package as in package_name/Messsage
    dependencies = []
    lines = msg_def.split('\n')
    for ll in lines:
        if ll != '' and not ll.startswith('#'):
            subtypes = ll.split('/')
            if len(subtypes) > 1:
                dependencies.append(subtypes[0])
    if not dependencies:
        dependencies.append('std_msgs')
    return dependencies


def create_packages_and_messages_from_definition_and_type(message_type,
                                                          message_definition):
    # Create a temporal workspace
    tmpdir = tempfile.mkdtemp(prefix="catkin_ws")
    tmpdir = tmpdir + '/src'
    os.mkdir(tmpdir)
    print("Working on temporal dir: " + tmpdir)

    print("Working on message: " + message_type)

    # Create first the package
    main_pkg_name, msg_name = message_type.split('/')
    # Keep track of the packages and messages created
    pkgs_msgs = {main_pkg_name: [msg_name]}

    msg_def = message_definition

    sep = '=' * 80 + '\n'
    # Separate in messages
    msgs = msg_def.split(sep)
    # Get top-level type text
    top_level = msgs[0]
    main_pkg_dependencies = get_dependencies_from_message_definition(top_level)
    msgs = msgs[1:]
    create_pkg_path_and_msg_file(main_pkg_name, msg_name, top_level, tmpdir)

    # Get the dependant messages
    for msg in msgs:
        # Description looks like:
        # MSG: geometry_msgs/PoseWithCovariance
        description = msg.split('\n')[0]
        desc = description.replace('MSG: ', '')
        pkg_name, msg_name = desc.split('/')
        msg_def = msg.replace(description + '\n', '')

        dependencies = get_dependencies_from_message_definition(msg_def)
        # Remove duplicates
        dependencies = list(set(dependencies))
        if pkg_name in pkgs_msgs:
            pkgs_msgs[pkg_name].append(msg_name)
        else:
            pkgs_msgs[pkg_name] = [msg_name]

        create_pkg_path_and_msg_file(pkg_name, msg_name, msg_def, tmpdir)
        this_pkg_msgs = pkgs_msgs[pkg_name]
        add_cmakelists_and_packagexml(
            pkg_name, tmpdir, dependencies, this_pkg_msgs)

    main_pkg_msgs = pkgs_msgs[main_pkg_name]
    add_cmakelists_and_packagexml(
        main_pkg_name, tmpdir, main_pkg_dependencies, main_pkg_msgs)

    tmpdir_no_src = tmpdir[:-4]
    num_pkgs = len(pkgs_msgs)
    print("\nGenerated " + str(num_pkgs) + " packages:")
    for k in pkgs_msgs:
        print("  Package: " + k)
        print("     Messages: " + str(pkgs_msgs[k]))
    print("\nDone, your workspace is at: " + tmpdir_no_src)
    return tmpdir_no_src

if __name__ == '__main__':
    # Example input
    conn_d = {'callerid': '/test',
              'latching': '0',
              'md5sum': '9d486b76ee1f72a8b0d33e8c66a97306',
              'message_definition': "Header header\naruco_msgs/Marker[] markers\n\n================================================================================\nMSG: std_msgs/Header\n# Standard metadata for higher-level stamped data types.\n# This is generally used to communicate timestamped data \n# in a particular coordinate frame.\n# \n# sequence ID: consecutively increasing ID \nuint32 seq\n#Two-integer timestamp that is expressed as:\n# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n# time-handling sugar is provided by the client library\ntime stamp\n#Frame this data is associated with\n# 0: no frame\n# 1: global frame\nstring frame_id\n\n================================================================================\nMSG: aruco_msgs/Marker\nHeader header\nuint32 id\ngeometry_msgs/PoseWithCovariance pose\nfloat64 confidence\n\n================================================================================\nMSG: geometry_msgs/PoseWithCovariance\n# This represents a pose in free space with uncertainty.\n\nPose pose\n\n# Row-major representation of the 6x6 covariance matrix\n# The orientation parameters use a fixed-axis representation.\n# In order, the parameters are:\n# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\nfloat64[36] covariance\n\n================================================================================\nMSG: geometry_msgs/Pose\n# A representation of pose in free space, composed of postion and orientation. \nPoint position\nQuaternion orientation\n\n================================================================================\nMSG: geometry_msgs/Point\n# This contains the position of a point in free space\nfloat64 x\nfloat64 y\nfloat64 z\n\n================================================================================\nMSG: geometry_msgs/Quaternion\n# This represents an orientation in free space in quaternion form.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 w\n",
              'topic': '/testpub',
              'type': 'aruco_msgs/MarkerArray'}

    create_packages_and_messages_from_definition_and_type(conn_d['type'],
                                                          conn_d['message_definition'],)
