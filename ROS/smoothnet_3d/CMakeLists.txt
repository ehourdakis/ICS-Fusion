cmake_minimum_required(VERSION 2.8.3)
project(smoothnet_3d)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
)

# add_message_files(
#     FILES
#     time_msg.msg
# )

catkin_python_setup()

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

find_package(catkin REQUIRED COMPONENTS 
    actionlib_msgs
    actionlib    
    message_generation
)

add_action_files(
  DIRECTORY action
  FILES SmoothNet3d.action
)

generate_messages(DEPENDENCIES 
    actionlib_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    actionlib_msgs
)

set( nodes 
    src/smoothnet_3d_server.py
    src/rand_keypts_node.py
    src/test_registration_node.py
)

catkin_install_python(PROGRAMS
    ${nodes}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
