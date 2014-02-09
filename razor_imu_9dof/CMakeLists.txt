cmake_minimum_required(VERSION 2.8.3)
project(razor_imu_9dof)

find_package(catkin REQUIRED genmsg std_msgs)

add_message_files(FILES RazorImu.msg)

generate_messages(DEPENDENCIES std_msgs) 

catkin_package(DEPENDS std_msgs)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS nodes/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS nodes/display_3D_visualization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
