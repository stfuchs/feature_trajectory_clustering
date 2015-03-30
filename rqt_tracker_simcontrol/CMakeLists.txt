cmake_minimum_required(VERSION 2.8.3)
project(rqt_tracker_simcontrol)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_package()
catkin_python_setup()

install(PROGRAMS scripts/rqt_tracker_simcontrol
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(FILES plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY resource
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(PROGRAMS scripts/rqt_reconfigure
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
