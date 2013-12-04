# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_teacher)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_python_setup()

add_service_files(
  FILES
  CalibrateCameraFrame.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    DEPENDS  # TODO
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS scripts/cob_teacher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
