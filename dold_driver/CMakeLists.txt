cmake_minimum_required(VERSION 2.8.3)
project(dold_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  dold_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dold_driver
  CATKIN_DEPENDS roscpp serial dold_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(dold_driver src/dold_driver.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(dold_driver 
  ${catkin_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS dold_driver 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

