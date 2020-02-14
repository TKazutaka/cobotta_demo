cmake_minimum_required(VERSION 2.8.3)
project(cobotta_client)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  denso_gripper_srvs
  denso_state_srvs
  geometry_msgs
  roscpp
  std_srvs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(communicate_master
  src/communicate_master.cpp
  src/communicate_master_node.cpp
)

target_link_libraries(communicate_master
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

set(EXEC_FILES
  communicate_master
  )

foreach(nodes ${EXEC_FILES})
  install(TARGETS ${nodes}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach()

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(DIRECTORIES
  ./launch/
  )

foreach(dirs ${DIRECTORIES})
  install(DIRECTORY ${dirs}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
endforeach()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()