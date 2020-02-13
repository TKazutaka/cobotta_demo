cmake_minimum_required(VERSION 2.8.3)
project(cobotta_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf2_ros
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

###########
## Build ##
###########

add_executable(cobotta_server
  src/cobotta_server.cpp
)
target_link_libraries(cobotta_server ${catkin_LIBRARIES})

###########
# Install #
###########

# set(EXEC_FILES
#   crop_cloud_node
# )
# # Install executables
# foreach(nodes ${EXEC_FILES})
#   install(TARGETS ${nodes}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# endforeach()
#
# set(DIRECTORIES
#   ./launch/
#   )
# # Install all files
# foreach(dirs ${DIRECTORIES})
#   install(DIRECTORY ${dirs}/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dirs})
# endforeach()
#

#############
## Testing ##
#############

# if (CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#
#   set(DIRECTORIES
#     ./launch/
#     )
#
#   foreach(dirs ${DIRECTORIES})
#     roslaunch_add_file_check(${dirs})
#   endforeach()
# endif()
