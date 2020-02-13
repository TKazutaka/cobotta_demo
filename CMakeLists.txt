cmake_minimum_required(VERSION 2.8.3)
project(cobotta_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

###########
## Build ##
###########

add_executable(crop_cloud_node
  src/crop_cloud.cpp
  src/crop_cloud_node.cpp)
target_link_libraries(crop_cloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

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
