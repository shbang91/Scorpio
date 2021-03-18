cmake_minimum_required(VERSION 2.8.3)
project(gazebo_scorpio_plugin)

##=============================================================================
## CMAKE Flags
##=============================================================================
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

##=============================================================================
## DEPENDANCY
##=============================================================================

#system
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

##TODO:Replace this with Config
include_directories("~/catkin_ws/src/PnC")
#include_directories("~/code/ut/PnC")

#gazebo & catkin
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp rospy std_msgs geometry_msgs control_msgs message_generation)

add_service_files(
  FILES
  MoveEndEffectorToSrv.srv
  GripperCommandSrv.srv
  InterruptSrv.srv
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

generate_messages(DEPENDENCIES std_msgs geometry_msgs control_msgs)

catkin_package(DEPENDS roscpp rospy gazebo_ros message_runtime std_msgs geometry_msgs control_msgs)

#dart
#find_package(DART 6.9 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
#include_directories(${DART_INCLUDE_DIRS})

#Eigen
if(NOT EIGEN_FOUND)
find_package(Eigen)
endif()
include_directories(${EIGEN_INCLUDE_DIRS})

#PnC
find_package(PnC REQUIRED)
include_directories(${PNC_INCLUDE_DIRS})

add_executable(mock_node src/scorpio_mock_node.cpp)
target_link_libraries(mock_node ${catkin_LIBRARIES})

##=============================================================================
## BUILD SETUP
##=============================================================================

add_library(${PROJECT_NAME} SHARED src/scorpio_plugin.cpp)
target_link_libraries(${PROJECT_NAME}
                    ${catkin_LIBRARIES}
                    ${GAZEBO_LIBRARIES}
                    ${PNC_LIBRARIES}
                    )
add_dependencies(mock_node ${gazebo_scorpio_plugin_EXPORTED_TARGETS})

