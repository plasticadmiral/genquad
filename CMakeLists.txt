cmake_minimum_required(VERSION 2.8.3)
project(genquad)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)

# Find the path to the pigpio includes.
find_path(pigpio_INCLUDE_DIR
        NAMES pigpio.h pigpiod_if.h pigpiod_if2.h
        HINTS /usr/local/include)

# Find the pigpio libraries.
find_library(pigpio_LIBRARY
        NAMES libpigpio.so
        HINTS /usr/local/lib)
find_library(pigpiod_if_LIBRARY
        NAMES libpigpiod_if.so
        HINTS /usr/local/lib)
find_library(pigpiod_if2_LIBRARY
        NAMES libpigpiod_if2.so
        HINTS /usr/local/lib)

# Set the pigpio variables to plural form to make them accessible for
# the paramount cmake modules.
set(pigpio_INCLUDE_DIRS ${pigpio_INCLUDE_DIR})
set(pigpio_INCLUDES     ${pigpio_INCLUDE_DIR})

# Handle REQUIRED, QUIET, and version arguments
# and set the <packagename>_FOUND variable.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pigpio
    DEFAULT_MSG
    pigpio_INCLUDE_DIR pigpio_LIBRARY pigpiod_if_LIBRARY pigpiod_if2_LIBRARY)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    )


#set(CMAKE_CXX_FLAGS "-Wall  -lpigpiod_if2 -lpthread"  ${CMAKE_CXX_FLAGS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(
    FILES
    bmp280_dat.msg
    mpu9250_dat.msg
    )


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_service_files(
    FILES
    ESCControl.srv
    )


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

generate_messages(
    DEPENDENCIES
    std_msgs geometry_msgs sensor_msgs
    )


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES genquad
    CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs sensor_msgs
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(${Boost_LIBRARIES})

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    ${boost_INCLUDE_DIR}
    ${pigpio_INCLUDE_DIRS}
    )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/genquad.cpp
# )


add_library(libbmp2801 src/bmp280.h src/bmp280.cpp)
add_library(libmpu9250 src/mpu9250.h src/mpu9250.cpp)
add_library(libesccontrol src/esccontrol.h src/esccontrol.cpp)


target_link_libraries(libbmp2801 ${pigpiod_if2_LIBRARY})
target_link_libraries(libmpu9250 ${pigpiod_if2_LIBRARY})
target_link_libraries(libesccontrol ${pigpiod_if2_LIBRARY})

set_target_properties(libbmp2801
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

set_target_properties(libmpu9250
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

set_target_properties(libesccontrol
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/genquad_node.cpp)

add_executable(dummy_node nodes/dummy_node.cpp)
add_executable(bmp280_node nodes/bmp280_node.cpp)
add_executable(mpu9250_node nodes/mpu9250_node.cpp)
add_executable(esccontrol_node nodes/esccontrol_node.cpp)


add_dependencies(bmp280_node genquad_generate_messages_cpp)
add_dependencies(mpu9250_node genquad_generate_messages_cpp)
add_dependencies(esccontrol_node genquad_generate_messages_cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(dummy_node ${catkin_LIBRARIES})
target_link_libraries(bmp280_node libbmp2801 ${catkin_LIBRARIES})
target_link_libraries(mpu9250_node libmpu9250 ${catkin_LIBRARIES})
target_link_libraries(esccontrol_node libesccontrol ${catkin_LIBRARIES} ${Boost_LIBRARIES})

set_target_properties(dummy_node
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(bmp280_node
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(mpu9250_node
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(esccontrol_node
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/bin
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_genquad.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
