# ## check c++11 / c++0x
# include(CheckCXXCompilerFlag)
# check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
# check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#     set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)
# elseif(COMPILER_SUPPORTS_CXX0X)
#     set(CMAKE_CXX_COMPILER_ARG1 -std=c++0x)
# else()
#     message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)



cmake_minimum_required(VERSION 2.8.3)
project(hanp_layer)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  costmap_2d
  dynamic_reconfigure
  hanp_msgs
)

# add dynamic reconfigure configs
generate_dynamic_reconfigure_options(
  cfg/HANPLayer.cfg
)

## declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    hanp_layer
  CATKIN_DEPENDS
    roscpp
    tf
    costmap_2d
    dynamic_reconfigure
    hanp_msgs
)



## build ##

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## declare a cpp library
add_library(hanp_layer
  src/hanp_layer.cpp
)


## cmake target dependencies of the executable/library
# add_dependencies(hanp_layer_node hanp_layer_generate_messages_cpp)

# build config headers
add_dependencies(hanp_layer hanp_layer_gencfg)

## libraries to link a library or executable target against
target_link_libraries(hanp_layer
  ${catkin_LIBRARIES}
)



## install ##

## executables and/or libraries for installation
install(TARGETS hanp_layer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## cpp-header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)



## testing ##

## add helpers for testing
add_executable(fake_human_publisher
  src/test/fake_human_publisher_node.cpp
)
target_link_libraries(fake_human_publisher
  ${catkin_LIBRARIES}
)

## add catch based cpp test target
if(COMMAND catkin_add_catchtest)
  catkin_add_catchtest(${PROJECT_NAME}-test src/test/test_hanp_layer.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  endif()
endif()
