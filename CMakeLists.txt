cmake_minimum_required(VERSION 2.8.3)
project(robot_assembler)

find_package(catkin REQUIRED COMPONENTS
  roseus euslisp
  )

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
  INCLUDE_DIRS
  LIBRARIES
  )

if(NOT euslisp_INCLUDE_DIRS)
  if(EXISTS ${euslisp_SOURCE_DIR}/jskeus)
    set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
  else()
    set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
  endif()
  message("-- Set euslisp_PACKAGE_PATH to ${euslisp_PACKAGE_PATH}")
  set(euslisp_INCLUDE_DIRS ${euslisp_PACKAGE_PATH}/jskeus/eus/include)
endif()
message("-- Set euslisp_INCLUDE_DIRS to ${euslisp_INCLUDE_DIRS}")
include_directories(/usr/include /usr/X11R6/include ${euslisp_INCLUDE_DIRS} ${assimpdevel_INCLUDE_DIRS})

find_package(yaml-cpp)
add_library(eus_yaml_cpp src/eus_yaml_cpp.cpp)
target_link_libraries(eus_yaml_cpp ${YAML_CPP_LIBRARIES})
set_target_properties(eus_yaml_cpp PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)

# compile flags
add_definitions(-std=c++11 -O2 -Wno-write-strings -Wno-comment)
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"9.00\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions('-DREPOVERSION="\\"${REPOVERSION}\\""')
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* OR
   ${CMAKE_SYSTEM_PROCESSOR} MATCHES aarch64* )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
 add_definitions(-Dx86_64)
 set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -flat_namespace -undefined suppress")
endif()

set_target_properties(eus_yaml_cpp PROPERTIES PREFIX "" SUFFIX ".so")

# install
install(DIRECTORY euslisp launch gazebo sample config meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
