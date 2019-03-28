cmake_minimum_required(VERSION 2.8.9)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ../)

SET(gtest_build_samples OFF)

project (Robo)

#add_subdirectory(extern/cpr)
#include_directories(${CPR_INCLUDE_DIRS})

#include_directories(extern src)
FILE(GLOB_RECURSE SOURCEFILES_EXTERN "extern/*.cc" )
FILE(GLOB_RECURSE SOURCEFILES_EXTERN_CPP "extern/*.cpp" )
FILE(GLOB_RECURSE HEADERFILES_EXTERN "extern/*.hh" )
FILE(GLOB_RECURSE HEADERFILES_EXTERN_H "extern/*.h" )

FILE(GLOB_RECURSE SOURCEFILES_SRC "src/*.cc" )
FILE(GLOB_RECURSE HEADERFILES_SRC "src/*.hh" )

IF (EXISTS extern_lib/googletest)
  ADD_SUBDIRECTORY(extern_lib/googletest)
  enable_testing()
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} ${gmock_SOURCE_DIR}/include ${gmock_SOURCE_DIR})
  set(GTESTFILES gtest gtest_main gmock gmock_main)
  add_definitions(-DBUILD_WITH_GTEST)
ENDIF()

Find_Package(SDL2)
Find_Package(SDL2TTF)
Find_Package(OpenGL)
Find_Package(CURL)
find_package(Eigen3 REQUIRED)

#enable_testing()
#find_package(GTest REQUIRED)
#include_directories(${GTEST_INCLUDE_DIRS})

include_directories( ${EIGEN3_INCLUDE_DIR} )

if(SDL2_FOUND AND SDL2TTF_FOUND AND OPENGL_FOUND)
  add_definitions(-DBUILD_WITH_SDL)
endif()

add_definitions(-std=c++14 -Wno-deprecated-declarations)

set(ALLFILES ${SOURCEFILES_EXTERN} ${SOURCEFILES_EXTERN_CPP} ${SOURCEFILES_SRC} ${HEADERFILES_EXTERN} ${HEADERFILES_EXTERN_H} ${HEADERFILES_SRC})

add_executable(roboGL mainGL.cc ${ALLFILES})
add_executable(roboCLT mainCLT.cc ${ALLFILES})

target_link_libraries (roboGL ${SDL2_LIBRARY} ${SDL2_TTF_LIBRARY} ${OPENGL_LIBRARIES})
target_link_libraries (roboCLT ${SDL2_LIBRARY} ${SDL2_TTF_LIBRARY} ${OPENGL_LIBRARIES} ${CPR_LIBRARIES} ${CURL_LIBRARIES} ${GTESTFILES})

INSTALL(TARGETS roboGL DESTINATION ../)
INSTALL(TARGETS roboCLT DESTINATION ../)