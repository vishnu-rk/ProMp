# Author: Vishnu Radhakrishnan
cmake_minimum_required(VERSION 2.8.12)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


set(PROJECTNAME ProMP)
project(${PROJECTNAME})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})



find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(./include)

add_executable(test1 src/main.cpp
	src/promp.cpp
)

target_link_libraries(test1 ${EIGEN3_LIBRARIES})


add_library(promp SHARED src/promp.cpp)
install(TARGETS promp DESTINATION /usr/lib)
