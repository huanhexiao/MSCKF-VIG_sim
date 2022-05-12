cmake_minimum_required(VERSION 2.8.0)
project(tools)

# set compiler option
# SET(CMAKE_CXX_COMPILE "g++")
# SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O2 -pthread " )
SET( CMAKE_BUILD_TYPE "Debug")

# set output path of executable and libraries
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# find_package(glog REQUIRED)
# find_package(Eigen3 REQUIRED)

# include some directories of header file
# include_directories(${GLOG_INCLUDE_DIRS} 
                    # ${EIGEN3_INCLUDE_DIRS}) 

include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(${PROJECT_SOURCE_DIR}/src cpp_files)

add_library(${PROJECT_NAME} SHARED ${cpp_files})
target_link_libraries(${PROJECT_NAME}  ${GLOG_LIBRARY})

# add_subdirectory(${PROJECT_SOURCE_DIR}/test)