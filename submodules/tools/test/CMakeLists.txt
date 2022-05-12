cmake_minimum_required(VERSION 2.8)
project(tools_test)

add_executable(test_log ${PROJECT_SOURCE_DIR}/NavLog.cc)
add_executable(test_time ${PROJECT_SOURCE_DIR}/NavTime.cc)
add_executable(test_attitude ${PROJECT_SOURCE_DIR}/NavAttitude.cc)

target_link_libraries(test_log tools ${GLOG_LIBRARY})
target_link_libraries(test_time tools ${GLOG_LIBRARY})
target_link_libraries(test_attitude tools ${GLOG_LIBRARY})
