cmake_minimum_required(VERSION 2.8.4)
project(OptiTrack_mavlink)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(MAV_INC_DIR ../../mavlink/c_library/common/)
SET(OPTITRACK_DIR ../OptiTrack)

find_library(OPTITRACK_LIBRARY NAMES OptiTrack PATHS ${OPTITRACK_DIR}/lib/)
SET(OPTITRACK_LIBRARY ${OPTITRACK_LIBRARY} vrpn pthread)

include_directories(${MAV_INC_DIR} ${OPTITRACK_DIR}/include)

set(SOURCE_FILES main.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${OPTITRACK_LIBRARY})
