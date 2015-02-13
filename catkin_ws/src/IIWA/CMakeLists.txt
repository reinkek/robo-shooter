cmake_minimum_required(VERSION 2.8.3)
project(IIWA)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

# Declare the message files to be built
add_message_files(FILES IIWAMsg.msg)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES	 ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime
)

SET(OPTITRACK_DIR ../../../OptiTrack)
SET(SCL_DIR ${OPTITRACK_DIR}/../../scl-manips-v2.git)
SET(SUTIL_INC_DIR ${SCL_DIR}/3rdparty/sUtil/src/)
SET(TAO_INC_DIR ${SCL_DIR}/src/scl/dynamics/tao/)
SET(CHAI_INC_DIR ${SCL_DIR}/3rdparty/chai3d-3.0/chai3d/)
SET(IPOPT_DIR ${SCL_DIR}/../Ipopt-3.11.9/build)
SET(IPOPT_INC_DIR ${IPOPT_DIR}/include/coin/)
ADD_DEFINITIONS(-D_LINUX -DLINUX)

#Set the compilation flags
SET(CMAKE_CXX_FLAGS "-Wall -fPIC -fopenmp")
SET(CMAKE_CXX_FLAGS_DEBUG "-ggdb -O0 -pg -std=c++0x -DGRAPHICS_ON -DASSERT=assert -DDEBUG=1")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++0x -DGRAPHICS_ON -DW_THREADING_ON -DNDEBUG")

find_library(OPTITRACK_LIBRARY NAMES OptiTrack PATHS ${OPTITRACK_DIR}/lib/)
SET(OPTITRACK_LIBRARY ${OPTITRACK_LIBRARY} vrpn pthread)

set(OPTIMIZER ON)
if(OPTIMIZER)
find_library(IPOPT_LIBRARY NAMES ipopt PATHS ${IPOPT_DIR}/lib/)
find_library(HSL_LIBRARY NAMES coinhsl PATHS ${IPOPT_DIR}/lib/)
find_library(BLAS_LIBRARY NAMES coinblas PATHS ${IPOPT_DIR}/lib/)
find_library(MUMPS_LIBRARY NAMES coinmumps PATHS ${IPOPT_DIR}/lib/)
find_library(METIS_LIBRARY NAMES coinmetis PATHS ${IPOPT_DIR}/lib/)
find_library(LAPACK_LIBRARY NAMES coinlapack PATHS ${IPOPT_DIR}/lib/)
SET(IPOPT_LIBRARY ${IPOPT_LIBRARY} ${HSL_LIBRARY} ${BLAS_LIBRARY} ${MUMPS_LIBRARY} ${METIS_LIBRARY} ${LAPACK_LIBRARY})
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${OPTITRACK_DIR}/include ${Eigen_INCLUDE_DIRS} ${SCL_DIR}/src/ ${SUTIL_INC_DIR} ${TAO_INC_DIR} ${CHAI_INC_DIR} ${IPOPT_INC_DIR})

add_library(${PROJECT_NAME} src/IIWARobot.cpp include/IIWARobot.h)

add_executable(${PROJECT_NAME}-bin src/main.cpp ${SCL_DIR}/src/scl/graphics/chai/CGraphicsChai.cpp 
                                                ${SCL_DIR}/src/scl/graphics/chai/ChaiGlutHandlers.cpp
                                                src/Trajectory_nlp.cpp)

target_link_libraries(${PROJECT_NAME}-bin ${PROJECT_NAME} ${catkin_LIBRARIES} ${OPTITRACK_LIBRARY} ${Eigen_LIBRARIES} ${IPOPT_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK SCL's LIB DIR ######################
find_library( SCL_LIBRARY_DEBUG NAMES scl
            PATHS   ${SCL_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES debug )

find_library( SCL_LIBRARY_RELEASE NAMES scl
            PATHS   ${SCL_DIR}/applications-linux/scl_lib/
            PATH_SUFFIXES release )

SET( SCL_LIBRARY debug     ${SCL_LIBRARY_DEBUG}
              optimized ${SCL_LIBRARY_RELEASE} )

target_link_libraries(${PROJECT_NAME}-bin ${SCL_LIBRARY})

###############SPECIAL CODE TO FIND AND LINK CHAI's LIB DIR ######################
find_library( CHAI_LIBRARY_DEBUG NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES debug )

find_library( CHAI_LIBRARY_RELEASE NAMES chai3d
            PATHS   ${CHAI_INC_DIR}../lib/
            PATH_SUFFIXES release )

SET( CHAI_LIBRARY debug     ${CHAI_LIBRARY_DEBUG}
              optimized ${CHAI_LIBRARY_RELEASE} )

target_link_libraries(${PROJECT_NAME}-bin ${CHAI_LIBRARY})


###############CODE TO FIND AND LINK REMANING LIBS ######################
target_link_libraries(${PROJECT_NAME}-bin gomp GL GLU GLEW glut ncurses rt dl)

#FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
#SET(QT_USE_QTOPENGL true)
#INCLUDE(${QT_USE_FILE})
#INCLUDE_DIRECTORIES(${QT_INCLUDES})
#target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
