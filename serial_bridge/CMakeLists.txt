cmake_minimum_required(VERSION 2.6)
project(serial_bridge)

include_directories(include)
include_directories(c_library_v1)

#set(EXECUTABLE_OUTPUT_PATH "../bin")
set(CMAKE_CXX_FLAGS "-Wall -lboost_iostreams -lboost_system -lboost_filesystem -pthread -std=c++11")

#if (ZMQ_STATIC)
#    add_definitions(-D ZMQ_STATIC)
#endif()

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)

find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

#if (ZMQ_STATIC)
#    add_definitions(-D ZMQ_STATIC)
#endif()



#find_package(ZeroMQ 4.0 REQUIRED)

FIND_PACKAGE(Boost 1.58.0 COMPONENTS serialization program_options chrono regex system filesystem thread REQUIRED)
MESSAGE(${Boost_INCLUDE_DIR}/boost)
MESSAGE(${Boost_LIBRARY_DIRS})


#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)
#add_definitions(-DZMQ_BUILD_DRAFT_API=1)
#find_package(ZeroMQ)

#find_package(ZMQ REQUIRED)
#find_package(CapnProto REQUIRED)
#MESSAGE(${CAPNP_LIBRARIES})
#find_library(CAPNP_LIB_CAPNP-RPC capnp-rpc
#  HINTS "${PKGCONFIG_CAPNP_RPC_LIBDIR}" ${PKGCONFIG_CAPNP_RPC_LIBRARY_DIRS}
#)
#find_library(CAPNP_LIB_CAPNP capnp
#  HINTS "${PKGCONFIG_CAPNP_LIBDIR}" ${PKGCONFIG_CAPNP_LIBRARY_DIRS}
#)
#MESSAGE(${CAPNP_LIB_CAPNP})

#set (SERIAL_PATH $ENV{PROJECT_ROOT}/include/common)
#set (SERIAL_INCLUDE_PATH $ENV{PROJECT_ROOT}/include)
#set(CAPNPC_OUTPUT_DIR ~/qbase/rabbitmq/capnpfiles)
#MESSAGE(${CAPNP_SRCS})


#set(Boost_DEBUG TRUE)
#find_package(boost REQUIRED COMPONENTS unit_test_framework)
#message(STATUS "Boost_VERSION=" ${Boost_VERSION})
#message(STATUS "Boost_LIBRARIES=" ${Boost_LIBRARIES})
#message(STATUS "Boost_LIBRARY_DIRS=" ${Boost_LIBRARY_DIRS})
#message(STATUS "Boost_INCLUDE_DIRS=" ${Boost_INCLUDE_DIRS})

#include_directories(${SERIAL_PATH} ${SERIAL_INCLUDE_PATH})

if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR}/boost ${Boost_LIBRARY_DIRS} ${ZeroMQ_INCLUDE_DIR}) 
	ADD_EXECUTABLE( sb src/serial_create.cpp src/SerialPort.cpp)
	#ADD_EXECUTABLE( sub sub.cpp)
	#ADD_EXECUTABLE( pub pub.cpp)
	#target_include_directories(pub PUBLIC ${ZeroMQ_INCLUDE_DIR})	
	#ADD_EXECUTABLE( sub sub.cpp)
	TARGET_LINK_LIBRARIES( sb Threads::Threads ${Boost_LIBRARIES} ${ZeroMQ_LIBRARY})
	#TARGET_LINK_LIBRARIES( sub  Threads::Threads ${Boost_LIBRARIES} ${ZeroMQ_LIBRARY})
	#TARGET_LINK_LIBRARIES( pub  Threads::Threads ${Boost_LIBRARIES} ${ZeroMQ_LIBRARY})
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
 	set(CMAKE_CXX_FLAGS "-O3")
	set(CMAKE_EXE_LINKER_FLAGS "-s")
endif()
