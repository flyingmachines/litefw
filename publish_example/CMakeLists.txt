cmake_minimum_required(VERSION 2.6)
project(pub)

include_directories(include)

set(CMAKE_CXX_FLAGS "-Wall -lboost_iostreams -lboost_system -lboost_filesystem -pthread -std=c++11")

#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/config")

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

FIND_PACKAGE(Boost 1.58.0 COMPONENTS serialization program_options chrono regex system filesystem thread REQUIRED)
MESSAGE(${Boost_INCLUDE_DIR}/boost)
MESSAGE(${Boost_LIBRARY_DIRS})


if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR}/boost ${Boost_LIBRARY_DIRS} ${ZeroMQ_INCLUDE_DIR}) 
	ADD_EXECUTABLE( pub src/pub.cpp)
	TARGET_LINK_LIBRARIES( pub  Threads::Threads ${Boost_LIBRARIES} ${ZeroMQ_LIBRARY})
endif()

#if(CMAKE_COMPILER_IS_GNUCXX)
# 	set(CMAKE_CXX_FLAGS "-O3")
#	set(CMAKE_EXE_LINKER_FLAGS "-s")
#endif()
