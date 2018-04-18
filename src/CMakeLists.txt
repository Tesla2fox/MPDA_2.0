#include_directories (../include)

cmake_minimum_required(VERSION 3.0)

message(StaticConstrn)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



set(file_list
	stadfx.h
	ssconfig.hpp
	ReadConfig.h
	ReadConfig.cpp
	tutorial.cpp
	TaskPnt.cpp
	TaskPnt.hpp
	OrgEncodeMat.cpp
	OrgEncodeMat.h
	#
	StaticConstrn.hpp
	StaticConstrn.cpp
	
)

#add—boost
find_package(Boost 1.65.0)
if(Boost_FOUND)
set(Boost_USE_STATIC_LIBS        ON) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
include_directories(${Boost_INCLUDE_DIRS})
message(${Boost_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS thread)
message(${Boost_LIBRARY_DIRS})
endif()


FIND_PACKAGE( OpenMP REQUIRED)  
if(OPENMP_FOUND)  
message("OPENMP FOUND")  
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")  
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")  
endif()  

include_directories(D:\\Eigen3)

message(${Boost_LIBRARIES})

add_executable(mpda_StaticConstrn ${file_list})

set_target_properties(mpda_StaticConstrn PROPERTIES LINKER_LANGUAGE CXX)

#target_link_libraries(mpda_sConstruct Boost::thread)




