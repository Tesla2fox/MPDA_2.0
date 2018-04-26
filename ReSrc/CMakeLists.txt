
cmake_minimum_required(VERSION 3.0)

message(mpda_ReStaticConstrn)

include_directories (../DecodeSrc)

set(file_dir ../DecodeSrc)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



set(file_list
	${file_dir}/stadfx.h
	${file_dir}/ssconfig.hpp
	${file_dir}/ReadConfig.h
	${file_dir}/ReadConfig.cpp
	${file_dir}/TaskPnt.cpp
	${file_dir}/TaskPnt.hpp
	#OrgEncodeMatR表示release
	${file_dir}/OrgEncodeMatR.cpp
	${file_dir}/OrgEncodeMatR.h
	#
	tutorialRe.cpp
	StaticConstrnRe.hpp
	StaticConstrnRe.cpp	
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

add_executable(mpda_ReStaticConstrn ${file_list})

set_target_properties(mpda_ReStaticConstrn PROPERTIES LINKER_LANGUAGE CXX)

