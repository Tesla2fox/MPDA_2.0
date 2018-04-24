
#最小版本的要求
cmake_minimum_required(VERSION 3.0)

#项目名称
project(MPDA_Constrn)

message("PROJECT_SOURCE_DIR" ${PROJECT_SOURCE_DIR})
#添加外部dll
#add_library(libZMQ SHARED IMPORTED)
#set_target_properties(libZMQ PROPERTIES
#	IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libzmq.dll 
#	IMPORTED_IMPLIB   ${PROJECT_SOURCE_DIR}/lib/libzmq.lib 
#)

#测试主函数的文件夹
add_subdirectory(OrgSrc mpda_OrgStaticConstrn)
