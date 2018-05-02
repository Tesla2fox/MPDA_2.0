
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

#原来的静态的排序方法
add_subdirectory(OrgStaticSrc mpda_OrgStaticConstrn)
#随机生成排序方法
add_subdirectory(RandSrc mpda_RandConstrn)
#新的静态的排序方法
add_subdirectory(ReStaticSrc mpda_ReStaticConstrn)
#原来的动态排序方法
add_subdirectory(OrgDynamicSrc mpda_OrgDynamicConstrn)