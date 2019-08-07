# cmake版本
cmake_minimum_required(VERSION 3.1.0)
# 项目名称
project(ousterK)

# 全局变量
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# 动态链接库设置enable_pic
option(ENABLE_PIC "Generate position independent code (necessary for shared libraries)" TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ${ENABLE_PIC})

# Package
find_package(Threads)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)

# 包含目录
include_directories(include)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${jsoncpp_INCLUDE_DIRS}
)

# 搜集所有在指定路径下的源文件的文件名，将输出结果列表储存在指定的变量中
 AUX_SOURCE_DIRECTORY(./src DIR_SRCS)

# 生成动态链接库
add_library(ouster_call_lines SHARED ${DIR_SRCS})
target_link_libraries(ouster_call_lines jsoncpp)
