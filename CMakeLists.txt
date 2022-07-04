cmake_minimum_required(VERSION 3.0.2)
project(front_odometry)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

#find_package用于查找包（通常是使用三方库），并返回关于包的细节（使用包所依赖的头文件、库文件、编译选项、链接选项等）
#find_package(<PackageName> [version] [EXACT] [QUIET] [MODULE]
#             [REQUIRED] [[COMPONENTS] [components...]]
#             [OPTIONAL_COMPONENTS components...]
#             [NO_POLICY_SCOPE])
#PackageName:依赖包名称
#version:版本号，如果指定，那么find_package一定要检查找到的包是否与与version版本兼容
#EXACT：如果指定，表示必须完全匹配version指定版本的包而不是仅仅兼容就可以
#如果指定，表示disable信息输出，即命令运行时不输出相关message，即使没有找到该包。但是如果REQUIRED被指定，还是会有信息输出；
#MODULE：表示只用于module模式，即使没有找到也不会进入CONFIG模式
#REQUIRED：表示此包是必须的，如果没有找到，构建程序报错并终止
#COMPONENTS] [components…]：表示查找的包中必须要找到的组件(components），如果有任何一个找不到就算失败，类似于REQUIRED，导致cmake停止执行
#OPTIONAL_COMPONENTS components…：可选的依赖组件，即使找不到也不会影响cmake继续执行
find_package(catkin REQUIRED COMPONENTS #
  cmake_modules
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  # Eigen3
)

#catkin_package()是catkin提供的CMake宏，用于为catkin提供构建、生成pkg-config和CMake文件所需要的信息
#是给下游（使用该包的包）使用的，它用于向其他包导出依赖，这些依赖可能是本包给其他包提供的公共头文件、库，或者是本包依赖的其他包
#INCLUDE_DIRS - 声明给其它package的include路径
#LIBRARIES - 声明给其它package的库
#CATKIN_DEPENDS - 本包依赖的catkin package
#DEPENDS - 本包依赖的非catkin package
#CFG_EXTRAS - 其它配置参数 
#
catkin_package()

#将指定目录添加到编译器的头文件搜索路径之下，指定的目录被解释成当前源码路径的相对路径
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
include_directories("/usr/local/include/eigen3")
include_directories(include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} )#${EIGEN_DEFINITIONS}

#基于特定的源文件，创建可执行文件
add_executable(front_odometry src/front_odometry.cpp)
#为target连接库的头文件路径和对应定义的函数库路径
target_link_libraries(front_odometry ${catkin_LIBRARIES})
