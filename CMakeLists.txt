cmake_minimum_required(VERSION 3.0)
project(reflectivity_map_builder)
# ビルドフラグの設定
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -pthread")
# インクルードパスの追加
if(WIN32)
    include_directories(C:/local/include)
    link_directories(C:/local/lib)
else()
    include_directories(/usr/local/include)
    include_directories(/opt/ros/foxy/include)
    link_directories(/usr/local/lib)
    link_directories(/opt/ros/foxy/lib)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

include_directories(
    inc
)

# ビルドに関する設定
set(EXECUTABLES
    ReflectivityMapBuilder
)
set(DEPENDENT_LIBS 
    opencv_core
    opencv_highgui
    opencv_imgcodecs
    opencv_imgproc
    sensor_msgs
    geometry_msgs
)
foreach(target IN LISTS EXECUTABLES)
    add_executable(${target} src/${target}.cpp)
    target_link_libraries(${target} ${DEPENDENT_LIBS})
endforeach()