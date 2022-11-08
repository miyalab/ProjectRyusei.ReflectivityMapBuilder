cmake_minimum_required(VERSION 3.0)
project(reflectivity_map_builder)
# ビルドフラグの設定
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -pthread")
# インクルードパスの追加
if(WIN32)
    include_directories(C:/local/include)
    link_directories(C:/local/lib)
else()
    include_directories(
        /usr/local/include
        /opt/ros/foxy/include
    )
    link_directories(/usr/local/lib)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
    inc
    lib
    ~/workspace/ros2_ws/install/ros_robocore_interfaces/include
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
)

add_library(${PROJECT_NAME} STATIC
    lib/point_rotational_transform/point_rotational_transform.cpp
)

foreach(target IN LISTS EXECUTABLES)
    add_executable(${target} src/${target}.cpp)
    target_link_libraries(${target} ${DEPENDENT_LIBS} ${PROJECT_NAME})
endforeach()