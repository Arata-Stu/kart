cmake_minimum_required(VERSION 3.16)

project(silkyevcam_bias_tuner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
    MetavisionSDK
    COMPONENTS
    base
    stream
    REQUIRED
)

find_package(MetavisionHAL REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(
    silkyevcam_bias_tuner
    main.cpp
)

target_link_libraries(
    silkyevcam_bias_tuner
    PRIVATE
    MetavisionSDK::base
    MetavisionSDK::stream
    Metavision::HAL
    ${OpenCV_LIBS}
)

target_include_directories(silkyevcam_bias_tuner PRIVATE ${OpenCV_INCLUDE_DIRS})
