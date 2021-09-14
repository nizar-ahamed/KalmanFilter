cmake_minimum_required(VERSION 3.0)

project(kalmanFilter)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)
file(GLOB source_files "src/*.cpp")

add_executable(${PROJECT_NAME} ${source_files})

