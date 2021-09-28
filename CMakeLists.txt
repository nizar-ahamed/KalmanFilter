cmake_minimum_required(VERSION 3.0)

project(kalmanFilter)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)
include_directories(external)
file(GLOB source_files "src/*.cpp" "external/pbPlots/*.cpp")

add_executable(${PROJECT_NAME} ${source_files})

