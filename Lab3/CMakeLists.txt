cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer PRIVATE Eigen3::Eigen ${OpenCV_LIBRARIES})
# target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)