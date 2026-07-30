cmake_minimum_required(VERSION 3.16)
project(ball_vision_sender LANGUAGES CXX)

find_package(OpenCV REQUIRED)
add_executable(ball_vision_sender ball_vision_sender.cpp)
target_compile_features(ball_vision_sender PRIVATE cxx_std_17)
target_compile_options(ball_vision_sender PRIVATE -O3 -Wall -Wextra)
target_link_libraries(ball_vision_sender PRIVATE ${OpenCV_LIBS} pthread)
