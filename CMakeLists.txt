cmake_minimum_required(VERSION 3.10)
project(TCS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
file(GLOB_RECURSE SRC src/*.cc)


#test
add_executable(test_event test/component/event/event_test.cc ${SRC})

