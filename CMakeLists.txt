cmake_minimum_required(VERSION 3.10)
project(TCS)
set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(VISUALIZATION "enable visualization" ON)
set(BOOST_ROOT "/usr/include/boost")
# set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")
find_package(Boost REQUIRED)
if(DEFINED VISUALIZATION)
    message(STATUS  "visualization: " ${VISUALIZATION})
endif()
if(VISUALIZATION)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_definitions(-DVISUAL)
    file(GLOB_RECURSE VISUAL_SRC src/component/visualization/*.cc )
endif()
file(GLOB_RECURSE COMPONENTS_SRC src/component/data/*.cc
 src/component/event/*.cc
)
file(GLOB_RECURSE KERNEL_SRC src/kernel/*.cc )
file(GLOB_RECURSE LOG_SRC src/component/log/*.cc )
file(GLOB_RECURSE XML_SRC src/component/xml/*.cc )
file(GLOB_RECURSE MAIN_SRC src/main/*.cc )


#test
add_executable(test_event test/component/event/event_test.cc ${COMPONENTS_SRC})
add_executable(log_event test/component/log/log_test.cc ${LOG_SRC} )
add_executable(xml_test test/component/xml/xml_test.cc ${XML_SRC})
if(VISUALIZATION)
   add_executable(TCS_test  test/main/tcs_test.cc
   ${KERNEL_SRC}
   ${LOG_SRC}
   ${XML_SRC}
   ${COMPONENTS_SRC}
   ${VISUAL_SRC}
   ${MAIN_SRC}
   )
   target_link_libraries(TCS_test ${OpenCV_LIBS})
else()
   add_executable(TCS_test test/main/tcs_test.cc
   ${KERNEL_SRC}
   ${LOG_SRC}
   ${XML_SRC}
   ${COMPONENTS_SRC}
   ${MAIN_SRC}
   )
endif()




