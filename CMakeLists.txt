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
file(GLOB_RECURSE TOOLS_SRC src/component/tools/*.cc )
file(GLOB_RECURSE MAIN_SRC src/main/*.cc )


#test
add_executable(test_event test/component/event/event_test.cc ${COMPONENTS_SRC})
add_executable(log_event test/component/tools/log/log_test.cc ${TOOLS_SRC} )
add_executable(xml_test test/component/tools/xml/xml_test.cc  src/component/tools/xml/pugixml.cc)
if(VISUALIZATION)
   add_executable(TCS_test  test/main/tcs_test.cc
   ${KERNEL_SRC}
   ${TOOLS_SRC}
   ${COMPONENTS_SRC}
   ${VISUAL_SRC}
   ${MAIN_SRC}
   )
   target_link_libraries(TCS_test ${OpenCV_LIBS})
    add_executable(server_test  test/main/server_test.cc
   ${KERNEL_SRC}
   ${TOOLS_SRC}
   ${COMPONENTS_SRC}
   ${VISUAL_SRC}
   ${MAIN_SRC}
   )
   target_link_libraries(server_test ${OpenCV_LIBS})
else()
   add_executable(TCS_test test/main/tcs_test.cc
   ${KERNEL_SRC}
   ${TOOLS_SRC}
   ${COMPONENTS_SRC}
   ${MAIN_SRC}
   )
   add_executable(server_test  test/main/server_test.cc
   ${KERNEL_SRC}
   ${TOOLS_SRC}
   ${COMPONENTS_SRC}
   ${MAIN_SRC}
   )
endif()
add_executable(http_test test/component/tools/http/http_test.cc)
add_executable(json_test test/component/tools/json/json_test.cc 
   ${TOOLS_SRC}
)





