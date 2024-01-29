cmake_minimum_required(VERSION 3.10)
project(TCS)
set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(VISUALIZATION "enable visualization" OFF)
include_directories("/usr/include/eigen3")


#########
#visual##
#########
if(DEFINED VISUALIZATION)
    message(STATUS  "visualization: " ${VISUALIZATION})
endif()
if(VISUALIZATION)
    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    add_definitions(-DVISUAL)
    file(GLOB_RECURSE VISUAL_SRC src/component/visualization/*.cc )
endif()

##
file(GLOB_RECURSE EVENT_SRC src/component/data/*.cc
 src/component/event/*.cc
)
file(GLOB_RECURSE COMPONENTS_SRC 
 src/component/data/*.cc
 src/component/tools/*.cc
 src/component/http/*.cc
)
file(GLOB_RECURSE KERNEL_SRC src/kernel/*.cc )
file(GLOB_RECURSE MAIN_SRC src/main/*.cc )

#########
#execute#
#########


#########
#install#
#########


########
##test##
########
add_executable(test_event test/component/event/event_test.cc ${EVENT_SRC})
add_executable(log_event test/component/tools/log/log_test.cc ${COMPONENTS_SRC} )
target_link_libraries(log_event)
add_executable(xml_test test/component/tools/xml/xml_test.cc  src/component/tools/xml/pugixml.cc)
if(VISUALIZATION)
    add_executable(server_test  test/main/server_test.cc
   ${KERNEL_SRC}
   ${COMPONENTS_SRC}
   ${VISUAL_SRC}
   ${MAIN_SRC}
   )
   target_link_libraries(server_test ${OpenCV_LIBS})
else()
   add_executable(server_test  test/main/server_test.cc
   ${KERNEL_SRC}
   ${COMPONENTS_SRC}
   ${MAIN_SRC}
   )
   target_link_libraries(server_test)
endif()
add_executable(http_test test/component/tools/http/http_test.cc)
target_link_libraries(http_test)
add_executable(json_test test/component/tools/json/json_test.cc 
   ${COMPONENTS_SRC} 
)





