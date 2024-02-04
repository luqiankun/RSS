cmake_minimum_required(VERSION 3.10)
project(TCS)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
"MinSizeRel" "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
# set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(VISUALIZATION "Enable visualization" OFF)
option(TEST "Build test " OFF)
if(DEFINED TEST)
    message(STATUS  "Build test: " ${TEST})
endif()
set(Eigen_DIR "/usr/include/eigen3" CACHE STRING "eigen3 path" )
include_directories(${Eigen_DIR})
#########
#visual##
#########
if(DEFINED VISUALIZATION)
    message(STATUS  "Visualization: " ${VISUALIZATION})
endif()
if(VISUALIZATION)
    find_package(OpenCV REQUIRED)
    if(${OpenCV_FOUND})
    message(STATUS  "OpenCV: Found " "Version " ${OpenCV_VERSION})
    else()
    message(FATAL_ERROR "Can not found OpenCV")
    endif()
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

## 版本低的gcc 可能需要链接pthread
if(VISUALIZATION)
    add_executable(tcs
    ${KERNEL_SRC}
    ${COMPONENTS_SRC}
    ${VISUAL_SRC}
    ${MAIN_SRC}
    )
    target_link_libraries(tcs ${OpenCV_LIBS})
    else()
    add_executable(tcs
    ${KERNEL_SRC}
    ${COMPONENTS_SRC}
    ${MAIN_SRC}
    )
endif()

#########
#install#
#########
install(TARGETS tcs
RUNTIME DESTINATION  ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION  ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})


########
##test##
########

if(TEST)
    add_executable(test_event test/component/event/event_test.cc ${EVENT_SRC})
    add_executable(log_event test/component/tools/log/log_test.cc ${COMPONENTS_SRC} )
    target_link_libraries(log_event)
    add_executable(xml_test test/component/tools/xml/xml_test.cc  src/component/tools/xml/pugixml.cc)
    add_executable(http_test test/component/tools/http/http_test.cc)
    target_link_libraries(http_test)
    add_executable(json_test test/component/tools/json/json_test.cc 
    ${COMPONENTS_SRC} 
    )
    install(TARGETS test_event log_event xml_test http_test json_test
    RUNTIME DESTINATION  ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
    )
endif()





