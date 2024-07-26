cmake_minimum_required(VERSION 3.10)
project(TCS VERSION 1.0.0)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release
    MinSizeRel RelWithDebInfo)
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
message(STATUS "Install path: " ${CMAKE_INSTALL_PREFIX})
# set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
# set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(TEST "Build test " OFF)
if(DEFINED TEST)
    message(STATUS  "Build test: " ${TEST})
endif()
################
#PahoMqttCpp 依赖openssl 和paho-mqtt-c
find_package(OpenSSL)
find_package(eclipse-paho-mqtt-c REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(Threads REQUIRED)
################
find_package (Eigen3 REQUIRED NO_MODULE)
include_directories( ${EIGEN3_INCLUDE_DIR} )
################
##
file(GLOB_RECURSE TOOLS_SRC 
 src/component/tools/*.cc
)
file(GLOB_RECURSE MQTT_SRC 
 src/component/vda5050/*.cc
)
file(GLOB_RECURSE KERNEL_SRC src/kernel/*.cc )
file(GLOB_RECURSE MAIN_SRC src/main/*.cc )

#########
#execute#
#########

add_executable(tcs
${KERNEL_SRC}
${TOOLS_SRC}
${MAIN_SRC}
${MQTT_SRC}
)
target_link_libraries(tcs PahoMqttCpp::paho-mqttpp3 Threads::Threads Eigen3::Eigen)



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
    file(GLOB_RECURSE EVENT_SRC src/component/data/*.cc
    src/component/event/*.cc
    )
    add_executable(test_event test/component/event/event_test.cc ${EVENT_SRC})
    add_executable(log_event test/component/tools/log/log_test.cc ${TOOLS_SRC} )
    target_link_libraries(log_event)
    add_executable(xml_test test/component/tools/xml/xml_test.cc  src/component/tools/xml/pugixml.cc)
    add_executable(http_test test/component/tools/http/http_test.cc)
    target_link_libraries(http_test)
    add_executable(json_test test/component/tools/json/json_test.cc 
    ${TOOLS_SRC} 
    )
    add_executable(schema_test test/component/tools/json/schema_test.cc 
    ${TOOLS_SRC} 
    )
    add_executable(mqtt_test test/component/tools/mqtt/mqtt_test.cc  )
    target_link_libraries(mqtt_test PahoMqttCpp::paho-mqttpp3)
    add_executable(sim_agv_test test/kernel/driver/rabbit3_test.cc  ${MQTT_SRC} ${TOOLS_SRC} )
    target_link_libraries(sim_agv_test  PahoMqttCpp::paho-mqttpp3 Threads::Threads)
    install(TARGETS test_event log_event xml_test http_test json_test sim_agv_test mqtt_test
    RUNTIME DESTINATION  ${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME}
    )
  
endif()


#######
###cpack
#######
# include(InstallRequiredSystemLibraries)
# set(CPACK_SOURCE_IGNORE_FILES build)
# set(CPACK_GENERATOR "DEB")
# set(CPACK_PACKAGE_CONTACT "lqk1949@foxmail.com")
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "libpaho-mqttpp-dev")
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "libeigen3-dev")
# set(CPACK_PACKAGE_DESCRIPTION "lqk1949")
# set(CPACK_DEBIAN_PACKAGE_NAME "tcs")
# #########
# #install#
# #########
# install(TARGETS tcs
# RUNTIME DESTINATION  ${CMAKE_INSTALL_PREFIX}/bin
# )
# install(DIRECTORY config DESTINATION  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})
# include(CPack)





