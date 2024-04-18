cmake_minimum_required(VERSION 3.10)
project(TCS)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release
    MinSizeRel RelWithDebInfo)
endif()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
# set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
# set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(TEST "Build test " OFF)
if(DEFINED TEST)
    message(STATUS  "Build test: " ${TEST})
endif()
################
#PahoMqttCpp 依赖openssl 和paho-mqtt-c
find_package(OpenSSL)
find_package(eclipse-paho-mqtt-c)
find_package(PahoMqttCpp)
find_package(Threads REQUIRED)

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
target_link_libraries(tcs PahoMqttCpp::paho-mqttpp3 Threads::Threads)


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
    RUNTIME DESTINATION  ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
    )
  
endif()





