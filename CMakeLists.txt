cmake_minimum_required(VERSION 3.10)
project(RobotSchedulingSystem VERSION 1.0.0)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release
            MinSizeRel RelWithDebInfo)
endif ()
###
set(EXE_NAME rss)
###
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
message(STATUS "Install path: " ${CMAKE_INSTALL_PREFIX})
# set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
# set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
endif ()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(TEST "Build test " OFF)
if (DEFINED TEST)
    message(STATUS "Build test: " ${TEST})
endif ()
################
#PahoMqttCpp 依赖openssl 和paho-mqtt-c
find_package(OpenSSL)
find_package(eclipse-paho-mqtt-c REQUIRED)
find_package(PahoMqttCpp REQUIRED)
find_package(Threads REQUIRED)
################
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
################
##
file(GLOB_RECURSE 3rd_SRC
        src/3rdparty/*.cc
)
file(GLOB_RECURSE KERNEL_SRC src/kernel/*.cc)
file(GLOB_RECURSE MAIN_SRC src/main/*.cc)

#########
#execute#
#########

add_executable(${EXE_NAME}
        ${KERNEL_SRC}
        ${3rd_SRC}
        ${MAIN_SRC}
)
target_link_libraries(${EXE_NAME} PahoMqttCpp::paho-mqttpp3 Threads::Threads Eigen3::Eigen)


#########
#install#
#########
install(TARGETS ${EXE_NAME}
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME})

########
##test##
########

if (TEST)
    add_executable(log_event test/3rdparty/log/log_test.cc src/3rdparty/log/easylogging++.cc)
    target_link_libraries(log_event)
    add_executable(xml_test test/3rdparty/xml/xml_test.cc src/3rdparty/xml/pugixml.cc)
    add_executable(http_test test/3rdparty/http/http_test.cc)
    add_executable(json_test test/3rdparty/json/json_test.cc
            src/3rdparty/log/easylogging++.cc
    )
    add_executable(schema_test test/3rdparty/json/schema_test.cc
            src/3rdparty/log/easylogging++.cc
    )
    add_executable(nurbe_test test/kernel/nurbs_test.cc)
    add_executable(mqtt_test test/3rdparty/mqtt/mqtt_test.cc)
    target_link_libraries(mqtt_test PahoMqttCpp::paho-mqttpp3)
    add_executable(sim_agv_test test/kernel/driver/rabbit3_test.cc ${3rd_SRC})
    target_link_libraries(sim_agv_test PahoMqttCpp::paho-mqttpp3 Threads::Threads)
    install(TARGETS nurbe_test log_event xml_test http_test json_test sim_agv_test mqtt_test
            RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/${PROJECT_NAME}
    )

endif ()


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
# set(CPACK_DEBIAN_PACKAGE_NAME "${EXE_NAME}")
# #########
# #install#
# #########
# install(TARGETS ${EXE_NAME}
# RUNTIME DESTINATION  ${CMAKE_INSTALL_PREFIX}/bin
# )
# install(DIRECTORY config DESTINATION  ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})
# include(CPack)





