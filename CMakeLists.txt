
cmake_minimum_required(VERSION 2.8.3)
project(rpg_odroid_io)

IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_library(${PROJECT_NAME} src/gpio.cpp)
  
cs_install()
install(DIRECTORY parameters
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cs_export()
