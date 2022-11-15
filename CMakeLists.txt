##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
project("asyncserial")
##########################################################################
option(BUILD_EXAMPLES "Build all examples provided with this library" OFF)
##########################################################################
add_library(${PROJECT_NAME} STATIC src/AsyncSerial.cpp)
##########################################################################
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} boost_system boost_thread pthread)
##########################################################################
if(BUILD_EXAMPLES)
  add_subdirectory(examples/async-serial-receive)
  add_subdirectory(examples/sync-serial-transmit)
endif()
##########################################################################
