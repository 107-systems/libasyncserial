##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
project("asyncserial")
##########################################################################
add_library(${PROJECT_NAME} STATIC src/AsyncSerial.cpp)
##########################################################################
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} boost_system boost_thread pthread)
##########################################################################
add_subdirectory(examples/async-serial-receive)
add_subdirectory(examples/sync-serial-transmit)
##########################################################################
