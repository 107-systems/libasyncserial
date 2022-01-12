##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
project("libasyncserial")
##########################################################################
add_library(${PROJECT_NAME} INTERFACE)
##########################################################################
target_include_directories(${PROJECT_NAME} INTERFACE include)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(${PROJECT_NAME} INTERFACE boost_system boost_thread pthread)
##########################################################################
add_subdirectory(examples/serial-receive)
add_subdirectory(examples/serial-transmit)
##########################################################################
