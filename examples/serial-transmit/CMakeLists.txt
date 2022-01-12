##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(SERIAL_DEMO_TARGET serial-transmit)
##########################################################################
add_executable(${SERIAL_DEMO_TARGET} main.cpp)
include_directories(../../include)
#target_link_libraries(${SERIAL_DEMO_TARGET} asyncserial)
target_link_libraries(${SERIAL_DEMO_TARGET} boost_system boost_thread pthread)
##########################################################################
