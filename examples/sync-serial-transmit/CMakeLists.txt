##########################################################################
cmake_minimum_required(VERSION 3.15)
##########################################################################
set(SERIAL_DEMO_TARGET sync-serial-transmit)
##########################################################################
add_executable(${SERIAL_DEMO_TARGET} main.cpp)
target_link_libraries(${SERIAL_DEMO_TARGET} asyncserial)
##########################################################################
