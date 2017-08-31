##########################################################################

cmake_minimum_required (VERSION 2.8)

##########################################################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

##########################################################################

set(SERIAL_DEMO_TARGET serial-receive)

##########################################################################

set(SERIAL_DEMO_TARGET_SRCS
  main.cpp
)

##########################################################################

add_executable(
  ${SERIAL_DEMO_TARGET}
  ${SERIAL_DEMO_TARGET_SRCS}
)

target_link_libraries(
  ${SERIAL_DEMO_TARGET}
  serial
  pthread
)

##########################################################################
