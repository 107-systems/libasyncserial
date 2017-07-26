##########################################################################

cmake_minimum_required (VERSION 2.8)

##########################################################################

project (serial)

##########################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build/lib)

##########################################################################

include_directories(include)

##########################################################################

set(LIBSERIAL_NAME ${PROJECT_NAME})

##########################################################################

set(LIBSERIAL_SRCS
  src/Serial.cpp
)

##########################################################################

add_library(
  ${LIBSERIAL_NAME} 
  STATIC
  ${LIBSERIAL_SRCS}
)

target_link_libraries(
  ${LIBSERIAL_NAME}
   
  boost_system 
)

##########################################################################

