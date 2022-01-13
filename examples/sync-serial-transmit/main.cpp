/**
 * This software is distributed under the terms of the LGPL v2.1.
 * Copyright (c) 2022 Alexander Entinger, LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/libasyncserial/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>
#include <stdint.h>

#include <vector>

#include <asyncserial/AsyncSerial.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  AsyncSerial serial;

  serial.open("/dev/ttyUSB0", 115200);

  std::vector<uint8_t> const data = {0xDE, 0xAD, 0xBE, 0xEF};
  serial.transmit(data);

  serial.close();

  return EXIT_SUCCESS;
}
