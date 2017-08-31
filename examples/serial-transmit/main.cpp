/**
 * libserial provides an easy to use C++ interface for the serial interface using boost::asio.
 * Copyright (C) 2017 Alexander Entinger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <stdlib.h>
#include <stdint.h>

#include <vector>

#include <serial/Serial.h>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static std::string const SERIAL_DEV_NODE  = "/dev/ttyUSB0";
static size_t      const SERIAL_BAUD_RATE = 115200;

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char **argv)
{
  serial::Serial serial;

  serial.open(SERIAL_DEV_NODE, SERIAL_BAUD_RATE);

  std::vector<uint8_t> const data = {0xDE, 0xAD, 0xBE, 0xEF};

  serial.transmit(data);

  serial.close();

  return EXIT_SUCCESS;
}
