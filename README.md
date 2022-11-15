<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `libasyncserial`
==============================
A C++ serial interface library based on `boost::asio`.

### How-to-build
```bash
git clone https://github.com/107-systems/libasyncserial && cd libasyncserial
mkdir build && cd build
cmake .. && make
```
or
```bash
cmake -DBUILD_EXAMPLES=ON .. && make
```

### How-to-use
```C++
#include <asyncserial/AsyncSerial.h>

int main(int argc, char **argv)
{
  AsyncSerial serial;

  serial.open("/dev/ttyUSB0", 115200);

  std::future<std::vector<uint8_t>> future = serial.receive(10);
  std::vector<uint8_t> const received_data = future.get();
  std::for_each(std::begin(received_data),
                std::end  (received_data),
                [](uint8_t const ch) { std::cout << std::setw(2) << std::setfill('0') << static_cast<size_t>(ch) << " "; });
  std::cout << std::endl;

  serial.close();

  return EXIT_SUCCESS;
}
```
