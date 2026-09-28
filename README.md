# i2c-bus

What hwlib's I2C drivers need from a bus, as C++20 concepts — and a fake bus to
test them with. Header-only; no platform code, no allocation outside the fake.

Part of hwlib. A driver takes the bus as a template parameter and requires only
the transfers it makes; the application adapts its own HAL in a few lines.

## The concepts

`address` is the 7-bit address. Each call returns whether the device acknowledged;
timeouts, retries and bus recovery belong to the bus.

| Concept | Member | On the wire |
|---|---|---|
| `I2cWrite` | `bool Write(std::uint8_t address, std::span<const std::uint8_t> tx)` | START, address + W, bytes, STOP |
| `I2cRead` | `bool Read(std::uint8_t address, std::span<std::uint8_t> rx)` | START, address + R, bytes, STOP |
| `I2cWriteRead` | `bool WriteRead(std::uint8_t address, std::span<const std::uint8_t> tx, std::span<std::uint8_t> rx)` | a write, a repeated START, a read — a register read |
| `I2cBus` | `I2cWrite && I2cWriteRead` | what a register-based device needs |

## Use it

```cmake
add_subdirectory(external/hwlib/i2c-bus)   # before the drivers that use it
```

On Zephyr:

```cpp
#include <hwlib/drivers/i2c_bus.hpp>
#include <zephyr/drivers/i2c.h>

struct ZephyrI2c
{
    const device* dev;

    bool Write(std::uint8_t address, std::span<const std::uint8_t> tx)
    {
        return i2c_write(dev, tx.data(), tx.size(), address) == 0;
    }
    bool Read(std::uint8_t address, std::span<std::uint8_t> rx)
    {
        return i2c_read(dev, rx.data(), rx.size(), address) == 0;
    }
    bool WriteRead(std::uint8_t address, std::span<const std::uint8_t> tx, std::span<std::uint8_t> rx)
    {
        return i2c_write_read(dev, address, tx.data(), tx.size(), rx.data(), rx.size()) == 0;
    }
};
static_assert(hwlib::drivers::I2cBus<ZephyrI2c> && hwlib::drivers::I2cRead<ZephyrI2c>);
```

A bus that returns a status code rather than `bool` does not satisfy the concepts
on purpose: the adapter says which codes mean success.

## Testing a driver

`hwlib/drivers/testing/fake_i2c_bus.hpp` has `FakeI2cBus`: one device at one
address, reads answered from a queue of scripted responses, every transfer
recorded, and failures on demand — all of them, or the n-th. It allocates; it is for
host tests, not firmware.

```cpp
hwlib::drivers::testing::FakeI2cBus bus{0x44};
bus.QueueResponse({0x66, 0x66, 0x93, 0x80, 0x00, 0xA2});
// ... drive the sensor ...
EXPECT_EQ(bus.Transfers().front().written, (std::vector<std::uint8_t>{0xFD}));
```

## Develop it

```bash
git submodule update --init          # ci-shared, needed by pre-commit
cmake -S . -B build && cmake --build build -j && ctest --test-dir build
```
