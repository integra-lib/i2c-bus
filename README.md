# i2c-bus

What hwlib's I2C drivers need from a bus, as C++20 concepts — and a fake bus to
test them with. Header-only; no platform code, no allocation outside the fake.

Part of hwlib. A driver takes the bus as a template parameter and requires only
the transfers it makes; the application adapts its own HAL in a few lines.

## The concepts

`address` is the 7-bit address. Each call returns whether the device acknowledged;
timeouts, retries and bus recovery belong to the bus — for the last, see
[Clearing a stuck bus](#clearing-a-stuck-bus).

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

## Clearing a stuck bus

A device left mid-byte — the MCU reset in the middle of a read — holds SDA low,
waiting for clocks that never come, and every later transfer fails until its power
is cycled. `hwlib/drivers/i2c_bus_recovery.hpp` has the bus clear of UM10204
(3.1.16): up to nine clocks on SCL, each ending in an attempt at a STOP, so the
STOP lands in the clock where the device lets go, as Linux's
`i2c_generic_scl_recovery()` does.

```cpp
#include <hwlib/drivers/i2c_bus_recovery.hpp>

// SCL and SDA as open-drain GPIOs with their pull-ups; 0 pulls low, 1 releases.
struct I2cPins
{
    gpio_dt_spec scl;
    gpio_dt_spec sda;
    void PullSclLow() { gpio_pin_set_raw(scl.port, scl.pin, 0); }
    void ReleaseScl() { gpio_pin_set_raw(scl.port, scl.pin, 1); }
    void PullSdaLow() { gpio_pin_set_raw(sda.port, sda.pin, 0); }
    void ReleaseSda() { gpio_pin_set_raw(sda.port, sda.pin, 1); }
    bool Scl() { return gpio_pin_get_raw(scl.port, scl.pin) == 1; }
    bool Sda() { return gpio_pin_get_raw(sda.port, sda.pin) == 1; }
};

// With the I2C peripheral off and the pins switched to GPIO:
const auto result = hwlib::drivers::RecoverI2cBus(g_pins, [] { k_busy_wait(5); });
```

| Result | Means |
|---|---|
| `eFree` | SDA was not held; one clock and a STOP went out all the same |
| `eRecovered` | SDA was held and was let go |
| `eSdaHeld` | SDA is still low after nine clocks: a short, or a device that is not merely stuck mid-byte |
| `eSclHeld` | SCL is low, before or during the clocks: nothing can be clocked |

The only waiting is the callable, half a clock period between every change of a
line — 5 µs or more for standard mode — from a busy wait or a task, as the caller
chooses. Both lines are released first, whatever the adapter left them at, and
on return. Handing the pins from the peripheral to GPIO and back is the
adapter's; on Zephyr, `i2c_recover_bus()` does the whole job where the controller
driver implements it.

Tested against a simulated device stuck as a transmitter on every bit of a byte
— including a 1 that makes the bus look free, and the ACK slot — as a receiver
holding its ACK, and against shorts on either line; not yet on hardware.

### Coming from a159

a159-bpu-firmware's `i2cCtrl::SwBusReset()` did the same job with PIC32 registers
and `vTaskDelay(1)`, and **reported success when SDA never let go**: its loop
`while (!SDA_Get() && (i++ < TIMEOUT))` leaves `i` at 17 on a timeout, and the
function returned `i != TIMEOUT` with `TIMEOUT` 16. And it **sent no STOP**. Its
helpers are named the other way round — `SCL_Set()` makes the pin an output, and
LATB holds 0 there, so it pulls SCL low — and read that way its closing "STOP"
keeps SCL low while SDA is let go and pulled low again. Both lines rise only when
TRISB is restored, in one write, so whether a STOP results is down to which pin
settles first. Here the STOP is sent, in the very clock the device lets go.

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
