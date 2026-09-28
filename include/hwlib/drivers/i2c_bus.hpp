#pragma once

#include <concepts>
#include <cstdint>
#include <span>

namespace hwlib::drivers
{

/// What hwlib's I2C drivers need from a bus, one transfer at a time. `address` is
/// the 7-bit address; each returns whether the device acknowledged. Timeouts,
/// retries and bus recovery belong to the bus, not to the drivers.
///
/// A driver requires only the transfers it makes, so a HAL without a combined
/// transfer can still drive a sensor that never needs one.

/// A write: START, address + W, the bytes, STOP.
template<typename Bus>
concept I2cWrite = requires(Bus& bus, std::uint8_t address, std::span<const std::uint8_t> tx) {
    {
        bus.Write(address, tx)
    } -> std::same_as<bool>;
};

/// A read: START, address + R, the bytes, STOP.
template<typename Bus>
concept I2cRead = requires(Bus& bus, std::uint8_t address, std::span<std::uint8_t> rx) {
    {
        bus.Read(address, rx)
    } -> std::same_as<bool>;
};

/// A write followed by a read with a repeated start, as one transaction: how a
/// register is read.
template<typename Bus>
concept I2cWriteRead =
    requires(Bus& bus, std::uint8_t address, std::span<const std::uint8_t> tx, std::span<std::uint8_t> rx) {
        {
            bus.WriteRead(address, tx, rx)
        } -> std::same_as<bool>;
    };

/// A register-based device's bus: writes, and register reads.
template<typename Bus>
concept I2cBus = I2cWrite<Bus> && I2cWriteRead<Bus>;

} // namespace hwlib::drivers
