#pragma once

#include <concepts>
#include <cstdint>

namespace hwlib::drivers
{

/// SCL and SDA as open-drain GPIOs, taken from the I2C peripheral for the
/// recovery: each line is either pulled low or released to its pull-up, and
/// reads what is on the wire.
template<typename Pins>
concept I2cRecoveryPins = requires(Pins& pins) {
    {
        pins.PullSclLow()
    } -> std::same_as<void>;
    {
        pins.ReleaseScl()
    } -> std::same_as<void>;
    {
        pins.PullSdaLow()
    } -> std::same_as<void>;
    {
        pins.ReleaseSda()
    } -> std::same_as<void>;
    {
        pins.Scl()
    } -> std::same_as<bool>;
    {
        pins.Sda()
    } -> std::same_as<bool>;
};

enum class I2cRecovery : std::uint8_t
{
    eFree,      ///< SDA was not held; one clock and a STOP were sent all the same
    eRecovered, ///< SDA was held low, and was let go after clocking and a STOP
    eSdaHeld,   ///< SDA stays low after nine clocks: a short, or a device that is not stuck mid-byte
    eSclHeld,   ///< SCL stays low: nothing can be clocked
};

/// The I2C bus clear (UM10204, 3.1.16): a device left mid-byte by a reset of the
/// master holds SDA low, waiting for clocks that never come, and the bus looks
/// busy for good. Up to nine clocks on SCL let it finish its byte and let go of
/// SDA, and a STOP in that same clock resets every device's interface — as
/// Linux's i2c_generic_scl_recovery() does it.
///
/// `halfPeriod` waits half a clock period, and the only waiting here: 5 µs or
/// more keeps within standard mode (100 kHz). It runs between every change of a
/// line, from a task or from a busy wait, as the caller chooses; nothing in here
/// sleeps or counts time.
///
/// Both lines are released first, and on return; after a STOP the last wait is
/// the bus free time before the next START. Taking the pins from the I2C
/// peripheral and handing them back is the caller's, as is what to do with
/// eSdaHeld or eSclHeld.
template<I2cRecoveryPins Pins, std::invocable HalfPeriod>
[[nodiscard]] I2cRecovery RecoverI2cBus(Pins& pins, HalfPeriod&& halfPeriod)
{
    constexpr std::uint8_t MAX_CLOCKS = 9U;

    // Whatever the adapter left the pins at: SDA first, so that SCL rising cannot
    // turn it into a START.
    pins.ReleaseSda();
    halfPeriod();
    pins.ReleaseScl();
    halfPeriod();
    if (!pins.Scl())
    {
        return I2cRecovery::eSclHeld;
    }

    // Every clock ends in an attempt at a STOP: SDA pulled low with SCL low, and
    // let go while SCL is high. While the device holds SDA nothing happens; in the
    // clock where it lets go, SDA rises with SCL high — the STOP — and no further
    // falling edge moves the device on to a bit that could be a 0 again.
    const bool held = !pins.Sda();
    for (std::uint8_t clock = 0U; clock < MAX_CLOCKS; ++clock)
    {
        pins.PullSclLow();
        pins.PullSdaLow();
        halfPeriod();
        pins.ReleaseScl();
        halfPeriod();
        if (!pins.Scl())
        {
            pins.ReleaseSda();
            return I2cRecovery::eSclHeld; // stretched past a half period, or shorted
        }
        pins.ReleaseSda();
        halfPeriod();
        if (pins.Sda())
        {
            return held ? I2cRecovery::eRecovered : I2cRecovery::eFree;
        }
    }
    return I2cRecovery::eSdaHeld;
}

} // namespace hwlib::drivers
