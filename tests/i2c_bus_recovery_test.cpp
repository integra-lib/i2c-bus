#include "fake_i2c_lines.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <hwlib/drivers/i2c_bus_recovery.hpp>
#include <initializer_list>

namespace
{

using hwlib::drivers::I2cRecovery;
using hwlib::drivers::I2cRecoveryPins;
using hwlib::drivers::RecoverI2cBus;
using hwlib::drivers::testing::FakeI2cLines;

struct NoRead
{
    void PullSclLow();
    void ReleaseScl();
    void PullSdaLow();
    void ReleaseSda();
    bool Scl();
};

static_assert(I2cRecoveryPins<FakeI2cLines>);
static_assert(!I2cRecoveryPins<NoRead>);

I2cRecovery Recover(FakeI2cLines& lines)
{
    return RecoverI2cBus(lines, [&lines] { lines.Wait(); });
}

/// What every recovery must leave: both lines let go, no START, every change
/// after a wait.
void ExpectClean(const FakeI2cLines& lines)
{
    EXPECT_TRUE(lines.MasterReleased());
    EXPECT_EQ(lines.starts, 0U);
    EXPECT_EQ(lines.violations, 0U);
}

/// A recovery that ends in a STOP also waits the bus free time after it.
void ExpectFree(const FakeI2cLines& lines)
{
    ExpectClean(lines);
    EXPECT_TRUE(lines.Settled());
    EXPECT_TRUE(lines.Scl());
    EXPECT_TRUE(lines.Sda());
}

TEST(RecoverI2cBus, AFreeBusGetsOneClockAndAStop)
{
    FakeI2cLines lines;
    EXPECT_EQ(Recover(lines), I2cRecovery::eFree);
    EXPECT_EQ(lines.clocks, 1U);
    EXPECT_EQ(lines.stops, 1U);
    ExpectFree(lines);
}

TEST(RecoverI2cBus, ATransmitterStuckOnAnyBitOfAZeroByteLetsGo)
{
    // 0x00 from bit 7 to bit 0, and the ACK slot: SDA low at the start but in the
    // ACK slot, where the device waits for the master.
    for (std::uint8_t bit = 0U; bit <= 8U; ++bit)
    {
        FakeI2cLines lines;
        lines.transmitting       = true;
        lines.byte               = 0x00U;
        lines.bit                = bit;
        const I2cRecovery result = Recover(lines);
        EXPECT_EQ(result, bit == 8U ? I2cRecovery::eFree : I2cRecovery::eRecovered) << static_cast<int>(bit);
        EXPECT_FALSE(lines.Stuck()) << static_cast<int>(bit);
        EXPECT_EQ(lines.stops, 1U) << static_cast<int>(bit);
        EXPECT_LE(lines.clocks, 9U) << static_cast<int>(bit);
        ExpectFree(lines);
    }
}

TEST(RecoverI2cBus, ATransmitterStuckOnAOneStillGetsItsStop)
{
    // A 1 on SDA: the bus looks free, but the device is mid-byte, and the first
    // clock may bring a 0. Every bit position of a mixed byte.
    for (const std::uint8_t byte : std::initializer_list<std::uint8_t>{0xA5U, 0x5AU, 0xFEU, 0x7FU, 0xFFU})
    {
        for (std::uint8_t bit = 0U; bit < 8U; ++bit)
        {
            FakeI2cLines lines;
            lines.transmitting       = true;
            lines.byte               = byte;
            lines.bit                = bit;
            const bool held          = !lines.Sda();
            const I2cRecovery result = Recover(lines);
            EXPECT_EQ(result, held ? I2cRecovery::eRecovered : I2cRecovery::eFree)
                << static_cast<int>(byte) << " " << static_cast<int>(bit);
            EXPECT_FALSE(lines.Stuck()) << static_cast<int>(byte) << " " << static_cast<int>(bit);
            EXPECT_EQ(lines.stops, 1U);
            EXPECT_LE(lines.clocks, 9U);
            ExpectFree(lines);
        }
    }
}

TEST(RecoverI2cBus, AReceiverHoldingItsAckLetsGoInOneClock)
{
    FakeI2cLines lines;
    lines.ackingReceiver = true;
    EXPECT_EQ(Recover(lines), I2cRecovery::eRecovered);
    EXPECT_EQ(lines.clocks, 1U);
    EXPECT_EQ(lines.stops, 1U);
    EXPECT_FALSE(lines.Stuck());
    ExpectFree(lines);
}

TEST(RecoverI2cBus, LinesLeftLowByTheAdapterAreReleasedFirst)
{
    FakeI2cLines lines;
    lines.PullSclLow();
    lines.Wait();
    lines.PullSdaLow();
    lines.Wait();
    lines.clocks = 0U;
    EXPECT_EQ(Recover(lines), I2cRecovery::eFree);
    EXPECT_EQ(lines.stops, 1U);
    ExpectFree(lines);
}

TEST(RecoverI2cBus, AStopFromTheAdaptersStateStillWaitsItsSetupTime)
{
    // SCL high and SDA low, just set: releasing SDA at once would be a STOP
    // without tSU;STO.
    FakeI2cLines lines;
    lines.PullSdaLow();
    lines.starts = 0U; // the test's own, not the recovery's
    EXPECT_EQ(Recover(lines), I2cRecovery::eFree);
    EXPECT_EQ(lines.stops, 2U); // the release, then the clock's own
    ExpectFree(lines);
}

TEST(RecoverI2cBus, AShortedSdaIsReportedAfterNineClocks)
{
    // a159's SwBusReset() reported success in this case.
    FakeI2cLines lines;
    lines.sdaShorted = true;
    EXPECT_EQ(Recover(lines), I2cRecovery::eSdaHeld);
    EXPECT_EQ(lines.clocks, 9U);
    EXPECT_EQ(lines.stops, 0U);
    ExpectClean(lines);
}

TEST(RecoverI2cBus, AHeldSclIsReportedWithoutClocking)
{
    FakeI2cLines lines;
    lines.sclShorted = true;
    EXPECT_EQ(Recover(lines), I2cRecovery::eSclHeld);
    EXPECT_EQ(lines.clocks, 0U);
    EXPECT_EQ(lines.waits, 3U);
    ExpectClean(lines);
}

TEST(RecoverI2cBus, SclHeldMidRecoveryIsReportedAndSdaLetGo)
{
    FakeI2cLines lines;
    lines.transmitting       = true;
    lines.byte               = 0x00U;
    lines.bit                = 7U;
    lines.sclHeldAfterClocks = 3U;
    EXPECT_EQ(Recover(lines), I2cRecovery::eSclHeld);
    EXPECT_EQ(lines.clocks, 3U);
    ExpectClean(lines);
}

} // namespace
