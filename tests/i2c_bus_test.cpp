#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <hwlib/drivers/i2c_bus.hpp>
#include <hwlib/drivers/testing/fake_i2c_bus.hpp>
#include <span>
#include <vector>

namespace
{

using hwlib::drivers::I2cBus;
using hwlib::drivers::I2cRead;
using hwlib::drivers::I2cWrite;
using hwlib::drivers::I2cWriteRead;
using hwlib::drivers::testing::FakeI2cBus;
using Bytes = std::vector<std::uint8_t>;

constexpr std::uint8_t ADDRESS = 0x44U;

struct WriteOnly
{
    bool Write(std::uint8_t, std::span<const std::uint8_t>);
};

struct ReadOnly
{
    bool Read(std::uint8_t, std::span<std::uint8_t>);
};

struct RegisterBus
{
    bool Write(std::uint8_t, std::span<const std::uint8_t>);
    bool WriteRead(std::uint8_t, std::span<const std::uint8_t>, std::span<std::uint8_t>);
};

struct ReturnsInt
{
    int Write(std::uint8_t, std::span<const std::uint8_t>);
};

static_assert(I2cWrite<WriteOnly> && !I2cRead<WriteOnly> && !I2cBus<WriteOnly>);
static_assert(I2cRead<ReadOnly> && !I2cWrite<ReadOnly>);
static_assert(I2cBus<RegisterBus> && !I2cRead<RegisterBus>);
// A Zephyr-style int status is not a bool: the adapter says what success is.
static_assert(!I2cWrite<ReturnsInt>);
static_assert(!I2cWrite<int>);

TEST(FakeI2cBus, RecordsEveryTransfer)
{
    FakeI2cBus bus{ADDRESS};
    bus.QueueResponse({0x01, 0x02});
    bus.QueueResponse({0x03});
    const std::array command{std::uint8_t{0xFD}};
    std::array<std::uint8_t, 2> two{};
    std::array<std::uint8_t, 1> one{};
    ASSERT_TRUE(bus.Write(ADDRESS, command));
    ASSERT_TRUE(bus.Read(ADDRESS, two));
    ASSERT_TRUE(bus.WriteRead(ADDRESS, command, one));
    EXPECT_EQ(two, (std::array<std::uint8_t, 2>{0x01, 0x02}));
    EXPECT_EQ(one, (std::array<std::uint8_t, 1>{0x03}));
    EXPECT_EQ(bus.Transfers(),
              (std::vector<FakeI2cBus::Transfer>{
                  {FakeI2cBus::Kind::eWrite,     ADDRESS, Bytes{0xFD}, 0U},
                  {FakeI2cBus::Kind::eRead,      ADDRESS, Bytes{},     2U},
                  {FakeI2cBus::Kind::eWriteRead, ADDRESS, Bytes{0xFD}, 1U},
    }));
}

TEST(FakeI2cBus, AnotherAddressIsNotAcknowledged)
{
    FakeI2cBus bus{ADDRESS};
    const std::array command{std::uint8_t{0x00}};
    EXPECT_FALSE(bus.Write(0x45U, command));
    EXPECT_EQ(bus.Transfers().size(), 1U);
}

TEST(FakeI2cBus, AReadWithNothingQueuedOrTheWrongSizeFails)
{
    FakeI2cBus bus{ADDRESS};
    std::array<std::uint8_t, 2> rx{};
    EXPECT_FALSE(bus.Read(ADDRESS, rx));
    bus.QueueResponse({0x01});
    EXPECT_FALSE(bus.Read(ADDRESS, rx));
}

TEST(FakeI2cBus, FailsAllOrOneTransfer)
{
    FakeI2cBus bus{ADDRESS};
    const std::array command{std::uint8_t{0x00}};
    bus.FailTransfer(1U);
    EXPECT_TRUE(bus.Write(ADDRESS, command));
    EXPECT_FALSE(bus.Write(ADDRESS, command));
    EXPECT_TRUE(bus.Write(ADDRESS, command));
    bus.FailAll(true);
    EXPECT_FALSE(bus.Write(ADDRESS, command));
}

} // namespace
