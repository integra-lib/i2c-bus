#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <hwlib/drivers/i2c_bus.hpp>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace hwlib::drivers::testing
{

/// A bus for driver tests, on the host: one device at one address, answering
/// reads with scripted bytes, recording every transfer, failing on request.
/// Not for firmware — it allocates.
class FakeI2cBus
{
public:
    enum class Kind : std::uint8_t
    {
        eWrite     = 0U,
        eRead      = 1U,
        eWriteRead = 2U,
    };

    struct Transfer
    {
        Kind kind{Kind::eWrite};
        std::uint8_t address{0U};
        std::vector<std::uint8_t> written;
        std::size_t readSize{0U};

        friend bool operator==(const Transfer&, const Transfer&) = default;
    };

    explicit FakeI2cBus(std::uint8_t deviceAddress) noexcept
        : m_address{deviceAddress}
    {}

    /// The next read, or the read half of the next WriteRead, gets these bytes.
    /// A read with nothing queued fails, like a device that does not answer.
    void QueueResponse(std::vector<std::uint8_t> bytes)
    {
        m_responses.push_back(std::move(bytes));
    }

    /// Every transfer fails — no device, or a stuck bus.
    void FailAll(bool fail) noexcept
    {
        m_failAll = fail;
    }

    /// Only the transfer with this index (counted from 0, all kinds) fails.
    void FailTransfer(std::size_t index) noexcept
    {
        m_failAt = index;
    }

    [[nodiscard]] const std::vector<Transfer>& Transfers() const noexcept
    {
        return m_transfers;
    }

    [[nodiscard]] bool Write(std::uint8_t address, std::span<const std::uint8_t> tx)
    {
        return Record(Kind::eWrite, address, tx, {});
    }

    [[nodiscard]] bool Read(std::uint8_t address, std::span<std::uint8_t> rx)
    {
        return Record(Kind::eRead, address, {}, rx);
    }

    [[nodiscard]] bool WriteRead(std::uint8_t address, std::span<const std::uint8_t> tx, std::span<std::uint8_t> rx)
    {
        return Record(Kind::eWriteRead, address, tx, rx);
    }

private:
    bool Record(Kind kind, std::uint8_t address, std::span<const std::uint8_t> tx, std::span<std::uint8_t> rx)
    {
        const std::size_t index = m_transfers.size();
        m_transfers.push_back(Transfer{
            kind, address, {tx.begin(), tx.end()},
              rx.size()
        });
        if (m_failAll || m_failAt == index || address != m_address)
        {
            return false;
        }
        if (kind == Kind::eWrite)
        {
            return true;
        }
        if (m_responses.empty() || m_responses.front().size() != rx.size())
        {
            return false;
        }
        const std::vector<std::uint8_t> response = std::move(m_responses.front());
        m_responses.pop_front();
        for (std::size_t i = 0U; i < rx.size(); ++i)
        {
            rx[i] = response[i];
        }
        return true;
    }

    std::uint8_t m_address;
    std::deque<std::vector<std::uint8_t>> m_responses;
    std::vector<Transfer> m_transfers;
    bool m_failAll{false};
    std::optional<std::size_t> m_failAt;
};

static_assert(I2cWrite<FakeI2cBus> && I2cRead<FakeI2cBus> && I2cWriteRead<FakeI2cBus> && I2cBus<FakeI2cBus>);

} // namespace hwlib::drivers::testing
