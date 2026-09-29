#pragma once

#include <cstddef>
#include <cstdint>

namespace hwlib::drivers::testing
{

/// SCL and SDA as open-drain wires, with a master's pins and one device on them.
///
/// The device can be left mid-byte as a transmitter — shifting out `byte` from
/// bit `bit` (7 is the MSB, 8 the ACK slot), a new bit on every falling edge of
/// SCL, and a further 0x00 byte whenever the master acknowledges — or holding
/// its own ACK low as a receiver. A START or STOP resets it, as UM10204 requires.
/// It can also be a short: SDA or SCL held low for good.
///
/// Counts what the recovery must not do: a START, and any change on the wire
/// without a half-period wait since the change before it. Lines change at once:
/// rise and fall times, and a second master, are left to the hardware checks.
struct FakeI2cLines
{
    // The device.
    bool transmitting{false};
    bool ackingReceiver{false};
    std::uint8_t byte{0x00U};
    std::uint8_t bit{7U};
    bool sdaShorted{false};
    bool sclShorted{false};
    /// SCL is held low from this falling edge on: a device stretching for good.
    std::size_t sclHeldAfterClocks{SIZE_MAX};

    // What happened.
    std::size_t clocks{0U}; ///< falling edges of SCL
    std::size_t stops{0U};
    std::size_t starts{0U};
    std::size_t violations{0U};
    std::size_t waits{0U};

    void PullSclLow()
    {
        SetMaster(m_masterScl, false);
    }

    void ReleaseScl()
    {
        SetMaster(m_masterScl, true);
    }

    void PullSdaLow()
    {
        SetMaster(m_masterSda, false);
    }

    void ReleaseSda()
    {
        SetMaster(m_masterSda, true);
    }

    [[nodiscard]] bool Scl() const
    {
        return m_masterScl && !sclShorted && clocks < sclHeldAfterClocks;
    }

    [[nodiscard]] bool Sda() const
    {
        return m_masterSda && !sdaShorted && !DeviceHoldsSda();
    }

    void Wait()
    {
        ++waits;
        m_waited = true;
    }

    [[nodiscard]] bool Stuck() const
    {
        return transmitting || ackingReceiver;
    }

    [[nodiscard]] bool MasterReleased() const
    {
        return m_masterScl && m_masterSda;
    }

    /// Whether the master waited since the last change on the wire.
    [[nodiscard]] bool Settled() const
    {
        return m_waited;
    }

private:
    bool m_masterScl{true};
    bool m_masterSda{true};
    bool m_waited{true};

    [[nodiscard]] bool DeviceHoldsSda() const
    {
        if (ackingReceiver)
        {
            return true;
        }
        return transmitting && bit < 8U && ((byte >> bit) & 1U) == 0U;
    }

    void SetMaster(bool& line, bool level)
    {
        const bool scl    = Scl();
        const bool sda    = Sda();
        line              = level;
        const bool newScl = Scl();
        const bool newSda = Sda();
        if ((scl != newScl || sda != newSda) && !m_waited)
        {
            ++violations;
        }
        if (scl != newScl || sda != newSda)
        {
            m_waited = false;
        }
        if (scl && newScl && sda != newSda)
        {
            if (newSda)
            {
                ++stops;
            }
            else
            {
                ++starts;
            }
            transmitting   = false;
            ackingReceiver = false;
            return;
        }
        if (scl && !newScl)
        {
            FallingEdge();
        }
        if (!scl && newScl)
        {
            RisingEdge(newSda);
        }
    }

    void FallingEdge()
    {
        ++clocks;
        if (ackingReceiver)
        {
            ackingReceiver = false; // its ACK clock is over
            return;
        }
        if (!transmitting)
        {
            return;
        }
        if (bit == 8U)
        {
            // The ACK clock is over: acknowledged, the next byte; else the read ends.
            transmitting = m_acked;
            byte         = 0x00U;
            bit          = 7U;
            return;
        }
        bit = bit == 0U ? std::uint8_t{8U} : static_cast<std::uint8_t>(bit - 1U);
    }

    void RisingEdge(bool sda)
    {
        if (transmitting && bit == 8U)
        {
            m_acked = !sda; // the master's ACK is sampled on the rising edge
        }
    }

    bool m_acked{false};
};

} // namespace hwlib::drivers::testing
