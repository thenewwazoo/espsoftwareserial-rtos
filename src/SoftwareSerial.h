/*
SoftwareSerial.h - Implementation of the Arduino software serial for ESP8266/ESP32.
Copyright (c) 2015-2016 Peter Lerup. All rights reserved.
Copyright (c) 2018-2019 Dirk O. Kaar. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __SoftwareSerial_h
#define __SoftwareSerial_h

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "impl.h"

#include "circular_queue/circular_queue.h"

#define isFlashInterfacePin(p) ((p) >= 6 && (p) <= 11)

// Define lets bittiming calculation be based on cpu cycles instead
// microseconds. This has higher resolution and general precision under
// low-load conditions, but whenever the CPU frequency gets switched,
// like during WiFi operation, it in turn is much more imprecise.
// TODO: increase average timing precision in CCY mode by computing
// bit durations in 10ths of micros.
#undef CCY_TICKS

namespace EspSoftwareSerial {

// Interface definition for template argument of BasicUART
class IGpioCapabilities {
public:
    static constexpr bool isValidPin(gpio_num_t pin);
    static constexpr bool isValidInputPin(gpio_num_t pin);
    static constexpr bool isValidOutputPin(gpio_num_t pin);
    // result is only defined for a valid Rx pin
    static constexpr bool hasPullUp(gpio_num_t pin);
};

class GpioCapabilities : private IGpioCapabilities {
public:
    static constexpr bool isValidPin(gpio_num_t pin) {
        return (pin >= 0 && pin <= 16) && !isFlashInterfacePin(pin);
    }

    static constexpr bool isValidInputPin(gpio_num_t pin) {
        return isValidPin(pin) && (pin != 16);
    }

    static constexpr bool isValidOutputPin(gpio_num_t pin) {
        return isValidPin(pin)
            ;
    }

    // result is only defined for a valid Rx pin
    static constexpr bool hasPullUp(gpio_num_t pin) {
        return true;
    }
};

enum Parity : uint8_t {
    PARITY_NONE = 000,
    PARITY_EVEN = 020,
    PARITY_ODD = 030,
    PARITY_MARK = 040,
    PARITY_SPACE = 070,
};

enum Config {
    SWSERIAL_5N1 = PARITY_NONE,
    SWSERIAL_6N1,
    SWSERIAL_7N1,
    SWSERIAL_8N1,
    SWSERIAL_5E1 = PARITY_EVEN,
    SWSERIAL_6E1,
    SWSERIAL_7E1,
    SWSERIAL_8E1,
    SWSERIAL_5O1 = PARITY_ODD,
    SWSERIAL_6O1,
    SWSERIAL_7O1,
    SWSERIAL_8O1,
    SWSERIAL_5M1 = PARITY_MARK,
    SWSERIAL_6M1,
    SWSERIAL_7M1,
    SWSERIAL_8M1,
    SWSERIAL_5S1 = PARITY_SPACE,
    SWSERIAL_6S1,
    SWSERIAL_7S1,
    SWSERIAL_8S1,
    SWSERIAL_5N2 = 0200 | PARITY_NONE,
    SWSERIAL_6N2,
    SWSERIAL_7N2,
    SWSERIAL_8N2,
    SWSERIAL_5E2 = 0200 | PARITY_EVEN,
    SWSERIAL_6E2,
    SWSERIAL_7E2,
    SWSERIAL_8E2,
    SWSERIAL_5O2 = 0200 | PARITY_ODD,
    SWSERIAL_6O2,
    SWSERIAL_7O2,
    SWSERIAL_8O2,
    SWSERIAL_5M2 = 0200 | PARITY_MARK,
    SWSERIAL_6M2,
    SWSERIAL_7M2,
    SWSERIAL_8M2,
    SWSERIAL_5S2 = 0200 | PARITY_SPACE,
    SWSERIAL_6S2,
    SWSERIAL_7S2,
    SWSERIAL_8S2,
};

/// This class is compatible with the corresponding AVR one, however,
/// the constructor takes no arguments, for compatibility with the
/// HardwareSerial class.
/// Instead, the begin() function handles pin assignments and logic inversion.
/// It also has optional input buffer capacity arguments for byte buffer and ISR bit buffer.
/// Bitrates up to at least 115200 can be used.
class UARTBase {
public:
    /// Ctor to set defaults for pins.
    /// @param rxPin the GPIO pin used for RX
    /// @param txPin GPIO pin used for TX
    UARTBase(gpio_num_t rxPin, gpio_num_t txPin, bool invert = false);
    UARTBase(const UARTBase&) = delete;
    UARTBase& operator= (const UARTBase&) = delete;
    virtual ~UARTBase();
    /// Configure the UARTBase object for use.
    /// @param baud the TX/RX bitrate
    /// @param config sets databits, parity, and stop bit count
    /// @param rxPin -1 or default: either no RX pin, or keeps the rxPin set in the ctor
    /// @param txPin -1 or default: either no TX pin (onewire), or keeps the txPin set in the ctor
    /// @param invert true: uses invert line level logic
    /// @param bufCapacity the capacity for the received bytes buffer
    /// @param isrBufCapacity 0: derived from bufCapacity. The capacity of the internal asynchronous
    ///        bit receive buffer, a suggested size is bufCapacity times the sum of
    ///        start, data, parity and stop bit count.
    void begin(uint32_t baud, Config config,
        gpio_num_t rxPin, gpio_num_t txPin, bool invert);

    uint32_t baudRate();
    /// Transmit control pin.
    void setTransmitEnablePin(gpio_num_t txEnablePin);
    /// Enable (default) or disable interrupts during tx.
    void enableIntTx(bool on);
    /// Enable (default) or disable internal rx GPIO pull-up.
    void enableRxGPIOPullUp(bool on);
    /// Enable or disable (default) tx GPIO output mode.
    void enableTxGPIOOpenDrain(bool on);

    bool overflow();

    int available();
    int availableForWrite() {
        if (!m_txValid) return 0;
        return 1;
    }
    int peek();
    int read();
    /// @returns The verbatim parity bit associated with the last successful read() or peek() call
    bool readParity()
    {
        return m_lastReadParity;
    }
    /// @returns The calculated bit for even parity of the parameter byte
    static bool parityEven(uint8_t byte) {
        byte ^= byte >> 4;
        byte &= 0xf;
        return (0x6996 >> byte) & 1;
    }
    /// @returns The calculated bit for odd parity of the parameter byte
    static bool parityOdd(uint8_t byte) {
        byte ^= byte >> 4;
        byte &= 0xf;
        return (0x9669 >> byte) & 1;
    }
    /// The read(buffer, size) functions are non-blocking, the same as readBytes but without timeout
    int read(uint8_t* buffer, size_t size)
        ;
    /// The read(buffer, size) functions are non-blocking, the same as readBytes but without timeout
    int read(char* buffer, size_t size) {
        return read(reinterpret_cast<uint8_t*>(buffer), size);
    }
    /// @returns The number of bytes read into buffer, up to size. Times out if the limit set through
    ///          Stream::setTimeout() is reached.
    size_t readBytes(uint8_t* buffer, size_t size);
    /// @returns The number of bytes read into buffer, up to size. Times out if the limit set through
    ///          Stream::setTimeout() is reached.
    size_t readBytes(char* buffer, size_t size) {
        return readBytes(reinterpret_cast<uint8_t*>(buffer), size);
    }
    void flush();
    size_t write(uint8_t byte);
    size_t write(uint8_t byte, Parity parity);
    size_t write(const uint8_t* buffer, size_t size);
    size_t write(const char* buffer, size_t size) {
        return write(reinterpret_cast<const uint8_t*>(buffer), size);
    }
    size_t write(const uint8_t* buffer, size_t size, Parity parity);
    size_t write(const char* buffer, size_t size, Parity parity) {
        return write(reinterpret_cast<const uint8_t*>(buffer), size, parity);
    }

    /// Disable or enable interrupts on the rx pin.
    void enableRx(bool on);
    /// One wire control.
    void enableTx(bool on);

    // AVR compatibility methods.
    bool listen() { enableRx(true); return true; }
    void end();
    bool isListening() { return m_rxEnabled; }
    bool stopListening() { enableRx(false); return true; }

    /// onReceive sets a callback that will be called in interrupt context
    /// when data is received.
    /// More precisely, the callback is triggered when UARTBase detects
    /// a new reception, which may not yet have completed on invocation.
    /// Reading - never from this interrupt context - should therefore be
    /// delayed at least for the duration of one incoming word.
    void onReceive(const Delegate<void(), void*>& handler);
    /// onReceive sets a callback that will be called in interrupt context
    /// when data is received.
    /// More precisely, the callback is triggered when UARTBase detects
    /// a new reception, which may not yet have completed on invocation.
    /// Reading - never from this interrupt context - should therefore be
    /// delayed at least for the duration of one incoming word.
    void onReceive(Delegate<void(), void*>&& handler);

    [[deprecated("function removed; semantics of onReceive() changed; check the header file.")]]
    void perform_work();

protected:
    void beginRx(bool hasPullUp, int bufCapacity, int isrBufCapacity);
    void beginTx();
    // Member variables
    gpio_num_t m_rxPin;
    gpio_num_t m_txPin;
    bool m_invert = false;

private:
    // It's legal to exceed the deadline, for instance,
    // by enabling interrupts.
    void lazyDelay();
    // Synchronous precise delay
    void preciseDelay();
    // If withStopBit is set, either cycle contains a stop bit.
    // If dutyCycle == 0, the level is not forced to HIGH.
    // If offCycle == 0, the level remains unchanged from dutyCycle.
    void writePeriod(
        uint32_t dutyCycle, uint32_t offCycle, bool withStopBit);
    // safely set the pin mode for the Rx GPIO pin
    void setRxGPIOPinMode();
    // safely set the pin mode for the Tx GPIO pin
    void setTxGPIOPinMode();
    /* check m_rxValid that calling is safe */
    void rxBits();
    void rxBits(const uint32_t isrTick);
    static void disableInterrupts();
    static void restoreInterrupts();

    static void rxBitISR(UARTBase* self);
    static void rxBitSyncISR(UARTBase* self);

    static inline uint32_t IRAM_ATTR ticks() ALWAYS_INLINE_ATTR {
        return esp_get_cycle_count() << 1;
    }
    static inline uint32_t IRAM_ATTR microsToTicks(uint32_t micros) ALWAYS_INLINE_ATTR {
        return (esp_get_cpu_freq_mhz() * micros) << 1;
    }
    static inline uint32_t ticksToMicros(uint32_t ticks) ALWAYS_INLINE_ATTR {
        return (ticks >> 1) / esp_get_cpu_freq_mhz();
    }

    // Member variables
    volatile uint32_t* m_rxReg;
    uint32_t m_rxBitMask;
    uint32_t m_txBitMask;
    gpio_num_t m_txEnablePin;
    uint8_t m_dataBits;
    bool m_oneWire;
    bool m_rxValid = false;
    bool m_rxEnabled = false;
    bool m_txValid = false;
    bool m_txEnableValid = false;
    /// PDU bits include data, parity and stop bits; the start bit is not counted.
    uint8_t m_pduBits;
    bool m_intTxEnabled;
    bool m_rxGPIOHasPullUp = false;
    bool m_rxGPIOPullUpEnabled = true;
    bool m_txGPIOOpenDrain = false;
    Parity m_parityMode;
    uint8_t m_stopBits;
    bool m_lastReadParity;
    bool m_overflow = false;
    uint32_t m_bitTicks;
    uint8_t m_parityInPos;
    uint8_t m_parityOutPos;
    int8_t m_rxLastBit; // 0 thru (m_pduBits - m_stopBits - 1): data/parity bits. -1: start bit. (m_pduBits - 1): stop bit.
    uint8_t m_rxCurByte = 0;
    std::unique_ptr<circular_queue<uint8_t> > m_buffer;
    std::unique_ptr<circular_queue<uint8_t> > m_parityBuffer;
    uint32_t m_periodStart;
    uint32_t m_periodDuration;
    // the ISR stores the relative bit times in the buffer. The inversion corrected level is used as sign bit (2's complement):
    // 1 = positive including 0, 0 = negative.
    std::unique_ptr<circular_queue<uint32_t, UARTBase*> > m_isrBuffer;
    const Delegate<void(uint32_t&&), UARTBase*> m_isrBufferForEachDel { [](UARTBase* self, uint32_t&& isrTick) { self->rxBits(isrTick); }, this };
    std::atomic<bool> m_isrOverflow { false };
    uint32_t m_isrLastTick;
    bool m_rxCurParity = false;
    Delegate<void(), void*> m_rxHandler;
};

template< class GpioCapabilities > class BasicUART : public UARTBase {
    static_assert(std::is_base_of<IGpioCapabilities, GpioCapabilities>::value,
        "template argument is not derived from IGpioCapabilities");
public:
    BasicUART(gpio_num_t rxPin, gpio_num_t txPin, bool invert = false) :
        UARTBase(rxPin, txPin, invert) {
    }

    /// Configure the BasicUART object for use.
    /// @param baud the TX/RX bitrate
    /// @param config sets databits, parity, and stop bit count
    /// @param rxPin
    /// @param txPin
    /// @param invert true: uses invert line level logic
    /// @param bufCapacity the capacity for the received bytes buffer
    /// @param isrBufCapacity 0: derived from bufCapacity. The capacity of the internal asynchronous
    ///        bit receive buffer, a suggested size is bufCapacity times the sum of
    ///        start, data, parity and stop bit count.
    void begin(uint32_t baud, Config config,
        gpio_num_t rxPin, gpio_num_t txPin, bool invert,
        int bufCapacity = 64, int isrBufCapacity = 0) {
        UARTBase::begin(baud, config, rxPin, txPin, invert);
        if (GpioCapabilities::isValidInputPin(rxPin)) {
            beginRx(GpioCapabilities:: hasPullUp(rxPin), bufCapacity, isrBufCapacity);
        }
        if (GpioCapabilities::isValidOutputPin(txPin)) {
            beginTx();
        }
        enableRx(true);
    }

    void begin(uint32_t baud, Config config,
        gpio_num_t rxPin, gpio_num_t txPin) {
        begin(baud, config, rxPin, txPin, m_invert);
    }

    void begin(uint32_t baud, Config config,
        gpio_num_t rxPin) {
        begin(baud, config, rxPin, m_txPin, m_invert);
    }

    void begin(uint32_t baud, Config config = SWSERIAL_8N1) {
        begin(baud, config, m_rxPin, m_txPin, m_invert);
    }

    void setTransmitEnablePin(gpio_num_t txEnablePin) {
        if (GpioCapabilities::isValidOutputPin(txEnablePin)) {
            UARTBase::setTransmitEnablePin(txEnablePin);
        }
    }

};

using UART = BasicUART< GpioCapabilities >;

}; // namespace EspSoftwareSerial

using SoftwareSerial = EspSoftwareSerial::UART;
using namespace EspSoftwareSerial;

#if __GNUC__ < 12
// The template member functions below must be in IRAM, but due to a bug GCC doesn't currently
// honor the attribute. Instead, it is possible to do explicit specialization and adorn
// these with the IRAM attribute:
// Delegate<>::operator (), circular_queue<>::available,
// circular_queue<>::available_for_push, circular_queue<>::push_peek, circular_queue<>::push

extern template void delegate::detail::DelegateImpl<void*, void>::operator()() const;
extern template size_t circular_queue<uint32_t, EspSoftwareSerial::UARTBase*>::available() const;
extern template bool circular_queue<uint32_t, EspSoftwareSerial::UARTBase*>::push(uint32_t&&);
extern template bool circular_queue<uint32_t, EspSoftwareSerial::UARTBase*>::push(const uint32_t&);
#endif // __GNUC__ < 12

#endif // __SoftwareSerial_h

