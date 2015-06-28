#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <limits.h>

#include "blackbox_io.h"

#include "version.h"
#include "build_config.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/encoding.h"

#include "drivers/gpio.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/compass.h"
#include "drivers/timer.h"
#include "drivers/pwm_rx.h"
#include "drivers/accgyro.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"

#include "sensors/sensors.h"
#include "sensors/boardalignment.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"

#include "io/beeper.h"
#include "io/display.h"
#include "io/escservo.h"
#include "rx/rx.h"
#include "io/rc_controls.h"

#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_cli.h"
#include "io/serial_msp.h"
#include "io/statusindicator.h"
#include "rx/msp.h"
#include "telemetry/telemetry.h"
#include "common/printf.h"

#include "flight/mixer.h"
#include "flight/altitudehold.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

#include "io/flashfs.h"

#ifdef BLACKBOX

#define BLACKBOX_SERIAL_PORT_MODE MODE_TX

// How many bytes can we transmit per loop iteration when writing headers?
static uint8_t blackboxMaxHeaderBytesPerIteration;

// How many bytes can we write *this* iteration without overflowing transmit buffers or overstressing the OpenLog?
int32_t blackboxHeaderBudget;

static serialPort_t *blackboxPort = NULL;
static portSharing_e blackboxPortSharing;

void blackboxWrite(uint8_t value)
{
    switch (masterConfig.blackbox_device) {
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            flashfsWriteByte(value); // Write byte asynchronously
        break;
#endif
        case BLACKBOX_DEVICE_SERIAL:
        default:
            serialWrite(blackboxPort, value);
        break;
    }
}

static void _putc(void *p, char c)
{
    (void)p;
    blackboxWrite(c);
}

static int blackboxPrintfv(const char *fmt, va_list va)
{
    return tfp_format(NULL, _putc, fmt, va);
}


//printf() to the blackbox serial port with no blocking shenanigans (so it's caller's responsibility to not write too fast!)
int blackboxPrintf(const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);

    int written = blackboxPrintfv(fmt, va);

    va_end(va);

    return written;
}

/*
 * printf a Blackbox header line with a leading "H " and trailing "\n" added automatically. blackboxHeaderBudget is
 * decreased to account for the number of bytes written.
 */
void blackboxPrintfHeaderLine(const char *fmt, ...)
{
    va_list va;

    blackboxWrite('H');
    blackboxWrite(' ');

    va_start(va, fmt);

    int written = blackboxPrintfv(fmt, va);

    va_end(va);

    blackboxWrite('\n');

    blackboxHeaderBudget -= written + 3;
}

// Print the null-terminated string 's' to the blackbox device and return the number of bytes written
int blackboxPrint(const char *s)
{
    int length;
    const uint8_t *pos;

    switch (masterConfig.blackbox_device) {

#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            length = strlen(s);
            flashfsWrite((const uint8_t*) s, length, false); // Write asynchronously
        break;
#endif

        case BLACKBOX_DEVICE_SERIAL:
        default:
            pos = (uint8_t*) s;
            while (*pos) {
                serialWrite(blackboxPort, *pos);
                pos++;
            }

            length = pos - (uint8_t*) s;
        break;
    }

    return length;
}

/**
 * Write an unsigned integer to the blackbox serial port using variable byte encoding.
 */
void blackboxWriteUnsignedVB(uint32_t value)
{
    //While this isn't the final byte (we can only write 7 bits at a time)
    while (value > 127) {
        blackboxWrite((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    blackboxWrite(value);
}

/**
 * Write a signed integer to the blackbox serial port using ZigZig and variable byte encoding.
 */
void blackboxWriteSignedVB(int32_t value)
{
    //ZigZag encode to make the value always positive
    blackboxWriteUnsignedVB(zigzagEncode(value));
}

void blackboxWriteSignedVBArray(int32_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        blackboxWriteSignedVB(array[i]);
    }
}

void blackboxWriteSigned16VBArray(int16_t *array, int count)
{
    for (int i = 0; i < count; i++) {
        blackboxWriteSignedVB(array[i]);
    }
}

void blackboxWriteS16(int16_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
}

static uint8_t blackboxBitBuffer = 0;
static uint8_t blackboxBitBufferCount = 0;

#define BLACKBOX_BIT_BUFFER_CAPACITY (sizeof(blackboxBitBuffer) * CHAR_BIT)

void blackboxWriteBits(uint32_t bits, unsigned int bitCount) {
    if (bitCount == 0)
        return; // Nothing to write! (return now to avoid shifting left by 32 on the next line, which is undefined)

    // Align the bits to be written to the top of that variable:
    bits <<= sizeof(bits) * CHAR_BIT - bitCount;

    do {
        uint8_t availableCapacity = BLACKBOX_BIT_BUFFER_CAPACITY - blackboxBitBufferCount;
        uint8_t numBitsToWrite = bitCount <= availableCapacity ? bitCount : availableCapacity;

        // Align the bits to be written to the correct part of the buffer and insert them
        blackboxBitBuffer |= bits >> ((sizeof(bits) - sizeof(blackboxBitBuffer)) * CHAR_BIT + blackboxBitBufferCount);
        blackboxBitBufferCount += numBitsToWrite;

        // Did we fill the buffer? If so write the whole thing out
        if (blackboxBitBufferCount == BLACKBOX_BIT_BUFFER_CAPACITY) {
            for (int i = sizeof(blackboxBitBuffer) - 1; i >= 0; i--) {
                blackboxWrite(blackboxBitBuffer >> (CHAR_BIT * i));
            }
            blackboxBitBuffer = 0;
            blackboxBitBufferCount = 0;
        }

        bitCount -= numBitsToWrite;
        bits <<= numBitsToWrite;
    } while (bitCount > 0);
}

void blackboxFlushBits() {
    if (sizeof(blackboxBitBuffer) > 1) {
        // Round up the bits to get the number of occupied bytes
        int numBytes = (blackboxBitBufferCount + CHAR_BIT - 1) / CHAR_BIT;

        for (int i = 0; i < numBytes; i++) {
            // Write the top byte
            blackboxWrite(blackboxBitBuffer >> ((sizeof(blackboxBitBuffer) - 1) * CHAR_BIT));

            // And shift the remaining bits up to fill that space
            blackboxBitBuffer <<= CHAR_BIT;
        }

        blackboxBitBufferCount = 0;
    } else {
        if (blackboxBitBufferCount > 0) {
            blackboxWrite(blackboxBitBuffer);
            blackboxBitBuffer = 0;
            blackboxBitBufferCount = 0;
        }
    }
}

/**
 * How many bits would be required to fit the given integer? `i` must not be zero.
 */
static int numBitsToStoreInteger(uint32_t i)
{
    return sizeof(i) * CHAR_BIT - __builtin_clz(i);
}

// Value must be more than zero
static void blackboxWriteU32EliasDeltaInternal(uint32_t value)
{
    unsigned int valueLen, lengthOfValueLen;

    valueLen = numBitsToStoreInteger(value);
    lengthOfValueLen = numBitsToStoreInteger(valueLen);

    // Use unary to encode the number of bits we'll need to write the length of the `value`
    blackboxWriteBits(0, lengthOfValueLen - 1);
    // Now write the length of the `value`
    blackboxWriteBits(valueLen, lengthOfValueLen);
    // Having now encoded the position of the top bit of `value`, write its remaining bits
    blackboxWriteBits(value, valueLen - 1);
}

void blackboxWriteU32EliasDelta(uint32_t value)
{
    /* We can't encode value==0, so we need to add 1 to the value before encoding
     *
     * That would make it impossible to encode MAXINT, so use 0xFFFFFFFF as an escape code which can mean
     * either MAXINT - 1 or MAXINT
     */
    if (value < 0xFFFFFFFE) {
        blackboxWriteU32EliasDeltaInternal(value + 1);
    } else {
        // Write the escape code
        blackboxWriteU32EliasDeltaInternal(0xFFFFFFFF);
        // Add a one bit after the escape code if we wanted "MAXINT", or a zero if we wanted "MAXINT - 1"
        blackboxWriteBits(value - 0xFFFFFFFE, 1);
    }
}

void blackboxWriteS32EliasDelta(int32_t value)
{
    blackboxWriteU32EliasDelta(zigzagEncode(value));
}

/** Write unsigned integer **/
void blackboxWriteU32(int32_t value)
{
    blackboxWrite(value & 0xFF);
    blackboxWrite((value >> 8) & 0xFF);
    blackboxWrite((value >> 16) & 0xFF);
    blackboxWrite((value >> 24) & 0xFF);
}

/** Write float value in the integer form **/
void blackboxWriteFloat(float value)
{
    blackboxWriteU32(castFloatBytesToInt(value));
}

/**
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 * 
 * Returns true if all data has been flushed to the device.
 */
bool blackboxDeviceFlush(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            //Nothing to speed up flushing on serial, as serial is continuously being drained out of its buffer
            return isSerialTransmitBufferEmpty(blackboxPort);

#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            return flashfsFlushAsync();
#endif

        default:
            return false;
    }
}

/**
 * Attempt to open the logging device. Returns true if successful.
 */
bool blackboxDeviceOpen(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            {
                serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_BLACKBOX);
                baudRate_e baudRateIndex;
                portOptions_t portOptions = SERIAL_PARITY_NO | SERIAL_NOT_INVERTED;

                if (!portConfig) {
                    return false;
                }

                blackboxPortSharing = determinePortSharing(portConfig, FUNCTION_BLACKBOX);
                baudRateIndex = portConfig->blackbox_baudrateIndex;

                if (baudRates[baudRateIndex] == 230400) {
                    /*
                     * OpenLog's 230400 baud rate is very inaccurate, so it requires a larger inter-character gap in
                     * order to maintain synchronization.
                     */
                    portOptions |= SERIAL_STOPBITS_2;
                } else {
                    portOptions |= SERIAL_STOPBITS_1;
                }

                blackboxPort = openSerialPort(portConfig->identifier, FUNCTION_BLACKBOX, NULL, baudRates[baudRateIndex],
                    BLACKBOX_SERIAL_PORT_MODE, portOptions);

                /*
                 * The slowest MicroSD cards have a write latency approaching 150ms. The OpenLog's buffer is about 900
                 * bytes. In order for its buffer to be able to absorb this latency we must write slower than 6000 B/s.
                 *
                 * So:
                 *     Bytes per loop iteration = floor((looptime_ns / 1000000.0) * 6000)
                 *                              = floor((looptime_ns * 6000) / 1000000.0)
                 *                              = floor((looptime_ns * 3) / 500.0)
                 *                              = (looptime_ns * 3) / 500
                 */
                blackboxMaxHeaderBytesPerIteration = constrain((masterConfig.looptime * 3) / 500, 1, BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION);

                return blackboxPort != NULL;
            }
            break;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            if (flashfsGetSize() == 0 || isBlackboxDeviceFull()) {
                return false;
            }

            blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;

            return true;
        break;
#endif
        default:
            return false;
    }
}

/**
 * Close the Blackbox logging device immediately without attempting to flush any remaining data.
 */
void blackboxDeviceClose(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            closeSerialPort(blackboxPort);
            blackboxPort = NULL;

            /*
             * Normally this would be handled by mw.c, but since we take an unknown amount
             * of time to shut down asynchronously, we're the only ones that know when to call it.
             */
            if (blackboxPortSharing == PORTSHARING_SHARED) {
                mspAllocateSerialPorts(&masterConfig.serialConfig);
            }
            break;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            // No-op since the flash doesn't have a "close" and there's nobody else to hand control of it to.
            break;
#endif
    }
}

bool isBlackboxDeviceFull(void)
{
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            return false;

#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            return flashfsIsEOF();
#endif

        default:
            return false;
    }
}

/**
 * Call once every loop iteration in order to maintain the global blackboxHeaderBudget with the number of bytes we can
 * transmit this iteration.
 */
void blackboxReplenishHeaderBudget()
{
    int32_t freeSpace;

    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            freeSpace = serialTxBytesFree(blackboxPort);
        break;
#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            freeSpace = flashfsGetWriteBufferFreeSpace();
        break;
#endif
        default:
            freeSpace = 0;
    }

    blackboxHeaderBudget = MIN(MIN(freeSpace, blackboxHeaderBudget + blackboxMaxHeaderBytesPerIteration), BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET);
}

/**
 * You must call this function before attempting to write Blackbox header bytes to ensure that the write will not
 * cause buffers to overflow. The number of bytes you can write is capped by the blackboxHeaderBudget. Calling this
 * reservation function doesn't decrease blackboxHeaderBudget, so you must manually decrement that variable by the
 * number of bytes you actually wrote.
 *
 * When the Blackbox device is FlashFS, a successful return code guarantees that no data will be lost if you write that
 * many bytes to the device (i.e. FlashFS's buffers won't overflow).
 *
 * When the device is a serial port, a successful return code guarantees that Cleanflight's serial Tx buffer will not
 * overflow, and the outgoing bandwidth is likely to be small enough to give the OpenLog time to absorb MicroSD card
 * latency. However the OpenLog could still end up silently dropping data.
 *
 * Returns:
 *  BLACKBOX_RESERVE_SUCCESS - Upon success
 *  BLACKBOX_RESERVE_TEMPORARY_FAILURE - The buffer is currently too full to service the request, try again later
 *  BLACKBOX_RESERVE_PERMANENT_FAILURE - The buffer is too small to ever service this request
 */
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes)
{
    if (bytes <= blackboxHeaderBudget) {
        return BLACKBOX_RESERVE_SUCCESS;
    }

    // Handle failure:
    switch (masterConfig.blackbox_device) {
        case BLACKBOX_DEVICE_SERIAL:
            // One byte of the tx buffer isn't available for user data (due to its circular list implementation), hence the -1
            if (bytes > (int32_t) blackboxPort->txBufferSize - 1) {
                return BLACKBOX_RESERVE_PERMANENT_FAILURE;
            }

            return BLACKBOX_RESERVE_TEMPORARY_FAILURE;

#ifdef USE_FLASHFS
        case BLACKBOX_DEVICE_FLASH:
            if (bytes > (int32_t) flashfsGetWriteBufferSize()) {
                return BLACKBOX_RESERVE_PERMANENT_FAILURE;
            }

            if (bytes > (int32_t) flashfsGetWriteBufferFreeSpace()) {
                /*
                 * The write doesn't currently fit in the buffer, so try to make room for it. Our flushing here means
                 * that the Blackbox header writing code doesn't have to guess about the best time to ask flashfs to
                 * flush, and doesn't stall waiting for a flush that would otherwise not automatically be called.
                 */
                flashfsFlushAsync();
            }

            return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
#endif

        default:
            return BLACKBOX_RESERVE_PERMANENT_FAILURE;
    }
}

#endif
