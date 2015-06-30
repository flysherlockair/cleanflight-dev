#include <signal.h>
#include <stdlib.h>
#include <stdbool.h>

#include "platform.h"

#include "common/utils.h"
#include "common/atomic.h"

#include "drivers/nvic.h"

#include "drivers/gpio.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "drivers/serial.h"

#include "io/serial.h"

#include "profiler.h"

#ifdef USE_PROFILER

// Must be a power of 2
#define PROFILE_BUFFER_SIZE 16
#define PROFILE_BUFFER_MASK (PROFILE_BUFFER_SIZE - 1)

#define EXC_RETURN_HANDLER_NO_FLOAT_MSP ((void*)0xFFFFFFF1)
#define EXC_RETURN_THREAD_NO_FLOAT_MSP  ((void*)0xFFFFFFF9)
#define EXC_RETURN_THREAD_NO_FLOAT_PSP  ((void*)0xFFFFFFFD)

#define EXC_RETURN_HANDLER_FLOAT_MSP ((void*)0xFFFFFFE1)
#define EXC_RETURN_THREAD_FLOAT_MSP  ((void*)0xFFFFFFE9)
#define EXC_RETURN_THREAD_FLOAT_PSP  ((void*)0xFFFFFFED)

// The ARM CPU pushes this onto the stack before it calls the IRQ handler
// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0553a/Babefdjc.html
typedef struct armExceptionStackFrame_t {
    uint32_t r0, r1, r2, r3, r12;
    void *lr, *pc;
    uint32_t xpsr;
} armExceptionStackFrame_t;

static void* profileSampleBuffer[PROFILE_BUFFER_SIZE];
static volatile sig_atomic_t profileSampleHead = 0;

static bool profilerEnabled = false;

static serialPort_t *profilerPort = NULL;

bool profilerPortOpen()
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_PROFILER);
    portOptions_t portOptions = SERIAL_PARITY_NO | SERIAL_NOT_INVERTED;

    if (!portConfig) {
        return false;
    }

    baudRate_e baudRateIndex;

    baudRateIndex = portConfig->profiler_baudrateIndex;

    if (baudRates[baudRateIndex] == 230400) {
        /*
         * OpenLog's 230400 baud rate is very inaccurate, so it requires a larger inter-character gap in
         * order to maintain synchronization.
         */
        portOptions |= SERIAL_STOPBITS_2;
    } else {
        portOptions |= SERIAL_STOPBITS_1;
    }

    profilerPort = openSerialPort(portConfig->identifier, FUNCTION_PROFILER, NULL, baudRates[baudRateIndex],
        MODE_TX, portOptions);

    return profilerPort != NULL;
}

/**
 * If the profiler is enabled, log an instruction sample.
 */
void profilerLogSample(void *returnAddress, void *stackFrame)
{
    if (!profilerEnabled) {
        return;
    }

    if (returnAddress == EXC_RETURN_THREAD_NO_FLOAT_MSP || returnAddress == EXC_RETURN_THREAD_FLOAT_MSP) {
        struct armExceptionStackFrame_t *exceptionFrame = (struct armExceptionStackFrame_t *) stackFrame;

        int oldHead = profileSampleHead;

        // Log the location that we would return to after the exception is handled (that's the code we interrupted)
        profileSampleBuffer[oldHead] = exceptionFrame->pc;

        profileSampleHead = (oldHead + 1) & PROFILE_BUFFER_MASK;
    }
}

static timerCCHandlerRec_t timerCallback;

static void doNothing(struct timerCCHandlerRec_s* self, uint16_t capture)
{
    (void) self;
    (void) capture;
}

static void profilerAllocateTimer()
{
    const timerHardware_t *timer;

    timer = &(timerHardware[PROFILER_TIMER_HARDWARE]);

    // Run timer at 1MHz and trigger every 1000 counts, so in other words sample at 1ms intervals
    configTimeBase(timer->tim, 1000, 1);
    TIM_Cmd(timer->tim, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = timer->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_PROFILER);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_PROFILER);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // We need to enable callbacks in order to enable IRQs for this timer
    timerChCCHandlerInit(&timerCallback, doNothing);
    timerChConfigCallbacks(timer, &timerCallback, NULL);
}

static void profilerDeallocateTimer()
{
// TODO
}

void profilerStart()
{
    if (profilerPortOpen()) {
        profilerAllocateTimer();

        profileSampleHead = 0;
        profilerEnabled = true;
    }
}

void profilerStop()
{
    if (profilerEnabled) {
        profilerEnabled = false;

        closeSerialPort(profilerPort);
        profilerPort = NULL;

        profilerDeallocateTimer();
    }
}

static void serialWriteU32(serialPort_t *port, uint32_t value)
{
    serialWrite(port, value & 0xFF);
    serialWrite(port, (value >> 8) & 0xFF);
    serialWrite(port, (value >> 16) & 0xFF);
    serialWrite(port, (value >> 24) & 0xFF);
}

/**
 * Call periodically so the profiler can perform tasks like writing out the profile to the serial port
 */
void profilerProcess()
{
    const int PROFILER_ENTRY_SIZE = 1 + sizeof(void*);
    int i;

    if (!profilerEnabled)
        return;

    int samplesAvailable = profileSampleHead;

    // Work out how many samples we can cram into the available Tx buffer
    int serialBudgetBytes = serialTxBytesFree(profilerPort);

    for (i = 0; i < samplesAvailable && serialBudgetBytes >= PROFILER_ENTRY_SIZE; i++) {
        serialWrite(profilerPort, '>');
        serialWriteU32(profilerPort, (uint32_t) profileSampleBuffer[i]);

        serialBudgetBytes -= PROFILER_ENTRY_SIZE;
    }

    profileSampleHead = 0;
}

#endif
