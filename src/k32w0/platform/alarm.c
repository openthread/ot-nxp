/*
 *  Copyright (c) 2019, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file implements the OpenThread platform abstraction for the alarm.
 *
 */

#include "MicroSpecific_arm_sdk2.h"
#include "TMR_Adapter.h"
#include "TimersManager.h"
#include "fsl_clock.h"
#include "fsl_ctimer.h"
#include "fsl_device_registers.h"
#include "fsl_os_abstraction.h"
#include "fsl_wtimer.h"
#include "openthread-system.h"
#include <common/logging.hpp>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>

#ifdef ALARM_LOG_ENABLED
#include "dbg_logging.h"
#define ALARM_LOG(fmt, ...)                                                    \
    do                                                                         \
    {                                                                          \
        DbgLogAdd(__FUNCTION__, fmt, VA_NUM_ARGS(__VA_ARGS__), ##__VA_ARGS__); \
    } while (0);
#else
#define ALARM_LOG(...)
#endif

#define ALARM_USE_CTIMER 0
#define ALARM_USE_WTIMER 1

/* Timer frequency in Hz needed for 1ms tick */
#define TARGET_FREQ 1000U
#define MAX_TIMESTAMP_VALUE_MS 0x7CFFFFF

static bool     sEventFired = false;
static uint32_t refClk;

#if ALARM_USE_CTIMER
/* Match Configuration for Channel 0 */
static ctimer_match_config_t sMatchConfig = {.enableCounterReset = false,
                                             .enableCounterStop  = false,
                                             .matchValue         = 0x00,
                                             .outControl         = kCTIMER_Output_NoAction,
                                             .outPinInitState    = false,
                                             .enableInterrupt    = true};
#else
static TMR_tsActivityWakeTimerEvent otTimer;
static void                         TMR_ScheduleActivityCallback(void);
static uint32_t                     sAcumulatedTimestamp = 0;
static uint32_t                     sLastTimestamp       = 0;
#endif

/* Stub function for notifying application of wakeup */
WEAK void App_NotifyWakeup(void);

/**
 * Stub function for notifying application of wakeup
 *
 */
WEAK void App_NotifyWakeup(void)
{
}

void K32WAlarmInit(void)
{
#if ALARM_USE_CTIMER
    ctimer_config_t config;
    CTIMER_GetDefaultConfig(&config);

    /* Get clk frequency and use prescale to lower it */
    refClk = CLOCK_GetFreq(kCLOCK_Timer0);

    config.prescale = refClk / TARGET_FREQ;
    CTIMER_Init(CTIMER0, &config);
    CTIMER_StartTimer(CTIMER0);

    CTIMER_EnableInterrupts(CTIMER0, kCTIMER_Match0InterruptEnable);
    NVIC_ClearPendingIRQ(Timer0_IRQn);
    NVIC_EnableIRQ(Timer0_IRQn);

#else

    /* Handles WTIMER init inside, Wake timer 0 is 41 bits long and is used for keepig the timestamp */
    Timestamp_Init();

    /* Get clk frequency and use prescale to lower it */
    refClk           = CLOCK_GetFreq(kCLOCK_Xtal32k);
    otTimer.u8Status = TMR_E_ACTIVITY_FREE;
#endif
}

void K32WAlarmClean(void)
{
#if ALARM_USE_CTIMER
    CTIMER_StopTimer(CTIMER0);
    CTIMER_Deinit(CTIMER0);
    CTIMER_DisableInterrupts(CTIMER0, kCTIMER_Match0InterruptEnable);
    NVIC_ClearPendingIRQ(Timer0_IRQn);
#else
    Timestamp_Deinit();
    TMR_eRemoveActivity(&otTimer);
#endif
}

void K32WAlarmProcess(otInstance *aInstance)
{
    OSA_InterruptDisable();
    if (sEventFired)
    {
        sEventFired = false;
        OSA_InterruptEnable();
#if OPENTHREAD_ENABLE_DIAG

        if (otPlatDiagModeGet())
        {
            otPlatDiagAlarmFired(aInstance);
        }
        else
#endif
        {
            otPlatAlarmMilliFired(aInstance);
        }
    }
    else
    {
        OSA_InterruptEnable();
    }
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    OT_UNUSED_VARIABLE(aInstance);

#if ALARM_USE_CTIMER
    /* Load match register with current counter + app time */
    sMatchConfig.matchValue = aT0 + aDt;

    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &sMatchConfig);
#else
    uint64_t targetTicks = 0;

    /* Calculate the difference between now and the requested timestamp aT0 - this time will be
       substracted from the total time until the event needs to fire */
    uint32_t timestamp = otPlatAlarmMilliGetNow();

    otLogInfoUtil("Start timer: timestamp:%d, aTo:%d, aDt:%d", timestamp, aT0, aDt);
    if (timestamp >= aT0)
    {
        timestamp = timestamp - aT0;
    }
    else
    {
        timestamp = (~0UL) - aT0 + timestamp + 1;
    }

    if (aDt > timestamp)
    {
        targetTicks = MILLISECONDS_TO_TICKS32K((aDt - timestamp));
    }

    if (targetTicks > 0)
    {
        TMR_eRemoveActivity(&otTimer);
        if (targetTicks > WTIMER1_MAX_VALUE)
        {
            /* Because timer 1 is only 28 bits long we need to take into account and event longer than this
            so we arm the timer with the maximum value and re-arm with the remaing time once it fires */
            targetTicks = WTIMER1_MAX_VALUE;
        }
        TMR_eScheduleActivity32kTicks(&otTimer, targetTicks, TMR_ScheduleActivityCallback);
    }
    else
    {
        sEventFired = true;
        otSysEventSignalPending();
        App_NotifyWakeup();
    }

#endif
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    sEventFired = false;

#if ALARM_USE_CTIMER
    sMatchConfig.matchValue = 0;
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &sMatchConfig);
#else
    TMR_eRemoveActivity(&otTimer);
#endif
}

uint32_t otPlatAlarmMilliGetNow(void)
{
#if ALARM_USE_CTIMER
    return CTIMER0->TC;
#else

    uint64_t tiks = Timestamp_GetCounter32bit();

    tiks *= TARGET_FREQ;
    tiks /= refClk;
    uint32_t retTimestamp = (uint32_t)tiks;

    if (sLastTimestamp > retTimestamp)
    {
        sAcumulatedTimestamp += MAX_TIMESTAMP_VALUE_MS;
    }
    sLastTimestamp = retTimestamp;

    return retTimestamp + sAcumulatedTimestamp;
#endif
}

#if ALARM_USE_CTIMER
/**
 * Timer interrupt handler function.
 *
 */
void CTIMER0_IRQHandler(void)
{
    uint32_t flags = CTIMER_GetStatusFlags(CTIMER0);
    CTIMER_ClearStatusFlags(CTIMER0, flags);
    sEventFired = true;
    otSysEventSignalPending();
}
#else

static void TMR_ScheduleActivityCallback(void)
{
    ALARM_LOG("");
    sEventFired = true;
    App_NotifyWakeup();
    otSysEventSignalPending();
}
#endif
