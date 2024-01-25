/*
 *  Copyright (c) 2022, The OpenThread Authors.
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

#include "EmbeddedTypes.h"
#include "fsl_clock.h"
#include "fsl_component_timer_manager.h"
#include "fsl_device_registers.h"
#include <stdint.h>

#include "PWR_Interface.h"
#include "fsl_os_abstraction.h"
#include "fwk_platform.h"
#include "openthread-system.h"
#include <common/logging.hpp>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/diag.h>

static bool_t sEventFired = FALSE;
TIMER_MANAGER_HANDLE_DEFINE(sAlarmTimerHandle);

#if OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
static bool_t sEventMicroFired = FALSE;
TIMER_MANAGER_HANDLE_DEFINE(sAlarmMicroTimerHandle);
#endif

extern void PWR_DisallowDeviceToSleep(void);
extern void PWR_AllowDeviceToSleep(void);

static void timerCallback(void *param)
{
    sEventFired = TRUE;
    PWR_DisallowDeviceToSleep();
    otSysEventSignalPending();
}

#if OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
static void timerMicroCallback(void *param)
{
    sEventMicroFired = TRUE;
    PWR_DisallowDeviceToSleep();
    otSysEventSignalPending();
}
#endif

static uint32_t timestamp_to_ms(uint64_t timestamp)
{
    return (timestamp / 1000);
}

void otPlatAlarmInit(void)
{
    (void)TM_Open((timer_handle_t)sAlarmTimerHandle);
    (void)TM_InstallCallback((timer_handle_t)sAlarmTimerHandle, (timer_callback_t)timerCallback, NULL);

#if OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
    (void)TM_Open((timer_handle_t)sAlarmMicroTimerHandle);
    (void)TM_InstallCallback((timer_handle_t)sAlarmMicroTimerHandle, (timer_callback_t)timerMicroCallback, NULL);
#endif
}

void otPlatAlarmProcess(otInstance *aInstance)
{
    if (sEventFired == TRUE)
    {
        sEventFired = FALSE;
        PWR_AllowDeviceToSleep();
        otPlatAlarmMilliFired(aInstance);
    }

#if OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
    if (sEventMicroFired == TRUE)
    {
        sEventMicroFired = FALSE;
        PWR_AllowDeviceToSleep();
        otPlatAlarmMicroFired(aInstance);
    }
#endif
}

void otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    OT_UNUSED_VARIABLE(aInstance);

    TM_Stop(sAlarmTimerHandle);

    /* Calculate the difference between now and the requested timestamp aT0 - this time will be
       substracted from the total time until the event needs to fire */
    uint32_t timestamp = otPlatAlarmMilliGetNow();

    otLogInfoPlat("Start timer: timestamp:%ld, aTo:%ld, aDt:%ld", timestamp, aT0, aDt);
    if (timestamp >= aT0)
    {
        timestamp = timestamp - aT0;
    }
    else
    {
        timestamp = (~0UL) - aT0 + timestamp + 1;
    }

    if (aDt >= timestamp)
    {
        aDt -= timestamp;
    }

    if (aDt > 0)
    {
        TM_Start(sAlarmTimerHandle, kTimerModeSingleShot, aDt);
    }
    else
    {
        timerCallback(NULL);
    }
}

void otPlatAlarmMilliStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    TM_Stop(sAlarmTimerHandle);
}

uint32_t otPlatAlarmMilliGetNow(void)
{
    uint32_t tstamp;
    tstamp = timestamp_to_ms(TM_GetTimestamp());

    return tstamp;
}

#if OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE
uint32_t otPlatAlarmMicroGetNow(void)
{
    return TM_GetTimestamp();
}

void otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t aT0, uint32_t aDt)
{
    /* Need support for us timer in SDK. Untill then we contiune to use ms timer and devide input
       time to 1000 */
    OT_UNUSED_VARIABLE(aInstance);

    TM_Stop(sAlarmMicroTimerHandle);

    /* Calculate the difference between now and the requested timestamp aT0 - this time will be
       substracted from the total time until the event needs to fire */
    uint32_t timestamp = TM_GetTimestamp();

    otLogInfoPlat("Start timer: timestamp:%ld, aTo:%ld, aDt:%ld", timestamp, aT0, aDt);
    if (timestamp >= aT0)
    {
        timestamp = timestamp - aT0;
    }
    else
    {
        timestamp = (~0UL) - aT0 + timestamp + 1;
    }

    if (aDt >= timestamp)
    {
        aDt -= timestamp;
    }

    if (aDt > 0)
    {
        TM_Start(sAlarmMicroTimerHandle, kTimerModeSingleShot | kTimerModeSetMicrosTimer, aDt);
    }
    else
    {
        timerMicroCallback(NULL);
    }
}

void otPlatAlarmMicroStop(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);
    TM_Stop(sAlarmMicroTimerHandle);
}

#endif /* OPENTHREAD_CONFIG_PLATFORM_USEC_TIMER_ENABLE */

uint64_t otPlatTimeGet(void)
{
    static uint64_t u64SwTimestampUs = 0;
    static uint64_t u64HwTimestampUs = 0;

    OSA_InterruptDisable();

    /* Get new 32bit HW timestamp */
    uint64_t u64HwTimestampUs_new = (uint64_t)TM_GetTimestamp();
    uint64_t wrapped_val          = 0;
    uint64_t increment;

    /* Check if the timestamp has wrapped around */
    if (u64HwTimestampUs > u64HwTimestampUs_new)
    {
        wrapped_val = COUNT_TO_USEC(((uint64_t)1 << 32), PLATFORM_TM_CLK_FREQ);
    }

    increment = (u64HwTimestampUs_new + wrapped_val) - u64HwTimestampUs;
    u64SwTimestampUs += increment;

    /* Store new HW timestamp for next iteration */
    u64HwTimestampUs = u64HwTimestampUs_new;

    OSA_InterruptEnable();

    return u64SwTimestampUs;
}
