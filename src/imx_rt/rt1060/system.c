/*
 *  Copyright (c) 2021, The OpenThread Authors.
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
 *   This file includes the platform-specific initializers.
 *
 */
#include "platform-rt1060.h"

#include <openthread/tasklet.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/time.h>

void otSysInit(int argc, char *argv[])
{
    OT_UNUSED_VARIABLE(argc);
    OT_UNUSED_VARIABLE(argv);

    rt1060ApiLockInit();
    rt1060UartInit();
    rt1060RadioInit();

    /* TODO: do we need other RT initializations */
}

void otSysDeinit(void)
{
    rt1060RadioDeinit();
    rt1060UartDeinit();
    rt1060ApiLockDeinit();
}

void otSysMainloopInit(otSysMainloopContext *aMainloop)
{
    OT_UNUSED_VARIABLE(aMainloop);

    /* TODO: add aMainLoop specific initializations */
}

void otSysMainloopUpdate(otInstance *aInstance, otSysMainloopContext *aMainloop)
{
    rt1060AlarmUpdate(aMainloop);
    rt1060UartUpdate(aMainloop);
    rt1060RadioUpdate(aMainloop);

    if (otTaskletsArePending(aInstance))
    {
        /* TODO: aMainloop updates? */
    }
}

int otSysMainloopPoll(otSysMainloopContext *aMainloop)
{
    OT_UNUSED_VARIABLE(aMainloop);

    /* TODO */

    return 0;
}

bool otSysPseudoResetWasRequested(void)
{
    /* TODO */

    return false;
}

void otSysMainloopProcess(otInstance *aInstance, const otSysMainloopContext *aMainloop)
{
    rt1060UartProcess();
    rt1060RadioProcess(aInstance);
    rt1060AlarmProcess(aInstance);
}
