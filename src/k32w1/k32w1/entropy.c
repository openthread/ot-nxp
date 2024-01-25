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
 *   This file implements an entropy source based on TRNG.
 *
 */

#include <openthread/platform/entropy.h>
#include "mbedtls/entropy_poll.h"
#include "utils/code_utils.h"

#if defined(USE_RTOS) && (USE_RTOS == 1)
#include "fsl_os_abstraction.h"

#define mutex_lock() OSA_MutexLock(trngMutexHandle, osaWaitForever_c)
#define mutex_unlock() OSA_MutexUnlock(trngMutexHandle)

OSA_MUTEX_HANDLE_DEFINE(trngMutexHandle);
#else
#define mutex_lock(...)
#define mutex_unlock(...)
#endif

void K32WRandomInit(void)
{
#if defined(USE_RTOS) && (USE_RTOS == 1)
    (void)OSA_MutexCreate(trngMutexHandle);
    otEXPECT(NULL != trngMutexHandle);
#endif

exit:
    return;
}

otError otPlatEntropyGet(uint8_t *aOutput, uint16_t aOutputLength)
{
    otError error     = OT_ERROR_NONE;
    size_t  outputLen = 0;

    mutex_lock();

    otEXPECT_ACTION(aOutput, error = OT_ERROR_INVALID_ARGS);

    for (size_t partialLen = 0; outputLen < aOutputLength; outputLen += partialLen)
    {
        const uint16_t remaining = aOutputLength - outputLen;
        partialLen               = 0;

        // Non-zero return values for mbedtls_hardware_poll() signify an error has occurred
        otEXPECT_ACTION(0 == mbedtls_hardware_poll(NULL, &aOutput[outputLen], remaining, &partialLen),
                        error = OT_ERROR_FAILED);
    }

exit:
    mutex_unlock();
    return error;
}
