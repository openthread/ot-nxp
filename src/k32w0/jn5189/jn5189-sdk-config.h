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

#ifndef JN5189_SDK_CONFIG_H
#define JN5189_SDK_CONFIG_H

#include "openthread-core-jn5189-config.h"

#ifdef APP_PRE_INCLUDE
#include APP_PRE_INCLUDE
#endif

#ifndef gUsePdm_d
#define gUsePdm_d 1
#endif

#ifndef gPdmMemPoolId_c
#define gPdmMemPoolId_c 0
#endif

#ifndef gPdmNbSegments
#define gPdmNbSegments 63 /* number of sectors contained in PDM storage */
#endif

#ifndef USE_RTOS
#define USE_RTOS 0
#endif

#ifndef USE_SDK_OSA
#define USE_SDK_OSA 0
#endif

#ifndef gSerialManagerMaxInterfaces_c
#define gSerialManagerMaxInterfaces_c 2
#endif

#ifndef SUPPORT_FOR_15_4
#define SUPPORT_FOR_15_4 1
#endif

#define UART_USE_DRIVER 0
#define UART_USE_SERIAL_MGR 1
#define UART_USE_DRIVER_LOG 0
#define UART_USE_SERIAL_MGR_LOG 1
#define UART_USE_SWO_LOG 0

#ifndef SDK_DEBUGCONSOLE
// #if ((OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_PLATFORM_DEFINED) && (UART_USE_SWO_LOG == 1))
#if ((OPENTHREAD_CONFIG_LOG_OUTPUT == 3) && (UART_USE_SWO_LOG == 1))
#define SDK_DEBUGCONSOLE DEBUGCONSOLE_REDIRECT_TO_SDK
#else
#define SDK_DEBUGCONSOLE DEBUGCONSOLE_DISABLE
#endif
#endif

#if (UART_USE_SWO_LOG == 1)
#define SERIAL_PORT_TYPE_SWO 1
#define SERIAL_PORT_TYPE_UART 0
#endif

#ifndef gUartDebugConsole_d
#define gUartDebugConsole_d 0
#endif

#ifndef PoolsDetails_c
#define PoolsDetails_c                                 \
    _block_size_ 512 _number_of_blocks_ 2 _pool_id_(0) \
        padding _eol_ _block_size_ 768 _number_of_blocks_ 1 _pool_id_(0) padding _eol_
#endif

#endif // K32W061_SDK_CONFIG_H
