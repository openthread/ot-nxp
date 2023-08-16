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
 *   This file implements the OpenThread platform abstraction for UART communication.
 *
 */

#include "fsl_device_registers.h"
#include <stddef.h>
#include <stdint.h>

#include "openthread-system.h"
#include <utils/code_utils.h>
#include <openthread/platform/alarm-milli.h>
#include "utils/uart.h"

#include "fsl_clock.h"
#include "fsl_lpuart.h"
#include "fsl_port.h"

#include "app.h"
#include "fwk_platform_ot.h"
#include "pin_mux.h"

#define FLUSH_TO_MS 500

extern void PWR_DisallowDeviceToSleep(void);
extern void PWR_AllowDeviceToSleep(void);

static SERIAL_MANAGER_HANDLE_DEFINE(otCliSerialHandle);
static SERIAL_MANAGER_WRITE_HANDLE_DEFINE(otCliSerialWriteHandle);
static SERIAL_MANAGER_READ_HANDLE_DEFINE(otCliSerialReadHandle);

static void Uart_RxCallBack(void *pData, serial_manager_callback_message_t *message, serial_manager_status_t status);
static void Uart_TxCallBack(void *pBuffer, serial_manager_callback_message_t *message, serial_manager_status_t status);

uint8_t rxBuffer[kReceiveBufferSize];

static serial_port_uart_config_t uartConfig = {
    .instance     = OT_APP_UART_INSTANCE,
    .baudRate     = OT_APP_UART_BAUDRATE,
    .parityMode   = kSerialManager_UartParityDisabled,
    .stopBitCount = kSerialManager_UartOneStopBit,
    .enableRx     = 1,
    .enableTx     = 1,
};

static uint8_t                       s_ringBuffer[SERIAL_MANAGER_RING_BUFFER_SIZE];
static const serial_manager_config_t s_serialManagerConfig = {
    .type           = OT_APP_UART_TYPE,
    .ringBuffer     = &s_ringBuffer[0],
    .ringBufferSize = SERIAL_MANAGER_RING_BUFFER_SIZE,
    .portConfig     = (serial_port_uart_config_t *)&uartConfig,
};

static bool_t otPlatUartEnabled = FALSE;
static bool_t sUartRxFired      = FALSE;
/* tx flush ongoing */
volatile bool_t gTxFlush = FALSE;
/* pending transmissions */
volatile uint8_t gTxCntSerMgrIf = 0;

otError otPlatUartEnable(void)
{
    serial_manager_status_t status;
    otError                 error = OT_ERROR_NONE;
    otPlatUartEnabled             = TRUE;

    /* set clock */
    CLOCK_SetIpSrc(OT_APP_UART_CLK, OT_APP_UART_CLKSRC);
    /* enable clock */
    CLOCK_EnableClock(OT_APP_UART_CLK);

    uartConfig.clockRate = CLOCK_GetIpFreq(OT_APP_UART_CLK);

#if (OT_APP_UART_INSTANCE == 1U)
    BOARD_InitPinLPUART1_TX();
    BOARD_InitPinLPUART1_RX();
#elif (OT_APP_UART_INSTANCE == 0U)
    BOARD_InitPinLPUART0_TX();
    BOARD_InitPinLPUART0_RX();
#else
#error Only LPUART0 or LPUART1 supported
#endif

    /* Init Serial Manager */
    status = SerialManager_Init((serial_handle_t)otCliSerialHandle, &s_serialManagerConfig);
    otEXPECT_ACTION(status == kStatus_SerialManager_Success, error = OT_ERROR_FAILED);

    SerialManager_OpenWriteHandle((serial_handle_t)otCliSerialHandle, (serial_write_handle_t)otCliSerialWriteHandle);
    SerialManager_OpenReadHandle((serial_handle_t)otCliSerialHandle, (serial_read_handle_t)otCliSerialReadHandle);

    SerialManager_InstallRxCallback((serial_read_handle_t)otCliSerialReadHandle, Uart_RxCallBack, NULL);
    SerialManager_InstallTxCallback((serial_write_handle_t)otCliSerialWriteHandle, Uart_TxCallBack, NULL);

exit:
    return error;
}

void otPlatUartProcess()
{
    uint32_t bytesRead = 0U;
    if ((otPlatUartEnabled) && (sUartRxFired))
    {
        sUartRxFired = FALSE;
        PWR_AllowDeviceToSleep();
        if ((SerialManager_TryRead((serial_read_handle_t)otCliSerialReadHandle, rxBuffer, kReceiveBufferSize,
                                   &bytesRead) == kStatus_SerialManager_Success) &&
            (bytesRead != 0))
        {
            otPlatUartReceived(rxBuffer, bytesRead);
        }
    }
}

otError otPlatUartDisable(void)
{
    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    serial_manager_status_t status;
    otError                 error = OT_ERROR_NONE;

    /* new tx */
    gTxCntSerMgrIf++;

    status = SerialManager_WriteNonBlocking((serial_write_handle_t)otCliSerialWriteHandle, (uint8_t *)aBuf, aBufLength);
    otEXPECT_ACTION(status == kStatus_SerialManager_Success, gTxCntSerMgrIf--; error = OT_ERROR_FAILED);

exit:
    return error;
}

void otPlatUartSendBlocking(const uint8_t *aBuf, uint32_t len)
{
    if (otPlatUartEnabled)
    {
        SerialManager_WriteBlocking((serial_write_handle_t)otCliSerialWriteHandle, (uint8_t *)aBuf, len);
    }
}

otError otPlatUartFlush(void)
{
    if (gTxFlush)
    {
        return OT_ERROR_NONE;
    }

    uint32_t start = otPlatAlarmMilliGetNow();

    gTxFlush = TRUE;

    /* decremented in SerialMngr_TxCbApp() */
    while (gTxCntSerMgrIf)
    {
        if ((otPlatAlarmMilliGetNow() - start) > FLUSH_TO_MS)
        {
            break;
        }
    }

    gTxFlush = FALSE;

    return OT_ERROR_NONE;
}

static void Uart_RxCallBack(void *pData, serial_manager_callback_message_t *message, serial_manager_status_t status)
{
    sUartRxFired = TRUE;
    PWR_DisallowDeviceToSleep();
    otSysEventSignalPending();
}

static void Uart_TxCallBack(void *pBuffer, serial_manager_callback_message_t *message, serial_manager_status_t status)
{
    /* tx finished */
    if (gTxCntSerMgrIf)
    {
        gTxCntSerMgrIf--;
    }

    if (!gTxFlush)
    {
        otPlatUartSendDone();
    }
}

OT_TOOL_WEAK void otPlatUartSendDone(void)
{
}

OT_TOOL_WEAK void otPlatUartReceived(const uint8_t *aBuf, uint16_t aBufLength)
{
    OT_UNUSED_VARIABLE(aBuf);
    OT_UNUSED_VARIABLE(aBufLength);
}
