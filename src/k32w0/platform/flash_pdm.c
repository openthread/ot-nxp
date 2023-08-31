/*
 *  Copyright (c) 2016-2022, The OpenThread Authors.
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
 *   This file implements the OpenThread platform abstraction
 *    for non-volatile storage of settings on K32W platform.
 *
 */
#include <openthread-core-config.h>

#include "fsl_os_abstraction.h"
#include <string.h>
#include <openthread/instance.h>
#include <openthread/platform/memory.h>
#include <openthread/platform/settings.h>
#include "utils/code_utils.h"

#include "PDM.h"
#include "pdm_ram_storage_glue.h"
#include "ram_storage.h"

static ramBufferDescriptor *ramDescr       = NULL;
static osaMutexId_t         pdmMutexHandle = NULL;
static bool_t               pdmMutexTaken  = FALSE;

#if PDM_SAVE_IDLE
static bool_t settingsInitialized = FALSE;
#define mutex_lock OSA_MutexLock
#define mutex_unlock OSA_MutexUnlock
#define mutex_destroy OSA_MutexDestroy
#else
#define mutex_lock(...)
#define mutex_unlock(...)
#define mutex_destroy(...)
#endif /* PDM_SAVE_IDLE */

#define kNvmIdOTConfigData 0x4F00
#define kRamBufferInitialSize 1024

static otError mapRamStorageStatus(rsError rsStatus)
{
    otError error;

    switch (rsStatus)
    {
    case RS_ERROR_NONE:
        error = OT_ERROR_NONE;
        break;
    case RS_ERROR_NOT_FOUND:
        error = OT_ERROR_NOT_FOUND;
        break;
    default:
        error = OT_ERROR_NO_BUFS;
        break;
    }

    return error;
}

void otPlatSettingsInit(otInstance *aInstance, const uint16_t *aSensitiveKeys, uint16_t aSensitiveKeysLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    OT_UNUSED_VARIABLE(aSensitiveKeys);
    OT_UNUSED_VARIABLE(aSensitiveKeysLength);
    otError error = OT_ERROR_NONE;

    otEXPECT_ACTION((TRUE == PDM_RetrieveSegmentSize()), error = OT_ERROR_NO_BUFS);

#if PDM_SAVE_IDLE
    /* settings may have been already initialized:
     * e.g.: for PDM_SAVE_IDLE in XCVR context
     */
    if (settingsInitialized)
        return;

    otEXPECT_ACTION((TRUE == FS_Init()), error = OT_ERROR_NO_BUFS);
#endif

    otEXPECT_ACTION((PDM_E_STATUS_OK == PDM_Init()), error = OT_ERROR_NO_BUFS);

    ramDescr = getRamBuffer(kNvmIdOTConfigData, kRamBufferInitialSize, FALSE);
    otEXPECT_ACTION(ramDescr != NULL, error = OT_ERROR_NO_BUFS);
    otEXPECT_ACTION(ramDescr->buffer != NULL, error = OT_ERROR_NO_BUFS);
#if PDM_SAVE_IDLE
    pdmMutexHandle = ramDescr->header.mutexHandle;
#endif

exit:
#if PDM_SAVE_IDLE
    if (error != OT_ERROR_NONE)
    {
        if (pdmMutexHandle)
        {
            mutex_destroy(pdmMutexHandle);
        }

        FS_Deinit();
    }
    else
    {
        settingsInitialized = TRUE;
    }
#endif

    return;
}

void otPlatSettingsDeinit(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

#if ENABLE_STORAGE_DYNAMIC_MEMORY
    mutex_lock(pdmMutexHandle, osaWaitForever_c);
    pdmMutexTaken = TRUE;
    if (ramDescr->buffer)
    {
        otPlatFree(ramDescr->buffer);
        ramDescr->buffer = NULL;
    }
    pdmMutexTaken = FALSE;
    mutex_unlock(pdmMutexHandle);
    mutex_destroy(pdmMutexHandle);

    otPlatFree(ramDescr);
    ramDescr = NULL;
#endif

#if PDM_SAVE_IDLE
    FS_Deinit();
#endif
}

otError otPlatSettingsGet(otInstance *aInstance, uint16_t aKey, int aIndex, uint8_t *aValue, uint16_t *aValueLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    rsError ramStatus = RS_ERROR_NONE;

    if (!OSA_InIsrContext())
    {
        mutex_lock(pdmMutexHandle, osaWaitForever_c);
        pdmMutexTaken = TRUE;
    }

    ramStatus = ramStorageGet(ramDescr, aKey, aIndex, aValue, aValueLength);

    if (!OSA_InIsrContext())
    {
        pdmMutexTaken = FALSE;
        mutex_unlock(pdmMutexHandle);
    }

    return mapRamStorageStatus(ramStatus);
}

otError otPlatSettingsSet(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength)
{
    OT_UNUSED_VARIABLE(aInstance);
    rsError      ramStatus = RS_ERROR_NONE;
    PDM_teStatus pdmStatus = PDM_E_STATUS_OK;
    otError      error     = OT_ERROR_NONE;

#if ENABLE_STORAGE_DYNAMIC_MEMORY
    uint16_t lengthOfAlreadyExistingValue = 0;
#endif

    if (!OSA_InIsrContext())
    {
        mutex_lock(pdmMutexHandle, osaWaitForever_c);
        pdmMutexTaken = TRUE;
    }

#if ENABLE_STORAGE_DYNAMIC_MEMORY
    /* avoid resizing in case ramDescr already contains aValue whose length is >= aValueLength */
    if ((ramStorageGet(ramDescr, aKey, 0, NULL, &lengthOfAlreadyExistingValue) == RS_ERROR_NONE) &&
        (lengthOfAlreadyExistingValue < aValueLength))
    {
        ramStatus = ramStorageResize(ramDescr, aKey, aValue, aValueLength - lengthOfAlreadyExistingValue);
        otEXPECT_ACTION((RS_ERROR_NONE == ramStatus), error = mapRamStorageStatus(ramStatus));
    }
#endif
    ramStatus = ramStorageSet(ramDescr, aKey, aValue, aValueLength);
    otEXPECT_ACTION((RS_ERROR_NONE == ramStatus), error = mapRamStorageStatus(ramStatus));

    pdmStatus = PDM_SaveRecord((uint16_t)kNvmIdOTConfigData, ramDescr);
    otEXPECT_ACTION((PDM_E_STATUS_OK == pdmStatus), error = OT_ERROR_NO_BUFS);

exit:
    if (!OSA_InIsrContext())
    {
        pdmMutexTaken = FALSE;
        mutex_unlock(pdmMutexHandle);
    }
    return error;
}

otError otPlatSettingsAdd(otInstance *aInstance, uint16_t aKey, const uint8_t *aValue, uint16_t aValueLength)
{
    rsError      ramStatus = RS_ERROR_NONE;
    PDM_teStatus pdmStatus = PDM_E_STATUS_OK;
    otError      error     = OT_ERROR_NONE;

    mutex_lock(pdmMutexHandle, osaWaitForever_c);
    pdmMutexTaken = TRUE;

#if ENABLE_STORAGE_DYNAMIC_MEMORY
    ramStatus = ramStorageResize(ramDescr, aKey, aValue, aValueLength);
    otEXPECT_ACTION((RS_ERROR_NONE == ramStatus), error = mapRamStorageStatus(ramStatus));
#endif
    ramStatus = ramStorageAdd(ramDescr, aKey, aValue, aValueLength);
    otEXPECT_ACTION((RS_ERROR_NONE == ramStatus), error = mapRamStorageStatus(ramStatus));

    pdmStatus = PDM_SaveRecord((uint16_t)kNvmIdOTConfigData, ramDescr);
    otEXPECT_ACTION((PDM_E_STATUS_OK == pdmStatus), error = OT_ERROR_NO_BUFS);

exit:
    pdmMutexTaken = FALSE;
    mutex_unlock(pdmMutexHandle);
    return error;
}

otError otPlatSettingsDelete(otInstance *aInstance, uint16_t aKey, int aIndex)
{
    OT_UNUSED_VARIABLE(aInstance);
    rsError      ramStatus = RS_ERROR_NONE;
    PDM_teStatus pdmStatus = PDM_E_STATUS_OK;
    otError      error     = OT_ERROR_NONE;

    mutex_lock(pdmMutexHandle, osaWaitForever_c);
    pdmMutexTaken = TRUE;
    ramStatus     = ramStorageDelete(ramDescr, aKey, aIndex);
    otEXPECT_ACTION((RS_ERROR_NONE == ramStatus), error = mapRamStorageStatus(ramStatus));

    pdmStatus = PDM_SaveRecord((uint16_t)kNvmIdOTConfigData, ramDescr);
    otEXPECT_ACTION((PDM_E_STATUS_OK == pdmStatus), error = OT_ERROR_NO_BUFS);

exit:
    pdmMutexTaken = FALSE;
    mutex_unlock(pdmMutexHandle);
    return error;
}

void otPlatSettingsWipe(otInstance *aInstance)
{
    OT_UNUSED_VARIABLE(aInstance);

    mutex_lock(pdmMutexHandle, osaWaitForever_c);
    pdmMutexTaken = TRUE;
    memset(ramDescr->buffer, 0, ramDescr->header.maxLength);
    ramDescr->header.length = 0;
    PDM_vDeleteDataRecord(kNvmIdOTConfigData);
    pdmMutexTaken = FALSE;
    mutex_unlock(pdmMutexHandle);
}

#if gRadioUsePdm_d && PDM_SAVE_IDLE

/* in case BLE ISR tries to do a recalibration, make sure that the Ram Buffer Mutex is not
 * taken, otherwise execution will be stuck in a FreeRtos assert.
 */
bool should_skip_recal()
{
    return (OSA_InIsrContext() && pdmMutexTaken && idleMutexIsTaken());
}

bool bRadioCB_WriteNVM(uint8_t *pu8DataBlock, uint16_t u16Len)
{
    return (otPlatSettingsSet(NULL, PDM_ID_RADIO_SETTINGS, pu8DataBlock, u16Len) == OT_ERROR_NONE);
}

uint16_t u16RadioCB_ReadNVM(uint8_t *pu8DataBlock, uint16_t u16MaxLen)
{
    uint16_t lengthPtr = 0;

    /* lengthPtr will be 0 if no record is found */
    otPlatSettingsGet(NULL, PDM_ID_RADIO_SETTINGS, 0, pu8DataBlock, &lengthPtr);

    return lengthPtr;
}
#endif /* gRadioUsePdm_d && PDM_SAVE_IDLE */
