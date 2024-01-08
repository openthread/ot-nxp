/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/************************************************************************************
* Include
************************************************************************************/
#include "board_lp.h"
#include "app.h"
#include "fsl_adapter_gpio.h"
#include "fsl_component_button.h"
#include "fsl_component_serial_manager.h"
#include "fsl_lpuart.h"
#include "fsl_wuu.h"
#include "fsl_cmc.h"
#include "fwk_platform.h"

#include "EmbeddedTypes.h"
#include "PhyInterface.h"

#define WUU_P11         11  // wake-up source is PTC6 = pin id is 11
#define kCMC_AllSRAMs   ( kCMC_SRAMBank0 | kCMC_SRAMBank1 | kCMC_SRAMBank2 | kCMC_SRAMBank3 | \
                          kCMC_SRAMBank4 | kCMC_SRAMBank5 | kCMC_SRAMBank6 | kCMC_SRAMBank7 )

bool g_bBtnAllowDeviceToSleep = TRUE;

void Btn_AllowDeviceToSleep(void)
{
    g_bBtnAllowDeviceToSleep = TRUE;
}

void Btn_DisallowDeviceToSleep(void)
{
    g_bBtnAllowDeviceToSleep = FALSE;
}

button_status_t Button_0_Handle(void *buttonHandle, button_callback_message_t *message,void *callbackParam)
{
    static unsigned char changed = 0;

    switch (message->event)
    {
        case kBUTTON_EventOneClick:
        case kBUTTON_EventShortPress:
        {
            if ( changed == 0 )
            {
                Btn_DisallowDeviceToSleep();
                changed = 1;
            }
            else
            {
                Btn_AllowDeviceToSleep();
                changed = 0;
            }

            break;
        }

        case kBUTTON_EventLongPress:
            /* No action required */
            break;

        default:
            /* No action required */
            break;
    }

    return kStatus_BUTTON_Success;
}

void WUU0_IRQHandler(void)
{
    /* Clear WUU pin status flag */
    WUU0->PF |= WUU0->PF;
}

static void BOARD_InitWakeupSource()
{
    /* Wakeup source is external pin. */
    wuu_external_wakeup_pin_config_t pinConfig;

    pinConfig.edge  = kWUU_ExternalPinAnyEdge;
    pinConfig.event = kWUU_ExternalPinInterrupt;
    pinConfig.mode  = kWUU_ExternalPinActiveDSPD;

    WUU_SetExternalWakeUpPinsConfig(WUU0, WUU_P11, &pinConfig);

    EnableIRQ((IRQn_Type) PORTC_EFT_IRQn);
    EnableIRQ(WUU0_IRQn);

    NVIC_EnableIRQ(WUU0_IRQn);
}

void BOARD_LowPower(void)
{
    RFMC_Type * rfmc_regs = RFMC;
    RF_CMC1_Type * regRF_CMC = RF_CMC1;
    uint32_t regVal;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // preamble
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    CMC_EnableDebugOperation(CMC0, false);
    CMC_SetPowerModeProtection(CMC0, kCMC_AllowAllLowPowerModes);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // prepare
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    regRF_CMC->RADIO_LP |= RF_CMC1_RADIO_LP_SLEEP_EN_MASK;

    regVal = rfmc_regs->RF2P4GHZ_CTRL;
    regVal &= ~RFMC_RF2P4GHZ_CTRL_LP_WKUP_DLY_MASK;
    regVal |=  RFMC_RF2P4GHZ_CTRL_LP_WKUP_DLY(0x10);
    regVal &= ~RFMC_RF2P4GHZ_CTRL_LP_MODE_MASK;
    regVal |=  RFMC_RF2P4GHZ_CTRL_LP_MODE(0x3);
    rfmc_regs->RF2P4GHZ_CTRL = regVal;

    regVal = rfmc_regs->RF2P4GHZ_CTRL;
    regVal &= ~RFMC_RF2P4GHZ_CTRL_BLE_WKUP_MASK;
    rfmc_regs->RF2P4GHZ_CTRL = regVal;

    rfmc_regs->RF2P4GHZ_CTRL |= RFMC_RF2P4GHZ_CTRL_CPU_RST_MASK;

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // enter
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    cmc_power_domain_config_t config;

    config.clock_mode  = kCMC_GateAllSystemClocksEnterLowPowerMode;
    config.main_domain = kCMC_DeepSleepMode;
    config.wake_domain = kCMC_SleepMode;

    CMC_PowerOnSRAMLowPowerOnly(CMC0, kCMC_AllSRAMs);
    CMC_EnterLowPowerMode(CMC0, &config);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // exit
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    config.clock_mode  = kCMC_GateNoneClock;
    config.main_domain = kCMC_ActiveMode;
    config.wake_domain = kCMC_ActiveMode;

    CMC_SetWAKEPowerMode(CMC0, config.wake_domain);
    CMC_SetMAINPowerMode(CMC0, config.main_domain);
    CMC_SetClockMode(CMC0, kCMC_GateNoneClock);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // restore
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    rfmc_regs->RF2P4GHZ_CTRL &= ~RFMC_RF2P4GHZ_CTRL_CPU_RST_MASK;

    regVal = rfmc_regs->RF2P4GHZ_CTRL;
    regVal &= ~( RFMC_RF2P4GHZ_CTRL_LP_ENTER_MASK | RFMC_RF2P4GHZ_CTRL_LP_MODE_MASK |
                 RFMC_RF2P4GHZ_CTRL_RST_MASK | RFMC_RF2P4GHZ_CTRL_RF_POR_MASK);
    rfmc_regs->RF2P4GHZ_CTRL = regVal;

    regVal = rfmc_regs->RF2P4GHZ_CTRL;
    regVal |= RFMC_RF2P4GHZ_CTRL_BLE_WKUP_MASK;
    rfmc_regs->RF2P4GHZ_CTRL = regVal;
    PLATFORM_Delay(1000U);

    regRF_CMC->RADIO_LP &= ~(RF_CMC1_RADIO_LP_SLEEP_EN_MASK);
}
