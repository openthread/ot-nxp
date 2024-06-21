#
#  Copyright (c) 2021, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

set(UART_BAUD_RATE "115200" CACHE STRING "Uart baud rate")
set(UART_INSTANCE "1" CACHE STRING "Uart instance for RCP comms")

list(APPEND OT_PLATFORM_DEFINES
    "USE_NBU=${USE_NBU}"
    "CPU_K32W1480VFTA"
    "SERIAL_MANAGER_NON_BLOCKING_MODE=1"
    "SERIAL_USE_CONFIGURE_STRUCTURE=1"
    "SDK_COMPONENT_INTEGRATION=1"
    "SERIAL_PORT_TYPE_UART=1"
    "gSerialManagerMaxInterfaces_c=2"
    "HAL_RPMSG_SELECT_ROLE=0"
    "TM_ENABLE_TIME_STAMP=1"
    "FSL_OSA_TASK_ENABLE=1"
    "gAspCapability_d=1"
    "gNvStorageIncluded_d=1"
    "gUnmirroredFeatureSet_d=1"
    "gNvFragmentation_Enabled_d=1"
    "gAppButtonCnt_c=2"
    "gBleBondIdentityHeaderSize_c=56"
    "gPlatformShutdownEccRamInLowPower=0"
    "gMemManagerLightExtendHeapAreaUsage=1"
    "gAppHighSystemClockFrequency_d=1"
    "SSS_CONFIG_FILE=\"fsl_sss_config_elemu.h\""
    "SSCP_CONFIG_FILE=\"fsl_sscp_config_elemu.h\""
    "DBG_IO_ENABLE"
)

if (USE_NBU)
    set_property(SOURCE ${PROJECT_SOURCE_DIR}/src/k32w1/k32w1/startup_K32W1480.S PROPERTY LANGUAGE C)
else()
    set_property(SOURCE $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/gcc/startup_K32W1480.S PROPERTY LANGUAGE C)
endif()

if (OT_APP_LOWPOWER)
    list(APPEND OT_PLATFORM_DEFINES
        "gAppLowpowerEnabled_d=1"
    )
else()
    list(APPEND OT_PLATFORM_DEFINES
        "gAppLowpowerEnabled_d=0"
    )
endif()

set(K32W1_SDK_SOURCES
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/FunctionLib/FunctionLib.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/HWParameter/HWParameter.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1/fwk_platform.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1/fwk_platform_lowpower.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1/fwk_platform_lowpower_timer.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1/fwk_platform_ics.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1/fwk_platform_ot.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1/fwk_platform_extflash.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/NVM/Source/NV_Flash.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/LowPower/PWR.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/RNG/RNG.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/SecLib/SecLib_sss.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/K32W1480/clock_config.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/hardware_init.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/app_services_init.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/K32W1480/pin_mux.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/board_comp.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/board_dcdc.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/board_lp.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/board.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/board_extflash.c
    $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/system_K32W1480.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/osa/fsl_os_abstraction_bm.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/lists/fsl_component_generic_list.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/messaging/fsl_component_messaging.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/mem_manager/fsl_component_mem_manager_light.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/panic/fsl_component_panic.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/serial_manager/fsl_component_serial_manager.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/serial_manager/fsl_component_serial_port_uart.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/uart/fsl_adapter_lpuart.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/gpio/fsl_adapter_gpio.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/led/fsl_component_led.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/button/fsl_component_button.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/timer_manager/fsl_component_timer_manager.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/timer/fsl_adapter_lptmr.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/time_stamp/fsl_adapter_lptmr_time_stamp.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/internal_flash/fsl_adapter_k4_flash.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/power_manager/boards/K32W148-EVK/fsl_pm_board.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/power_manager/core/fsl_pm_core.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/flash/nor/lpspi/fsl_lpspi_nor_flash.c
    $ENV{NXP_K32W1_SDK_ROOT}/components/flash/nor/lpspi/fsl_lpspi_mem_adapter.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/dbg_ram_capture.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/mathfp.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/rfmc_ctrl.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/nxp2p4_xcvr.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/nxp_xcvr_trim.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen45/nxp_xcvr_coding_config.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen45/nxp_xcvr_common_config.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen45/nxp_xcvr_mode_config.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen45/nxp_xcvr_msk_config.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen45/nxp_xcvr_oqpsk_802p15p4_config.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/src/sscp/fsl_sscp_mu.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/src/sscp/fsl_sss_mgmt.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/src/sscp/fsl_sss_sscp.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/port/kw45_k4w1/sss_init.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/port/kw45_k4w1/sss_aes.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/port/kw45_k4w1/sss_ccm.c
    ${PROJECT_SOURCE_DIR}/third_party/k32w1_sdk/board_lp.c
)

if (NOT SDK_RELEASE)
    list(APPEND K32W1_SDK_SOURCES
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/utils/dbg_io.c
    )
endif()

if (USE_NBU)
    if (SDK_RELEASE)
        list(APPEND K32W1_SDK_SOURCES
            $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_imu.c
        )
    else()
        list(APPEND K32W1_SDK_SOURCES
            $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/imu/fsl_imu.c
        )
    endif()
    list(APPEND K32W1_SDK_SOURCES
        ${PROJECT_SOURCE_DIR}/src/k32w1/k32w1/startup_K32W1480.S
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/SerialDevice/ASP.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/SerialDevice/Phy.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhyTime.c
        $ENV{NXP_K32W1_SDK_ROOT}/components/rpmsg/fsl_adapter_rpmsg.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/mcmgr/src/mcmgr.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/rpmsg_lite.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/porting/environment/rpmsg_env_bm.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/rpmsg_lite/porting/platform/k32w1/rpmsg_platform.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/common/llist.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/virtio/virtqueue.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/mcmgr/src/mcmgr_imu_internal.c
        $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/mcmgr/src/mcmgr_internal_core_api_k32w1.c
    )
else()
     list(APPEND K32W1_SDK_SOURCES
       $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/gcc/startup_K32W1480.S
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhyPacket.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/ASP.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/MPM.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhyISR.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhyPacketProcessor.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhyPlmeData.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhyStateMachine.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhyTime.c
       $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source/PhySec.c
       )
endif()

if (SDK_RELEASE)
    list(APPEND K32W1_SDK_SOURCES
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_k4_flash.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_k4_controller.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_clock.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_gpio.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_lpuart.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_lpspi.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_ltc.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_lptmr.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_elemu.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_spc.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_cmc.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_ccm32k.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_crc.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_wuu.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_edma.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers/fsl_lpuart_edma.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/utilities/fsl_assert.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/utilities/debug_console/fsl_debug_console.c
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/utilities/str/fsl_str.c
)
else()
    list(APPEND K32W1_SDK_SOURCES
        $ENV{NXP_K32W1_SDK_ROOT}/devices/KW45B41Z83/drivers/fsl_clock.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/flash_k4/fsl_k4_flash.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/flash_k4/fsl_k4_controller.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/gpio/fsl_gpio.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpuart/fsl_lpuart.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/ltc/fsl_ltc.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lptmr/fsl_lptmr.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpspi/fsl_lpspi.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/assert/fsl_assert.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/debug_console/fsl_debug_console.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/str/fsl_str.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/elemu/fsl_elemu.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/spc/fsl_spc.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/cmc/fsl_cmc.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/ccm32k/fsl_ccm32k.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/crc/fsl_crc.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/wuu/fsl_wuu.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/dma3/fsl_edma.c
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpuart/fsl_lpuart_edma.c
)
endif()


add_library(nxp-k32w1-driver
    ${K32W1_SDK_SOURCES}
)

if (OT_RCP_FLOW_CONTROL)
    target_compile_definitions(nxp-k32w1-driver
        PUBLIC
             -DOT_APP_UART_BAUDRATE=${UART_BAUD_RATE}
             -DSERIAL_MANAGER_RING_BUFFER_SIZE=320
             -DOT_APP_SERIAL_PORT_USE_FC=1
             -DOT_APP_UART_INSTANCE=${UART_INSTANCE}
    )
endif()

if (OT_RCP_UART_DMA)
    target_compile_definitions(nxp-k32w1-driver
        PUBLIC
             -DSERIAL_PORT_TYPE_UART_DMA=1
             -DOT_APP_UART_BAUDRATE=${UART_BAUD_RATE}
             -DSERIAL_MANAGER_RING_BUFFER_SIZE=320
             -DOT_APP_UART_INSTANCE=${UART_INSTANCE}
             -DHAL_UART_DMA_ENABLE=1
             -DOT_APP_UART_TYPE=kSerialPort_UartDma
             -DOT_APP_SERIAL_PORT_USE_DMA=1
             -DLPUART_RING_BUFFER_SIZE=1024
    )
endif()

target_compile_definitions(nxp-k32w1-driver
    PUBLIC
        ${OT_PLATFORM_DEFINES}
)

target_compile_options(nxp-k32w1-driver
    PRIVATE
        ${OT_CFLAGS}
        -Wno-unknown-pragmas
        -Wno-sign-compare
        -Wno-unused-function
        -Wno-unused-parameter
        -Wno-empty-body
        -Wno-missing-field-initializers
        -Wno-clobbered
        -fno-strict-aliasing
)

add_library(nxp-k32w1-mbedtls
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/aes.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/aesni.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/arc4.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/aria.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/asn1parse.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/asn1write.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/base64.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/bignum.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/blowfish.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/camellia.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ccm.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/certs.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/cipher.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/cipher_wrap.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/chacha20.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/chachapoly.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/cmac.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/constant_time.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ctr_drbg.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/debug.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/des.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/dhm.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ecdh.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ecdsa.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ecjpake.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ecp.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ecp_curves.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/entropy.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/error.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/gcm.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/havege.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/hkdf.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/hmac_drbg.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/md.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/md2.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/md4.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/md5.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/memory_buffer_alloc.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/net_sockets.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/nist_kw.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/oid.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/padlock.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pem.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pk.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pk_wrap.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pkcs5.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pkcs11.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pkcs12.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pkparse.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/pkwrite.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/platform.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/platform_util.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/poly1305.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto_client.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto_driver_wrappers.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto_ecp.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto_rsa.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto_se.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto_slot_management.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_crypto_storage.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/psa_its_file.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ripemd160.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/rsa.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/rsa_internal.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/sha1.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/sha256.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/sha512.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_cache.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_ciphersuites.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_cli.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_cookie.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_msg.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_srv.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_ticket.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_tls.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/ssl_tls13_keys.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/threading.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/timing.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/version.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/version_features.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/x509.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/x509_create.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/x509_crl.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/x509_crt.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/x509_csr.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/x509write_crt.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/x509write_csr.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library/xtea.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/port/sssapi/entropy_poll_alt.c
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/port/sssapi/sssapi_mbedtls.c
)

target_include_directories(nxp-k32w1-mbedtls
    PRIVATE
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/library
    PUBLIC
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/include
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/mbedtls/port/sssapi
)

target_compile_definitions(nxp-k32w1-mbedtls
    PUBLIC
    "MBEDTLS_USER_CONFIG_FILE=\"k32w1-mbedtls-config.h\""
    "MBEDTLS_CONFIG_FILE=\"${PROJECT_SOURCE_DIR}/openthread/third_party/mbedtls/mbedtls-config.h\""
)

set(K32W1_SDK_INCLUDES
    ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
    ${PROJECT_SOURCE_DIR}/third_party/k32w1_sdk
    $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480
    $ENV{NXP_K32W1_SDK_ROOT}/CMSIS/Core/Include
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/inc
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/inc/elemu
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/secure-subsystem/port/kw45_k4w1
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/Common
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/FunctionLib
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/HWParameter
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/SecLib
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/RNG
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/Sensors
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/NVM/Source
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/NVM/Interface
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1/K32W1480
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/include
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1/configs
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/utils/
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen45
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/interface
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/include
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/include/platform/k32w1
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/rpmsg_lite/lib/include/environment/bm
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/multicore/mcmgr/src
    $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/elemu
    $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/spc
    $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpspi
    $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/ccm32k
    $ENV{NXP_K32W1_SDK_ROOT}/components/osa
    $ENV{NXP_K32W1_SDK_ROOT}/components/lists
    $ENV{NXP_K32W1_SDK_ROOT}/components/messaging
    $ENV{NXP_K32W1_SDK_ROOT}/components/mem_manager
    $ENV{NXP_K32W1_SDK_ROOT}/components/panic
    $ENV{NXP_K32W1_SDK_ROOT}/components/serial_manager
    $ENV{NXP_K32W1_SDK_ROOT}/components/uart
    $ENV{NXP_K32W1_SDK_ROOT}/components/gpio
    $ENV{NXP_K32W1_SDK_ROOT}/components/led
    $ENV{NXP_K32W1_SDK_ROOT}/components/button
    $ENV{NXP_K32W1_SDK_ROOT}/components/timer_manager
    $ENV{NXP_K32W1_SDK_ROOT}/components/time_stamp
    $ENV{NXP_K32W1_SDK_ROOT}/components/timer
    $ENV{NXP_K32W1_SDK_ROOT}/components/rpmsg
    $ENV{NXP_K32W1_SDK_ROOT}/components/internal_flash
    $ENV{NXP_K32W1_SDK_ROOT}/components/flash/nor/lpspi
    $ENV{NXP_K32W1_SDK_ROOT}/components/flash/nor
    $ENV{NXP_K32W1_SDK_ROOT}/components/power_manager/boards/K32W148-EVK
)

if (SDK_RELEASE)
    list(APPEND K32W1_SDK_INCLUDES
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/utilities
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/utilities/debug_console
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/utilities/str
        $ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/utilities/format
)
else()
    list(APPEND K32W1_SDK_INCLUDES
        $ENV{NXP_K32W1_SDK_ROOT}/devices/KW45B41Z83/drivers
        $ENV{NXP_K32W1_SDK_ROOT}/devices/KW45B41Z83/drivers/romapi
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/common
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/flash_k4
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/gpio
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpuart
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/ltc
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/port
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lptmr
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpspi
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/imu
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/wuu
        $ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/dma3
        $ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/misc_utilities
        $ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/debug_console
        $ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/str
)
endif()

target_include_directories(nxp-k32w1-driver
    PUBLIC
    ${K32W1_SDK_INCLUDES}
)

target_link_libraries(nxp-k32w1-driver PRIVATE ot-config)

target_link_libraries(nxp-k32w1-mbedtls PRIVATE ot-config)
