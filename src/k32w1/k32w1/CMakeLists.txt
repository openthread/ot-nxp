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

list(APPEND OT_PUBLIC_DEFINES
    -Wno-unused-parameter
    -Wno-expansion-to-defined
)

list(APPEND OT_PUBLIC_INCLUDES
        "${PROJECT_SOURCE_DIR}/include/"
        "${PROJECT_SOURCE_DIR}/openthread/third_party/mbedtls/"
        "${PROJECT_SOURCE_DIR}/openthread/third_party/mbedtls/repo/"
        "${PROJECT_SOURCE_DIR}/openthread/third_party/mbedtls/repo/include/"
        "$ENV{NXP_K32W1_SDK_ROOT}"
        "$ENV{NXP_K32W1_SDK_ROOT}/CMSIS/Include"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/common"
        "$ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480"
        "$ENV{NXP_K32W1_SDK_ROOT}/devices/K32W1480/drivers"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/flash_k4"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/gpio"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpspi"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lpuart"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/ltc"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/port"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/cmc"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/ccm32k"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/crc"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/tpm"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/drivers/lptmr"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/misc_utilities"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/debug_console"
        "$ENV{NXP_K32W1_SDK_ROOT}/platform/utilities/debug_console/str"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/osa"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/lists"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/messaging"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/mem_manager"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/internal_flash"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/panic"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/serial_manager"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/uart"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/gpio"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/led"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/button"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/timer_manager"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/timer"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/power_manager/boards"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/power_manager/boards/KW45B41Z-EVK"
        "$ENV{NXP_K32W1_SDK_ROOT}/components/power_manager/core"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/Common"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/kw45_k32w1"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/include"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/DBG"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/FunctionLib"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/HWParameter"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/SecLib"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/RNG"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/Sensors"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/LowPower"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/soc/KW45B41Z83"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/platform/KW45B41Z83"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/boards/kw45_k32w1"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/XCVR/drv/nb2p4ghz/configs/gen45"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/source"
        "$ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/ieee-802.15.4/ieee_802_15_4/phy/interface"
)

set(K32W1_SOURCES
    alarm.c
    diag.c
    entropy.c
    logging.c
    misc.c
    radio.c
    system.c
    uart.c
    power.c
    ecdsa_sss.cpp
    ../../common/flash_nvm.c
    ../../../openthread/examples/apps/cli/cli_uart.cpp
)

set(K32W1_LIBS
    $ENV{NXP_K32W1_SDK_ROOT}/middleware/wireless/framework/SecLib/lib_crypto_m33.a
)

set(K32W1_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
)

include(k32w1.cmake)
