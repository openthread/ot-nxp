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
list(APPEND OT_PLATFORM_DEFINES
    "CPU_K32W061HN"
    "CPU_JN518X"
    "CPU_JN518X_REV=2"
    "JENNIC_CHIP_FAMILY_JN518x"
    "JENNIC_CHIP_FAMILY_NAME=_JN518x"
    "USE_SDK_OSA=0"
    "SDK_DEBUGCONSOLE=0"
)

list(APPEND OT_PUBLIC_DEFINES
    -DCPU_K32W061HN
    -DCPU_JN518X
    -DCPU_JN518X_REV=2
    -DJENNIC_CHIP_FAMILY_JN518x
    -DJENNIC_CHIP_FAMILY_NAME=_JN518x
    -Wno-unused-parameter
    -Wno-expansion-to-defined
)

list(APPEND OT_PUBLIC_INCLUDES
        "${PROJECT_SOURCE_DIR}/include/"
        "${PROJECT_SOURCE_DIR}/openthread/third_party/mbedtls/"
        "${PROJECT_SOURCE_DIR}/openthread/third_party/mbedtls/repo/"
        "${PROJECT_SOURCE_DIR}/openthread/third_party/mbedtls/repo/include/"
        "$ENV{NXP_K32W061_SDK_ROOT}"
        "$ENV{NXP_K32W061_SDK_ROOT}/CMSIS/Include/"
        "$ENV{NXP_K32W061_SDK_ROOT}/components/serial_manager/"
        "$ENV{NXP_K32W061_SDK_ROOT}/devices/K32W061"
        "$ENV{NXP_K32W061_SDK_ROOT}/devices/K32W061/drivers/"
        "$ENV{NXP_K32W061_SDK_ROOT}/devices/K32W061/utilities/"
        "$ENV{NXP_K32W061_SDK_ROOT}/devices/K32W061/utilities/debug-console/"
        "$ENV{NXP_K32W061_SDK_ROOT}/devices/K32W061/utilities/debug_console/"
        "$ENV{NXP_K32W061_SDK_ROOT}/devices/K32W061/utilities/str/"
        "$ENV{NXP_K32W061_SDK_ROOT}/drivers/components/uart/"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/mbedtls/port/ksdk/"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/ieee-802.15.4/uMac/Include"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/Common/"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/Lists"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/MemManager/Interface"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/PDM/Include"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/SerialManager/Interface"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/TimersManager/Source/"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/XCVR/DK6"
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/XCVR/DK6/Build/Include"
)

if (SDK_RELEASE)
    list(APPEND OT_PUBLIC_INCLUDES
        "$ENV{NXP_K32W061_SDK_ROOT}/boards/k32w061dk6/wireless_examples/openthread/enablement"
    )
else()
    list(APPEND OT_PUBLIC_INCLUDES
        "$ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/openthread/boards/dk6_jn5189/enablement"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/aes"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/common"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/ctimer"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/flexcomm"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/jn_flash"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/jn_iocon"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/jn_rng"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/lpc_adc"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/lpc_gpio"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/drivers/sha"
        "$ENV{NXP_K32W061_SDK_ROOT}/platform/utilities/debug_console"
    )
endif()

set(K32W0_COMM_SOURCES
    platform/alarm.c
    platform/diag.c
    platform/entropy.c
    platform/flash.c
    platform/logging.c
    platform/misc.c
    platform/radio.c
    platform/settings_k32w.c
    platform/system.c
    platform/uart.c
    $ENV{NXP_K32W061_SDK_ROOT}/middleware/mbedtls/port/ksdk/aes_alt.c
    $ENV{NXP_K32W061_SDK_ROOT}/middleware/mbedtls/port/ksdk/ksdk_mbedtls.c
)

if (SDK_RELEASE)
    list(APPEND K32W0_COMM_SOURCES
        $ENV{NXP_K32W061_SDK_ROOT}/devices/K32W061/utilities/fsl_assert.c
    )
else()
    list(APPEND K32W0_COMM_SOURCES
        $ENV{NXP_K32W061_SDK_ROOT}/platform/utilities/assert/fsl_assert.c
    )
endif()

set(K32W0_LIBS
    $ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/ieee-802.15.4/lib/libMiniMac.a
    $ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/PDM/Library/libPDM.a
    $ENV{NXP_K32W061_SDK_ROOT}/middleware/wireless/framework/XCVR/lib/libRadio.a
)

set(K32W0_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/openthread/examples/platforms
)

if(OT_BUILD_K32W061)
    include(k32w061/k32w061.cmake)
elseif(OT_BUILD_JN5189)
    include(jn5189/jn5189.cmake)
endif()
