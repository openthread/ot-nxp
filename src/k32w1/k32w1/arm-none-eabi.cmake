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

# TOOLCHAIN EXTENSION
if(WIN32)
    set(TOOLCHAIN_EXT ".exe")
else()
    set(TOOLCHAIN_EXT "")
endif()


# TOOLCHAIN_DIR AND NANO LIBRARY
set(TOOLCHAIN_DIR $ENV{ARMGCC_DIR})
string(REGEX REPLACE "\\\\" "/" TOOLCHAIN_DIR "${TOOLCHAIN_DIR}")

if(NOT TOOLCHAIN_DIR)
    message(STATUS "***ARMGCC_DIR is not set, assume toolchain bins are in your PATH***")
    set(TOOLCHAIN_BIN_DIR "")
else()
    message(STATUS "TOOLCHAIN_DIR: " ${TOOLCHAIN_DIR})
    set(TOOLCHAIN_BIN_DIR ${TOOLCHAIN_DIR}/bin/)
endif()

# TARGET_TRIPLET
set(TARGET_TRIPLET "arm-none-eabi")

set(CMAKE_SYSTEM_NAME              Generic)
set(CMAKE_SYSTEM_PROCESSOR         ARM)

set(CMAKE_C_COMPILER ${TOOLCHAIN_BIN_DIR}${TARGET_TRIPLET}-gcc${TOOLCHAIN_EXT})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_BIN_DIR}${TARGET_TRIPLET}-g++${TOOLCHAIN_EXT})
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_BIN_DIR}${TARGET_TRIPLET}-gcc${TOOLCHAIN_EXT})

message(STATUS "CMAKE_C_COMPILER: " ${CMAKE_C_COMPILER})

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

set(CMAKE_OBJCOPY ${TOOLCHAIN_BIN_DIR}${TARGET_TRIPLET}-objcopy CACHE INTERNAL "objcopy tool")
set(CMAKE_OBJDUMP ${TOOLCHAIN_BIN_DIR}${TARGET_TRIPLET}-objdump CACHE INTERNAL "objdump tool")

# set(CMAKE_RANLIB                   arm-none-eabi-ranlib)

execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE COMPILER_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)

set(COMMON_C_FLAGS                 "-mcpu=cortex-m33 -mfloat-abi=hard -mthumb -mfpu=fpv5-sp-d16 -fmessage-length=0 -fdata-sections -ffunction-sections")

if(COMPILER_VERSION VERSION_GREATER_EQUAL 7)
    set(COMMON_C_FLAGS             "${COMMON_C_FLAGS} -Wno-expansion-to-defined")
endif()

set(CMAKE_C_FLAGS_INIT             "${COMMON_C_FLAGS} -std=gnu99")
set(CMAKE_CXX_FLAGS_INIT           "${COMMON_C_FLAGS} -fno-exceptions -fno-rtti")
set(CMAKE_ASM_FLAGS_INIT           "${COMMON_C_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT    "${COMMON_C_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections")

set(CMAKE_C_FLAGS_DEBUG            "-Og -g")
set(CMAKE_CXX_FLAGS_DEBUG          "-Og -g")
set(CMAKE_ASM_FLAGS_DEBUG          "-g")

set(CMAKE_C_FLAGS_RELEASE          "-Os")
set(CMAKE_CXX_FLAGS_RELEASE        "-Os")
set(CMAKE_ASM_FLAGS_RELEASE        "")
