/**
 * \file tinycrypt_util.h
 *
 * \brief Common and shared functions used by multiple modules in the Mbed TLS
 *        library.
 */
/*
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#ifndef TINYCRYPT_PLATFORM_UTIL_H
#define TINYCRYPT_PLATFORM_UTIL_H

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <stddef.h>
#if defined(MBEDTLS_HAVE_TIME_DATE)
#include <time.h>
#include "mbedtls/platform_time.h"
#endif /* MBEDTLS_HAVE_TIME_DATE */

#ifdef __cplusplus
extern "C" {
#endif

#define MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED -0x0070 /**< Hardware accelerator failed */
#define MBEDTLS_ERR_PLATFORM_FEATURE_UNSUPPORTED                        \
    -0x0072 /**< The requested feature is not supported by the platform \
             */
#define MBEDTLS_ERR_PLATFORM_FAULT_DETECTED                                                                           \
    -0x0071 /**< A hardware fault was detected in a critical path. As a security precaution this should be treated as \
               a potential physical attack */
#define MBEDTLS_ERR_PLATFORM_ALLOC_FAILED -0x0076 /**< Memory allocation failed */

#if defined(MBEDTLS_USE_TINYCRYPT)
/**
 * \brief       Securely zeroize a buffer
 *
 *              The function is meant to wipe the data contained in a buffer so
 *              that it can no longer be recovered even if the program memory
 *              is later compromised. Call this function on sensitive data
 *              stored on the stack before returning from a function, and on
 *              sensitive data stored on the heap before freeing the heap
 *              object.
 *
 *              It is extremely difficult to guarantee that calls to
 *              tinycrypt_platform_zeroize() are not removed by aggressive
 *              compiler optimizations in a portable way. For this reason, Mbed
 *              TLS provides the configuration option
 *              MBEDTLS_PLATFORM_ZEROIZE_ALT, which allows users to configure
 *              tinycrypt_platform_zeroize() to use a suitable implementation for
 *              their platform and needs
 *
 * \param buf   Buffer to be zeroized
 * \param len   Length of the buffer in bytes
 *
 * \return      The value of \p buf if the operation was successful.
 * \return      NULL if a potential FI attack was detected or input parameters
 *              are not valid.
 */
void *tinycrypt_platform_zeroize(void *buf, size_t len);

/**
 * \brief       Secure memset
 *
 *              This is a constant-time version of memset(). The buffer is
 *              initialised with random data and the order is also randomised
 *              using the RNG in order to further harden against side-channel
 *              attacks.
 *
 * \param ptr   Buffer to be set.
 * \param value Value to be used when setting the buffer.
 * \param num   The length of the buffer in bytes.
 *
 * \return      The value of \p ptr if the operation was successful.
 * \return      NULL if a potential FI attack was detected.
 */
void *tinycrypt_platform_memset(void *ptr, int value, size_t num);

/**
 * \brief       Secure memcpy
 *
 *              This is a constant-time version of memcpy(). The buffer is
 *              initialised with random data and the order is also randomised
 *              using the RNG in order to further harden against side-channel
 *              attacks.
 *
 * \param dst   Destination buffer where the data is being copied to.
 * \param src   Source buffer where the data is being copied from.
 * \param num   The length of the buffers in bytes.
 *
 * \return      The value of \p dst.
 * \return      NULL if a potential FI attack was detected.
 */
void *tinycrypt_platform_memcpy(void *dst, const void *src, size_t num);

/**
 * \brief       Secure memmove
 *
 *              This is a constant-time version of memmove(). It is based on
 *              the double use of the tinycrypt_platform_memcpy() function secured
 *              against side-channel attacks.
 *
 * \param dst   Destination buffer where the data is being moved to.
 * \param src   Source buffer where the data is being moved from.
 * \param num   The length of the buffers in bytes.
 *
 * \return      0 if the operation was successful
 * \return      #MBEDTLS_ERR_PLATFORM_ALLOC_FAILED if a memory allocation failed
 */
int tinycrypt_platform_memmove(void *dst, const void *src, size_t num);

#if !defined(MBEDTLS_DEPRECATED_REMOVED)
#if defined(MBEDTLS_DEPRECATED_WARNING)
#define MBEDTLS_DEPRECATED __attribute__((deprecated))
#else
#define MBEDTLS_DEPRECATED
#endif

/**
 * \brief       Secure memcmp
 *
 *              This is a constant-time version of memcmp(), but without checking
 *              if the bytes are greater or lower. The order is also randomised
 *              using the RNG in order to further harden against side-channel attacks.
 *
 * \param buf1  First buffer to compare.
 * \param buf2  Second buffer to compare against.
 * \param num   The length of the buffers in bytes.
 *
 * \deprecated  Superseded by tinycrypt_platform_memequal(), and is only an alias to it.
 *
 * \return      0 if the buffers were equal or an unspecified non-zero value
 *              otherwise.
 */
int tinycrypt_platform_memcmp(const void *buf1, const void *buf2, size_t num);

#endif
/**
 * \brief       Secure check if the buffers have the same data.
 *
 *              This is a constant-time version of memcmp(), but without checking
 *              if the bytes are greater or lower. The order is also randomised
 *              using the RNG in order to further harden against side-channel attacks.
 *
 * \param buf1  First buffer to compare.
 * \param buf2  Second buffer to compare against.
 * \param num   The length of the buffers in bytes.
 *
 * \return      0 if the buffers were equal or an unspecified non-zero value
 *              otherwise.
 */
int tinycrypt_platform_memequal(const void *buf1, const void *buf2, size_t num);

/**
 * \brief       RNG-function for getting a random 32-bit integer.
 *
 * \return      The generated random number.
 */
uint32_t tinycrypt_platform_random_uint32(void);

/**
 * \brief       RNG-function for getting a random in given range.
 *
 *              This function is meant to provide a global RNG to be used
 *              throughout Mbed TLS for hardening the library. It is used
 *              for generating a random delay, random data or random offset
 *              for utility functions. It is not meant to be a
 *              cryptographically secure RNG, but provide an RNG for utility
 *              functions.
 *
 * \param num   Max-value for the generated random number, exclusive.
 *              Must be greater than zero, otherwise an undefined behavior
 *              will occur on "num % 0".
 *              The generated number will be on range [0, num).
 *
 * \return      The generated random number.
 */
uint32_t tinycrypt_platform_random_in_range(uint32_t num);

/**
 * \brief       Random delay function.
 *
 *              Function implements a random delay by incrementing a local
 *              variable randomized number of times (busy-looping).
 *
 *              Duration of the delay is random as number of variable increments
 *              is randomized.
 *
 * \note        This function works only if the MBEDTLS_FI_COUNTERMEASURES flag
 *              is defined in the configuration. Otherwise, the function does
 *              nothing.
 */
void tinycrypt_platform_random_delay(void);

/**
 * \brief       RNG-function for getting a random buffer.
 *
 * \param buf   Buffer for random data
 * \param len   Length of the buffer in bytes
 *
 */
void tinycrypt_platform_random_buf(uint8_t *buf, size_t len);

#endif

#ifdef __cplusplus
}
#endif

#endif /* TINYCRYPT_PLATFORM_UTIL_H */
