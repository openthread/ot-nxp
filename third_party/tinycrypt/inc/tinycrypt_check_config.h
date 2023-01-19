/**
 * \file tinycrypt_check_config.h
 *
 * \brief Consistency checks for configuration options
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

/*
 * It is recommended to include this file from your config.h
 * in order to catch dependency issues early.
 */

#ifndef TINYCRYPT_CHECK_CONFIG_H
#define TINYCRYPT_CHECK_CONFIG_H

#if defined(MBEDTLS_USE_TINYCRYPT) && defined(MBEDTLS_NO_64BIT_MULTIPLICATION)
#error "MBEDTLS_USE_TINYCRYPT defined, but it cannot be defined with MBEDTLS_NO_64BIT_MULTIPLICATION"
#endif
#if defined(MBEDTLS_USE_TINYCRYPT) && !defined(MBEDTLS_SHA256_C)
#error "MBEDTLS_USE_TINYCRYPT defined, but not MBEDTLS_SHA256_C"
#endif
#if defined(MBEDTLS_OPTIMIZE_TINYCRYPT_ASM) && (!defined(MBEDTLS_HAVE_ASM) || !defined(MBEDTLS_USE_TINYCRYPT))
#error "MBEDTLS_OPTIMIZE_TINYCRYPT_ASM defined, but not all prerequesites"
#endif

#if defined(MBEDTLS_PK_C) && (!defined(MBEDTLS_RSA_C) && !defined(MBEDTLS_ECP_C) && !defined(MBEDTLS_USE_TINYCRYPT))
#error "MBEDTLS_PK_C defined, but not all prerequisites"
#endif

#endif /* TINYCRYPT_CHECK_CONFIG_H */
