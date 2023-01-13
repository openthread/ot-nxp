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
 *   This file implements ECDSA signing using TinyCrypt library.
 */

#ifdef MBEDTLS_USE_TINYCRYPT

#include <string.h>

#include <mbedtls/pk.h>
#include <mbedtls/version.h>

#include <crypto/tinycrypt/inc/ecc.h>
#include <crypto/tinycrypt/inc/ecc_dh.h>
#include <crypto/tinycrypt/inc/ecc_dsa.h>

#include <crypto/ecdsa.hpp>
#include <crypto/mbedtls.hpp>
#include <crypto/sha256.hpp>
#include <openthread/error.h>
#include <openthread/platform/crypto.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/random.hpp"

using namespace ot;
using namespace Crypto;

typedef struct
{
    uint8_t private_key[NUM_ECC_BYTES];
    uint8_t public_key[2 * NUM_ECC_BYTES];
} uecc_keypair;

otError otPlatCryptoEcdsaGenerateKey(otPlatCryptoEcdsaKeyPair *aKeyPair)
{
    uecc_keypair keypair;
    int          ret;

    ret = uECC_make_key(keypair.public_key, keypair.private_key);
    VerifyOrExit(ret == UECC_SUCCESS);

    memcpy(aKeyPair->mDerBytes, keypair.public_key, 2 * NUM_ECC_BYTES);
    memcpy(aKeyPair->mDerBytes + (2 * NUM_ECC_BYTES), keypair.private_key, NUM_ECC_BYTES);

    aKeyPair->mDerLength = static_cast<uint8_t>(3 * NUM_ECC_BYTES);

exit:
    return (ret >= 0) ? kErrorNone : MbedTls::MapError(ret);
}

otError otPlatCryptoEcdsaGetPublicKey(const otPlatCryptoEcdsaKeyPair *aKeyPair, otPlatCryptoEcdsaPublicKey *aPublicKey)
{
    memcpy(aPublicKey->m8, aKeyPair->mDerBytes, 2 * NUM_ECC_BYTES);

exit:
    return kErrorNone;
}

otError otPlatCryptoEcdsaSign(const otPlatCryptoEcdsaKeyPair *aKeyPair,
                              const otPlatCryptoSha256Hash   *aHash,
                              otPlatCryptoEcdsaSignature     *aSignature)
{
    otError      error = kErrorNone;
    uecc_keypair keypair;
    int          ret;

    memcpy(keypair.private_key, aKeyPair->mDerBytes + (2 * NUM_ECC_BYTES), NUM_ECC_BYTES);

    ret = uECC_sign(keypair.private_key, aHash->m8, Sha256::Hash::kSize, aSignature->m8);
    VerifyOrExit(ret == UECC_SUCCESS, error = MbedTls::MapError(ret));

exit:

    return error;
}

otError otPlatCryptoEcdsaVerify(const otPlatCryptoEcdsaPublicKey *aPublicKey,
                                const otPlatCryptoSha256Hash     *aHash,
                                const otPlatCryptoEcdsaSignature *aSignature)
{
    otError error = kErrorNone;
    int     ret;

    ret = uECC_verify(aPublicKey->m8, aHash->m8, Sha256::Hash::kSize, aSignature->m8);
    VerifyOrExit(ret == UECC_SUCCESS, error = kErrorSecurity);

exit:
    return error;
}

#endif // MBEDTLS_USE_TINYCRYPT
