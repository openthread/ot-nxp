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
 *   This file implements OT ECDSA operations for K32W1 secure sub system
 *
 */

#include <string.h>

#include <crypto/ecdsa.hpp>
#include <crypto/sha256.hpp>
#include <openthread/error.h>
#include <openthread/platform/crypto.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"

#include "sss_crypto.h"

using namespace ot;
using namespace Crypto;

#define P256_PRV_KEY_LEN_BYTES Ecdsa::P256::kMpiSize
#define P256_PRV_KEY_LEN_BITS Ecdsa::P256::kFieldBitLength

otError otPlatCryptoEcdsaGenerateKey(otPlatCryptoEcdsaKeyPair *aKeyPair)
{
    otError           error    = kErrorNone;
    size_t            blobSize = 3 * P256_PRV_KEY_LEN_BYTES + 24;
    sss_sscp_object_t keypair;

    // In case of fail cannot free key until it is actually allocated
    VerifyOrExit(sss_sscp_key_object_init(&keypair, &g_keyStore) == kStatus_SSS_Success, return kErrorSecurity);

    VerifyOrExit(sss_sscp_key_object_allocate_handle(&keypair, 0x0u, kSSS_KeyPart_Pair, kSSS_CipherType_EC_NIST_P,
                                                     3 * P256_PRV_KEY_LEN_BYTES,
                                                     SSS_KEYPROP_OPERATION_ASYM) == kStatus_SSS_Success,
                 return kErrorSecurity);

    // After this point the key cand be freed using SSS_KEY_OBJ_FREE
    VerifyOrExit(SSS_ECP_GENERATE_KEY(&keypair, P256_PRV_KEY_LEN_BITS) == kStatus_SSS_Success, error = kErrorSecurity);

    VerifyOrExit(sss_sscp_key_store_export_key(&g_keyStore, &keypair, aKeyPair->mDerBytes, &blobSize,
                                               kSSS_blobType_ELKE_blob) == kStatus_SSS_Success,
                 error = kErrorSecurity);

    aKeyPair->mDerLength = blobSize;

exit:
    (void)SSS_KEY_OBJ_FREE(&keypair);
    return error;
}

otError otPlatCryptoEcdsaGetPublicKey(const otPlatCryptoEcdsaKeyPair *aKeyPair, otPlatCryptoEcdsaPublicKey *aPublicKey)
{
    otError           error = kErrorNone;
    sss_sscp_object_t keypair;
    size_t            keyBitsLen = P256_PRV_KEY_LEN_BITS;
    size_t            keySize    = SSS_ECP_KEY_SZ(P256_PRV_KEY_LEN_BYTES);

    // In case of fail cannot free key until it is actually allocated
    VerifyOrExit(sss_sscp_key_object_init(&keypair, &g_keyStore) == kStatus_SSS_Success, return kErrorSecurity);

    VerifyOrExit(sss_sscp_key_object_allocate_handle(&keypair, 0x0u, kSSS_KeyPart_Pair, kSSS_CipherType_EC_NIST_P,
                                                     3 * P256_PRV_KEY_LEN_BYTES,
                                                     SSS_KEYPROP_OPERATION_ASYM) == kStatus_SSS_Success,
                 return kErrorSecurity);

    // After this point the key cand be freed using SSS_KEY_OBJ_FREE
    VerifyOrExit(sss_sscp_key_store_import_key(&g_keyStore, &keypair, aKeyPair->mDerBytes, aKeyPair->mDerLength,
                                               keyBitsLen, kSSS_blobType_ELKE_blob) == kStatus_SSS_Success,
                 error = kErrorSecurity);

    // Extract public key
    VerifyOrExit(SSS_KEY_STORE_GET_PUBKEY(&keypair, aPublicKey->m8, &keySize, &keyBitsLen) == kStatus_SSS_Success,
                 error = kErrorSecurity);

exit:
    (void)SSS_KEY_OBJ_FREE(&keypair);
    return error;
}

otError otPlatCryptoEcdsaSign(const otPlatCryptoEcdsaKeyPair *aKeyPair,
                              const otPlatCryptoSha256Hash   *aHash,
                              otPlatCryptoEcdsaSignature     *aSignature)
{
    otError               error = kErrorNone;
    sss_sscp_asymmetric_t asyc;
    bool                  bFreeAsyncCtx = false;
    sss_sscp_object_t     keypair;
    size_t                signatureSize = OT_CRYPTO_ECDSA_SIGNATURE_SIZE;

    // In case of fail cannot free key until it is actually allocated
    VerifyOrExit(sss_sscp_key_object_init(&keypair, &g_keyStore) == kStatus_SSS_Success, return kErrorSecurity);

    VerifyOrExit(sss_sscp_key_object_allocate_handle(&keypair, 0x0u, kSSS_KeyPart_Pair, kSSS_CipherType_EC_NIST_P,
                                                     3 * P256_PRV_KEY_LEN_BYTES,
                                                     SSS_KEYPROP_OPERATION_ASYM) == kStatus_SSS_Success,
                 return kErrorSecurity);

    // After this point the key cand be freed using SSS_KEY_OBJ_FREE
    VerifyOrExit(sss_sscp_key_store_import_key(&g_keyStore, &keypair, aKeyPair->mDerBytes, aKeyPair->mDerLength,
                                               P256_PRV_KEY_LEN_BITS, kSSS_blobType_ELKE_blob) == kStatus_SSS_Success,
                 error = kErrorSecurity);

    VerifyOrExit(sss_sscp_asymmetric_context_init(&asyc, &g_sssSession, &keypair, kAlgorithm_SSS_ECDSA_SHA256,
                                                  kMode_SSS_Sign) == kStatus_SSS_Success,
                 error = kErrorSecurity);

    bFreeAsyncCtx = true;
    VerifyOrExit(sss_sscp_asymmetric_sign_digest(&asyc, (uint8_t *)aHash->m8, Sha256::Hash::kSize, aSignature->m8,
                                                 &signatureSize) == kStatus_SSS_Success,
                 error = kErrorSecurity);

exit:

    if (bFreeAsyncCtx)
    {
        /* Need to be very carefull, if we try to free something that is not initialized with success we will get a hw
         * fault */
        (void)sss_sscp_asymmetric_context_free(&asyc);
    }
    (void)SSS_KEY_OBJ_FREE(&keypair);

    return error;
}

otError otPlatCryptoEcdsaVerify(const otPlatCryptoEcdsaPublicKey *aPublicKey,
                                const otPlatCryptoSha256Hash     *aHash,
                                const otPlatCryptoEcdsaSignature *aSignature)
{
    otError error         = kErrorNone;
    bool    bFreeAsyncCtx = false;

    sss_sscp_object_t     ecdsaPublic;
    sss_sscp_asymmetric_t asyc;
    size_t                keySize = SSS_ECP_KEY_SZ(P256_PRV_KEY_LEN_BYTES);

    // In case of fail cannot free key until it is actually allocated
    VerifyOrExit(sss_sscp_key_object_init(&ecdsaPublic, &g_keyStore) == kStatus_SSS_Success, return kErrorSecurity);

    VerifyOrExit(sss_sscp_key_object_allocate_handle(&ecdsaPublic, 0u, kSSS_KeyPart_Public, kSSS_CipherType_EC_NIST_P,
                                                     keySize, SSS_KEYPROP_OPERATION_ASYM) == kStatus_SSS_Success,
                 return kErrorSecurity);

    // After this point the key cand be freed using SSS_KEY_OBJ_FREE
    VerifyOrExit(SSS_KEY_STORE_SET_KEY(&ecdsaPublic, aPublicKey->m8, keySize, keySize * 8,
                                       (uint32_t)kSSS_KeyPart_Public) == kStatus_SSS_Success,
                 error = kErrorSecurity);

    VerifyOrExit(sss_sscp_asymmetric_context_init(&asyc, &g_sssSession, &ecdsaPublic, kAlgorithm_SSS_ECDSA_SHA256,
                                                  kMode_SSS_Verify) == kStatus_SSS_Success,
                 error = kErrorSecurity);

    bFreeAsyncCtx = true;
    VerifyOrExit(sss_sscp_asymmetric_verify_digest(&asyc, (uint8_t *)aHash->m8, Sha256::Hash::kSize,
                                                   (uint8_t *)aSignature->m8,
                                                   OT_CRYPTO_ECDSA_SIGNATURE_SIZE) == kStatus_SSS_Success,
                 error = kErrorSecurity);
exit:

    if (bFreeAsyncCtx)
    {
        /* Need to be very carefull, if we try to free something that is not initialized with success we will get a hw
         * fault */
        (void)sss_sscp_asymmetric_context_free(&asyc);
    }
    (void)SSS_KEY_OBJ_FREE(&ecdsaPublic);

    return error;
}
