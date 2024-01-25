/*
 *  Copyright (c) 2023, The OpenThread Authors.
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
 *   This file implements OT SHA256 and HMAC SHA256operations for K32W1 secure sub system
 *
 */

#include <string.h>

#include <crypto/sha256.hpp>
#include <openthread/error.h>
#include <openthread/platform/crypto.h>

#include "common/code_utils.hpp"
#include "common/debug.hpp"
#include "common/error.hpp"
#include "core/crypto/storage.hpp"

extern "C" {
#include "SecLib.h"
}

using namespace ot;
using namespace Crypto;

// HMAC implementations
otError otPlatCryptoHmacSha256Init(otCryptoContext *aContext)
{
    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    aContext->mContext = HMAC_SHA256_AllocCtx();

exit:
    return error;
}

otError otPlatCryptoHmacSha256Deinit(otCryptoContext *aContext)
{
    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    HMAC_SHA256_FreeCtx(aContext->mContext);
    aContext->mContext = nullptr;

exit:
    return error;
}

otError otPlatCryptoHmacSha256Start(otCryptoContext *aContext, const otCryptoKey *aKey)
{
    Error            error = kErrorNone;
    const LiteralKey key(*static_cast<const Key *>(aKey));
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    HMAC_SHA256_Init(aContext->mContext, key.GetBytes(), key.GetLength());

exit:
    return error;
}

otError otPlatCryptoHmacSha256Update(otCryptoContext *aContext, const void *aBuf, uint16_t aBufLength)
{
    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    HMAC_SHA256_Update(aContext->mContext, (const uint8_t *)aBuf, aBufLength);

exit:
    return error;
}

otError otPlatCryptoHmacSha256Finish(otCryptoContext *aContext, uint8_t *aBuf, size_t aBufLength)
{
    OT_UNUSED_VARIABLE(aBufLength);

    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    HMAC_SHA256_Finish(aContext->mContext, aBuf);

exit:
    return error;
}

// SHA256 platform implementations
otError otPlatCryptoSha256Init(otCryptoContext *aContext)
{
    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    aContext->mContext = SHA256_AllocCtx();
    SHA256_Init(aContext->mContext);

exit:
    return error;
}

otError otPlatCryptoSha256Deinit(otCryptoContext *aContext)
{
    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    SHA256_FreeCtx(aContext->mContext);
    aContext->mContext = nullptr;

exit:
    return error;
}

otError otPlatCryptoSha256Start(otCryptoContext *aContext)
{
    return kErrorNone;
}

otError otPlatCryptoSha256Update(otCryptoContext *aContext, const void *aBuf, uint16_t aBufLength)
{
    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    SHA256_HashUpdate(aContext->mContext, (const uint8_t *)aBuf, aBufLength);

exit:
    return error;
}

otError otPlatCryptoSha256Finish(otCryptoContext *aContext, uint8_t *aHash, uint16_t aHashSize)
{
    OT_UNUSED_VARIABLE(aHashSize);

    Error error = kErrorNone;
    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    SHA256_HashFinish(aContext->mContext, aHash);

exit:
    return error;
}
