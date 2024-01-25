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
 *   This file implements OT AES-ECB for K32W1 secure sub system
 *
 */

#include <string.h>

#include <crypto/aes_ecb.hpp>
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

// AES  Implementation
otError otPlatCryptoAesInit(otCryptoContext *aContext)
{
    OT_UNUSED_VARIABLE(aContext);

    return kErrorNone;
}

otError otPlatCryptoAesSetKey(otCryptoContext *aContext, const otCryptoKey *aKey)
{
    Error            error = kErrorNone;
    const LiteralKey key(*static_cast<const Key *>(aKey));

    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);
    VerifyOrExit(aContext->mContextSize >= key.GetLength(), error = kErrorFailed);

    memcpy(aContext, key.GetBytes(), key.GetLength());

exit:
    return error;
}

otError otPlatCryptoAesEncrypt(otCryptoContext *aContext, const uint8_t *aInput, uint8_t *aOutput)
{
    Error error = kErrorNone;

    VerifyOrExit(aContext != nullptr, error = kErrorInvalidArgs);

    AES_128_Encrypt(aInput, (uint8_t *)aContext, aOutput);

exit:
    return error;
}

otError otPlatCryptoAesFree(otCryptoContext *aContext)
{
    OT_UNUSED_VARIABLE(aContext);

    return kErrorNone;
}
