/*
 *  Copyright (c) 2021, The OpenThread Authors.
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

#include "spinel_hdlc.hpp"

#include "platform-rt1060.h"

#include <openthread/platform/time.h>

namespace ot {

namespace RT {

HdlcInterface::HdlcInterface(ot::Spinel::SpinelInterface::ReceiveFrameCallback aCallback,
                             void *                                            aCallbackContext,
                             ot::Spinel::SpinelInterface::RxFrameBuffer &      aFrameBuffer)
    : mReceiveFrameCallback(aCallback)
    , mReceiveFrameContext(aCallbackContext)
    , mReceiveFrameBuffer(aFrameBuffer)
    , mHdlcDecoder(aFrameBuffer, HandleHdlcFrame, this)
{
}

HdlcInterface::~HdlcInterface(void)
{
    /* TODO */
}

void HdlcInterface::Init(void)
{
    /* TODO */

    InitUart();
}

void HdlcInterface::Deinit(void)
{
    DeinitUart();

    /* TODO */
}

otError HdlcInterface::SendFrame(const uint8_t *aFrame, uint16_t aLength)
{
    otError                              error = OT_ERROR_NONE;
    ot::Hdlc::FrameBuffer<kMaxFrameSize> encoderBuffer;
    ot::Hdlc::Encoder                    hdlcEncoder(encoderBuffer);

    SuccessOrExit(error = hdlcEncoder.BeginFrame());
    SuccessOrExit(error = hdlcEncoder.Encode(aFrame, aLength));
    SuccessOrExit(error = hdlcEncoder.EndFrame());

    SuccessOrExit(error = Write(encoderBuffer.GetFrame(), encoderBuffer.GetLength()));

exit:
    if (error != OT_ERROR_NONE)
    {
        /* TODO */
    }
    else
    {
        /* TODO */
    }

    return error;
}

void HdlcInterface::Process(const otSysMainloopContext &aMainloop)
{
    OT_UNUSED_VARIABLE(aMainloop);

    /* TODO */
}

void HdlcInterface::Update(otSysMainloopContext &aMainloop)
{
    OT_UNUSED_VARIABLE(aMainloop);

    /* TODO */
}

int HdlcInterface::TryReadAndDecode(void)
{
    /* TODO */

    return 0;
}

otError HdlcInterface::WaitForWritable(void)
{
    /* TODO */

    return OT_ERROR_NONE;
}

otError HdlcInterface::Write(const uint8_t *aFrame, uint16_t aLength)
{
    OT_UNUSED_VARIABLE(aFrame);
    OT_UNUSED_VARIABLE(aLength);

    /* TODO */

    return OT_ERROR_NONE;
}

otError HdlcInterface::WaitForFrame(uint64_t aTimeoutUs)
{
    OT_UNUSED_VARIABLE(aTimeoutUs);

    /* TODO */

    return OT_ERROR_NONE;
}

void HdlcInterface::HandleHdlcFrame(void *aContext, otError aError)
{
    static_cast<HdlcInterface *>(aContext)->HandleHdlcFrame(aError);
}

void HdlcInterface::HandleHdlcFrame(otError aError)
{
    if (aError == OT_ERROR_NONE)
    {
        mReceiveFrameCallback(mReceiveFrameContext);
    }
    else
    {
        mReceiveFrameBuffer.DiscardFrame();
    }
}

void HdlcInterface::InitUart(void)
{
    /* TODO */
}

void HdlcInterface::DeinitUart(void)
{
    /* TODO */
}

} // namespace RT

} // namespace ot
