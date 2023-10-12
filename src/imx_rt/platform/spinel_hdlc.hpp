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

#ifndef OT_RT_SPINEL_HDLC_HPP_
#define OT_RT_SPINEL_HDLC_HPP_

#include "ot_platform_common.h"

#include "fsl_component_serial_manager.h"

#include "lib/hdlc/hdlc.hpp"
#include "lib/spinel/multi_frame_buffer.hpp"
#include "lib/spinel/spinel_interface.hpp"
#include "lib/url/url.hpp"

namespace ot {

namespace RT {

typedef uint8_t HdlcSpinelContext;

/**
 * This class defines an HDLC spinel interface to the Radio Co-processor (RCP).
 *
 */
class HdlcInterface : public ot::Spinel::SpinelInterface
{
public:
    /**
     * This constructor initializes the object.
     *
     * @param[in] aRadioUrl  RadioUrl parsed from radio url.
     *
     */
    HdlcInterface(const Url::Url &aRadioUrl);

    /**
     * This destructor deinitializes the object.
     *
     */
    ~HdlcInterface(void);

    /**
     * This method initializes the HDLC interface.
     *
     * @param[in] aCallback         Callback on frame received
     * @param[in] aCallbackContext  Callback context
     * @param[in] aFrameBuffer      A reference to a `RxFrameBuffer` object.
     *
     */
    otError Init(ReceiveFrameCallback aCallback, void *aCallbackContext, RxFrameBuffer &aFrameBuffer);

    /**
     * This method deinitializes the HDLC interface.
     *
     */
    void Deinit(void);

    /**
     * This method encodes and sends a spinel frame to Radio Co-processor (RCP) over the socket.
     *
     * This is blocking call, i.e., if the socket is not writable, this method waits for it to become writable for
     * up to `kMaxWaitTime` interval.
     *
     * @param[in] aFrame     A pointer to buffer containing the spinel frame to send.
     * @param[in] aLength    The length (number of bytes) in the frame.
     *
     * @retval OT_ERROR_NONE     Successfully encoded and sent the spinel frame.
     * @retval OT_ERROR_NO_BUFS  Insufficient buffer space available to encode the frame.
     * @retval OT_ERROR_FAILED   Failed to send due to socket not becoming writable within `kMaxWaitTime`.
     *
     */
    otError SendFrame(const uint8_t *aFrame, uint16_t aLength);

    /**
     * This method waits for receiving part or all of spinel frame within specified timeout.
     *
     * @param[in]  aTimeoutUs  The timeout value in microseconds.
     *
     * @retval OT_ERROR_NONE             Part or all of spinel frame is received.
     * @retval OT_ERROR_RESPONSE_TIMEOUT No spinel frame is received within @p aTimeoutUs.
     *
     */
    otError WaitForFrame(uint64_t aTimeoutUs);

    /**
     * Updates the file descriptor sets with file descriptors used by the radio driver.
     *
     * @param[in,out]   aMainloopContext  A pointer to the mainloop context containing fd_sets.
     *
     */
    void UpdateFdSet(void *aMainloopContext) { OT_UNUSED_VARIABLE(aMainloopContext); }

    /**
     * Returns the bus speed between the host and the radio.
     *
     * @returns   Bus speed in bits/second.
     *
     */
    uint32_t GetBusSpeed(void) const
    {
        static constexpr uint32_t kDefaultSpeed = 1000000;
        return kDefaultSpeed;
    }

    /**
     * This method performs radio driver processing.
     *
     * @param[in]  aInstance  The ot instance
     *
     */
    void Process(const void *aContext);

    /**
     * This method hardware resets the RCP.
     *
     * @retval OT_ERROR_NONE             Successfully reset the RCP.
     * @retval OT_ERROR_NOT_IMPLEMENTED  The hardware reset is not implemented.
     *
     */
    otError HardwareReset(void) { return OT_ERROR_NOT_IMPLEMENTED; }

    /**
     * Returns the RCP interface metrics.
     *
     * @returns The RCP interface metrics.
     *
     */
    const otRcpInterfaceMetrics *GetRcpInterfaceMetrics(void) const { return nullptr; }

private:
    enum
    {
        /* Ring buffer size should be >= to the RCP max TX buffer size value which is 2048 */
        kMaxRingBufferSize = 2048,

    };

    SERIAL_MANAGER_HANDLE_DEFINE(otTransceiverSerialHandle);
    SERIAL_MANAGER_READ_HANDLE_DEFINE(otTransceiverSerialReadHandle);

    ot::Spinel::SpinelInterface::ReceiveFrameCallback mReceiveFrameCallback;
    void                                             *mReceiveFrameContext;
    ot::Spinel::SpinelInterface::RxFrameBuffer       *mReceiveFrameBuffer;

    ot::Spinel::FrameBuffer<ot::Spinel::SpinelInterface::kMaxFrameSize> encoderBuffer;
    ot::Hdlc::Encoder                                                   mHdlcEncoder;
    ot::Hdlc::Decoder                                                   mHdlcDecoder;

    uint8_t  s_ringBuffer[kMaxRingBufferSize];
    void     InitUart(void);
    void     DeinitUart(void);
    uint32_t TryReadAndDecode(bool fullRead);
    otError  Write(const uint8_t *aFrame, uint16_t aLength);

    static void HandleHdlcFrame(void *aContext, otError aError);
    void        HandleHdlcFrame(otError aError);
};

} // namespace RT

} // namespace ot

#endif // OT_RT_SPINEL_HDLC_HPP_
