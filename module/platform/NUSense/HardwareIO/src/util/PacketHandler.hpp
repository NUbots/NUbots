/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MODULE_PLATFORM_NUSENSE_PACKETHANDLER
#define MODULE_PLATFORM_NUSENSE_PACKETHANDLER

#include <algorithm>

#include "message/platform/NUSenseData.hpp"

// #include "main.h" // for debugging on GPIO pins

namespace module::platform::NUSense {

    using message::platform::NUSense;


    /// @brief Handles USB protobuf packets.
    class PacketHandler {

    public:
        /// @brief Constructs the packet-handler.
        PacketHandler()
            : pb_length(0)
            , remaining_length(0)
            , is_packet_ready(false)
            , rx_count(0)
            , decode_count(0)
            , missing_count(0) {}

        /// @brief   Handle outgoing bytes from the ring-buffer, parse any packet, and decode it.
        /// @return  Whether or not the packet has been decoded
        bool handle();

        /// @brief Handles the reception of packets and stores them in a buffer.
        ///
        /// This function receives a packet of data and stores it in a circular buffer. If the buffer is full, it
        /// discards the packet. If the packet size exceeds the remaining space at the end of the buffer, it wraps
        /// around and stores the rest of the packet at the beginning of the buffer.
        ///
        /// @param len The length of the incoming packet.
        /// @param buf A pointer to the incoming packet data.
        void receive(const uint32_t& len, uint8_t* buf);

        /// @brief Getter for the received NUSense protobuf message
        /// @return The NUSense protobuf message
        const NUSense& get_nusense_message();

        /// @brief The size of the receive buffer.
        static constexpr uint16_t RX_BUF_SIZE = 2048;

    private:
        /// @brief   Removes bytes from the ring-buffer of a given length, passing from a given
        ///          offset.
        /// @note    This is a helper function; it is not meant to encapsulate anything.
        /// @param   bytes the bytes that would be taken
        /// @param   length the number of bytes to take
        /// @param   offset the number of bytes skipped first
        /// @note    Normally, offset would be either zero for a traditional pop, or 5 to ignore the
        ///          header.
        /// @return  The number of bytes removed.

        uint16_t pop(uint8_t* bytes, uint16_t length, uint16_t offset);

        struct RingBuffer {
            /// @brief  the data of the buffer:
            uint8_t data[RX_BUF_SIZE]{};
            /// @brief  the front of the 'queue' where bytes are read or popped,
            /// @note   This is inclusive of the first byte.
            /// @note   "I have been waiting for so long; I am nearly at the front of the queue."
            uint16_t front = 0;
            /// @brief  the back of the 'queue' where bytes are added or pushed,
            /// @note   This is exclusive of the last byte.
            /// @note   "That rude man just cut in line; he should go at the back of the queue."
            uint16_t back = 0;
            /// @brief  the number of bytes in the ring-buffer,
            uint16_t size = 0;
        } rx_buffer{};

        /// @brief The NUSense protobuf message received from the NUSense.
        NUSense nusense{};

        /// @brief The buffer for the protobuf payload to be decoded.
        char pb_packets[RX_BUF_SIZE]{};

        /// @brief The length of the protobuf packet.
        uint16_t pb_length = 0;

        /// @brief The remaining length of the protobuf packet to be gathered by the lower-level
        ///        firmware, CDC_Receive_HS.
        uint16_t remaining_length = 0;

        /// @brief Whether a complete protobuf packet has been gathered to be decoded.
        bool is_packet_ready = false;

        /// @brief Debugging count for the number of received chunks of bytes, i.e. everytime when
        ///        rx_flag is set and cleared.
        uint16_t rx_count = 0;

        /// @brief Debugging count for the number of packets decoded.
        uint16_t decode_count = 0;

        /// @brief  Debugging count of packets that had missing targets, i.e. not twenty.
        uint16_t missing_count = 0;

        /// @brief Debugging variable of the count of true packets based on incremental ID, purely
        ///        for testing a specific version of code.
        uint16_t near_id = 0;
    };

}  // namespace module::platform::NUSense

#endif  // MODULE_PLATFORM_NUSENSE_PACKETHANDLER
