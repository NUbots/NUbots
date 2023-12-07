/*
 * MIT License
 *
 * Copyright (c) 2017 NUbots
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
#ifndef MESSAGE_MESSAGEBASE_HPP
#define MESSAGE_MESSAGEBASE_HPP

#include <nuclear>

namespace message {
    template <typename T>
    class MessageBase : public std::enable_shared_from_this<T> {};
}  // namespace message

namespace NUClear::util::serialise {

    template <typename T>
    struct Serialise<T, std::enable_if_t<std::is_base_of<::message::MessageBase<std::remove_cv_t<T>>, T>::value, T>> {

        using protobuf_type = typename T::protobuf_type;

        static inline std::vector<uint8_t> serialise(const T& in) {

            protobuf_type proto = in;

            std::vector<uint8_t> output(proto.ByteSizeLong());
            proto.SerializeToArray(output.data(), output.size());

            return output;
        }

        [[nodiscard]] static inline T deserialise(const uint8_t* in, const size_t& length) {

            // Make a buffer
            protobuf_type out;

            // Deserialize it
            if (!out.ParseFromArray(in, length)) {
                throw std::runtime_error("Message failed to deserialise.");
            }

            return out;
        }

        [[nodiscard]] static inline T deserialise(const std::vector<uint8_t>& in) {
            return deserialise(in.data(), in.size());
        }

        static inline uint64_t hash() {

            // We have to construct an instance to call the reflection functions
            protobuf_type type;

            // We have to remove the 'protobuf' namespace
            std::string typeName = type.GetTypeName().substr(9);

            // We base the hash on the name of the protocol buffer, removing the protobuf prefix on typeName
            return xxhash64(typeName.c_str(), typeName.size(), 0x4e55436c);
        }
    };

}  // namespace NUClear::util::serialise

#endif  // MESSAGE_MESSAGEBASE_HPP
