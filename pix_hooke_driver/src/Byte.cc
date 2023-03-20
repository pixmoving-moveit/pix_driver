// Copyright 2023 Pixmoving, Inc. 
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pix_hooke_driver/Byte.hpp>

Byte::Byte(std::uint8_t bytes)
{
    byte_t = bytes;
}

Byte::~Byte()
{
}

int32_t Byte::get_byte(uint start, uint len)
{
    uint8_t new_byte;
    new_byte = byte_t << (8-start-len);
    new_byte = new_byte >> (8-len);
    return static_cast<int32_t>(new_byte);
}

uint8_t Byte::return_byte_t()
{
    return byte_t;
}

void Byte::set_value(uint8_t value, uint start, uint len)
{
    value <<= (8-len);
    value >>= (8-start-len);
    byte_t += value;
}
