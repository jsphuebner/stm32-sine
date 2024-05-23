/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2024 David J. Fiddes <D.J@fiddes.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "canhardware.h"
#include "canmap.h"
#include "params.h"
#include "stub_canhardware.h"
#include "test.h"

#include <array>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>

class CanMapTest : public UnitTest
{
public:
    explicit CanMapTest(const std::list<VoidFunction>* cases) : UnitTest(cases)
    {
    }
    virtual void TestCaseSetup();
};

std::unique_ptr<CanStub> canStub;
std::unique_ptr<CanMap>  canMap;

void CanMapTest::TestCaseSetup()
{
    canStub = std::make_unique<CanStub>();
    canMap = std::make_unique<CanMap>(canStub.get(), false);
    Param::LoadDefaults();
}

const uint32_t CanId = 0x123;

std::ostream& operator<<(std::ostream& o, const std::array<uint8_t, 8>& data)
{
    for (const auto& element : data)
    {
        o << "0x" << std::setfill('0') << std::setw(2) << std::hex
          << (int)element << " ";
    }
    return o;
}

bool FrameMatches(const std::array<uint8_t, 8>& expected)
{
    if (canStub->m_canId != CanId)
    {
        std::cout << "CAN ID doesn't match. Expected: " << CanId
                  << " Actual: " << canStub->m_canId << "\n";
        return false;
    }

    if (canStub->m_len != 8)
    {
        std::cout << "CAN frame length doesn't match. Expected: 8 "
                  << "Actual: " << canStub->m_len << "\n";
        return false;
    }

    if (canStub->m_data != expected)
    {
        std::cout << "CAN frame data doesn't match.\n"
                  << "Actual  : " << canStub->m_data << "\n"
                  << "Expected: " << expected << "\n";

        return false;
    }

    return true;
}

static void send_map_little_endian_byte_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 0, 8, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 0x42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0x42, 0, 0, 0, 0, 0, 0, 0 }));
}

static void send_map_little_endian_16_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 0, 16, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 0x42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0x42, 0, 0, 0, 0, 0, 0, 0 }));
}

static void send_map_little_endian_32_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 0, 32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 0x42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0x42, 0, 0, 0, 0, 0, 0, 0 }));
}

static void send_map_little_endian_32_bit_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 32, 32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 0x42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0x42, 0, 0, 0 }));
}

static void send_map_little_endian_negative_number_16_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 0, 16, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0xfe, 0xff, 0, 0, 0, 0, 0, 0 }));
}

static void send_map_little_endian_negative_number_24_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 0, 24, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0xfe, 0xff, 0xff, 0, 0, 0, 0, 0 }));
}

static void send_map_little_endian_negative_number_32_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 0, 32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0xfe, 0xff, 0xff, 0xff, 0, 0, 0, 0 }));
}

static void send_map_little_endian_negative_number_32_bit_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 32, 32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0xfe, 0xff, 0xff, 0xff }));
}

static void send_map_little_endian_negative_number_32_bit_spanning_both_words()
{
    canMap->AddSend(Param::ocurlim, CanId, 16, 32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0xfe, 0xff, 0xff, 0xff, 0, 0 }));
}

static void send_map_little_endian_negative_number_32_bit_mostly_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 8, 32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0xfe, 0xff, 0xff, 0xff, 0, 0, 0 }));
}

static void
send_map_little_endian_negative_number_32_bit_mostly_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 24, 32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0xfe, 0xff, 0xff, 0xff, 0 }));
}

static void send_map_little_endian_negative_number_16_bit_at_end_of_frame()
{
    canMap->AddSend(Param::ocurlim, CanId, 48, 16, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0, 0, 0xfe, 0xff }));
}

static void send_map_big_endian_byte_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 7, -8, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 0x42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0x42, 0, 0, 0, 0, 0, 0, 0 }));
}

static void send_map_big_endian_16_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 23, -16, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 0x42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0x42, 0, 0, 0, 0, 0 }));
}

static void send_map_big_endian_32_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 31, -32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 0x42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0x42, 0, 0, 0, 0 }));
}

static void send_map_big_endian_negative_byte_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 7, -8, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0xfe, 0, 0, 0, 0, 0, 0, 0 }));
}

static void send_map_big_endian_negative_16_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 23, -16, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0xff, 0xfe, 0, 0, 0, 0, 0 }));
}

static void send_map_big_endian_negative_24_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 23, -24, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0xff, 0xff, 0xfe, 0, 0, 0, 0, 0 }));
}

static void send_map_big_endian_negative_32_bit_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 31, -32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0xff, 0xff, 0xff, 0xfe, 0, 0, 0, 0 }));
}

static void send_map_big_endian_negative_byte_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 39, -8, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0xfe, 0, 0, 0 }));
}

static void send_map_big_endian_negative_16_bit_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 47, -16, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0xff, 0xfe, 0, 0 }));
}

static void send_map_big_endian_negative_24_bit_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 55, -24, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0xff, 0xff, 0xfe, 0 }));
}

static void send_map_big_endian_negative_32_bit_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 63, -32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xfe }));
}

static void send_map_big_endian_negative_number_24_bit_at_end_of_frame()
{
    canMap->AddSend(Param::ocurlim, CanId, 63, -24, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0, 0, 0xff, 0xff, 0xfe }));
}

static void send_map_big_endian_negative_16_bit_spanning_both_words()
{
    canMap->AddSend(Param::ocurlim, CanId, 39, -16, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0xff, 0xfe, 0, 0, 0 }));
}

static void send_map_big_endian_negative_32_bit_spanning_both_words()
{
    canMap->AddSend(Param::ocurlim, CanId, 47, -32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0xff, 0xff, 0xff, 0xfe, 0, 0 }));
}

static void send_map_big_endian_negative_32_bit_mostly_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 39, -32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0xff, 0xff, 0xff, 0xfe, 0, 0, 0 }));
}

static void send_map_big_endian_negative_32_bit_mostly_in_second_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 55, -32, 1.0, 0);
    Param::SetFloat(Param::ocurlim, -2);

    canMap->SendAll();

    ASSERT(FrameMatches({ 0, 0, 0, 0xff, 0xff, 0xff, 0xfe, 0 }));
}

REGISTER_TEST(
    CanMapTest,
    send_map_little_endian_byte_in_first_word,
    send_map_little_endian_16_bit_in_first_word,
    send_map_little_endian_32_bit_in_first_word,
    send_map_little_endian_32_bit_in_second_word,
    send_map_little_endian_negative_number_16_bit_in_first_word,
    send_map_little_endian_negative_number_24_bit_in_first_word,
    send_map_little_endian_negative_number_32_bit_in_first_word,
    send_map_little_endian_negative_number_32_bit_in_second_word,
    send_map_little_endian_negative_number_32_bit_spanning_both_words,
    send_map_little_endian_negative_number_32_bit_mostly_in_first_word,
    send_map_little_endian_negative_number_32_bit_mostly_in_second_word,
    send_map_little_endian_negative_number_16_bit_at_end_of_frame,
    send_map_big_endian_byte_in_first_word,
    send_map_big_endian_16_bit_in_first_word,
    send_map_big_endian_32_bit_in_first_word,
    send_map_big_endian_negative_byte_in_first_word,
    send_map_big_endian_negative_16_bit_in_first_word,
    send_map_big_endian_negative_24_bit_in_first_word,
    send_map_big_endian_negative_32_bit_in_first_word,
    send_map_big_endian_negative_byte_in_second_word,
    send_map_big_endian_negative_16_bit_in_second_word,
    send_map_big_endian_negative_24_bit_in_second_word,
    send_map_big_endian_negative_32_bit_in_second_word,
    send_map_big_endian_negative_number_24_bit_at_end_of_frame,
    send_map_big_endian_negative_16_bit_spanning_both_words,
    send_map_big_endian_negative_32_bit_spanning_both_words,
    send_map_big_endian_negative_32_bit_mostly_in_first_word,
    send_map_big_endian_negative_32_bit_mostly_in_second_word);
