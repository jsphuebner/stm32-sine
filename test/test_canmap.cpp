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
#include <iostream>
#include <iterator>
#include <memory>

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

template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& o, const std::array<T, N>& arr)
{
    std::copy(arr.cbegin(), arr.cend(), std::ostream_iterator<T>(o, " "));
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
                  << "Expected: " << expected << "\n"
                  << "Actual  : " << canStub->m_data << "\n";
        return false;
    }

    return true;
}

static void send_map_little_endian_byte_in_first_word()
{
    canMap->AddSend(Param::ocurlim, CanId, 0, 8, 1.0, 0);
    Param::SetFloat(Param::ocurlim, 42);

    canMap->SendAll();

    ASSERT(FrameMatches({ 42, 0, 0, 0, 0, 0, 0, 0 }));
}

REGISTER_TEST(CanMapTest, send_map_little_endian_byte_in_first_word);
