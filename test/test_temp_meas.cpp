#include "temp_meas.h"
#include "test.h"

class TempMeasTest: public UnitTest
{
   public:
      TempMeasTest(const std::list<VoidFunction>* cases): UnitTest(cases) {}
};

static void TestTesla52kExtendedRange()
{
   ASSERT(TempMeas::Lookup(358, TempMeas::TEMP_TESLA_52K) == 100);
   ASSERT(TempMeas::Lookup(272, TempMeas::TEMP_TESLA_52K) == 110);
   ASSERT(TempMeas::Lookup(48, TempMeas::TEMP_TESLA_52K) == 180);
   ASSERT(TempMeas::Lookup(40, TempMeas::TEMP_TESLA_52K) == 180);
}

REGISTER_TEST(TempMeasTest, TestTesla52kExtendedRange);
