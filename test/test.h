#ifndef TEST_H_INCLUDED
#define TEST_H_INCLUDED
#include <iostream>
#include <list>

typedef void (*VoidFunction)();

class UnitTest
{
   public:
      UnitTest(const std::list<VoidFunction>*);
      virtual void TestSetup() {}
      virtual void TestCaseSetup() {}
      const std::list<VoidFunction> GetCases() { return *_cases; }
      void SetTestCaseList(const std::list<VoidFunction>* cases) { _cases = cases; }

   private:
      const std::list<VoidFunction>* _cases;
};

extern int _failedAssertions;


#define REGISTER_TEST(t, ...) static UnitTest* test = new t (new std::list<VoidFunction> { __VA_ARGS__ });

#define STRING(s) #s
#define ASSERT(c) \
   if (c)   \
      std::cout << "Test " << __FILE__ << "::" << __func__ << " passed." << std::endl; \
   else \
   {  \
      std::cout << "Assertion failed: " << STRING(c) << " in " __FILE__ " : " << std::dec << __LINE__ << std::endl;    \
      _failedAssertions++; \
   }

#endif // TEST_H_INCLUDED
