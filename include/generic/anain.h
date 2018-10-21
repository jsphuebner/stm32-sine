#ifndef ANAIO_H_INCLUDED
#define ANAIO_H_INCLUDED

#include <stdint.h>
#include "anain_prj.h"


class AnaIn
{
public:
   #define ANA_IN_ENTRY(name, port, pin) name,
   enum AnaIns
   {
       ANA_IN_LIST
       ANA_IN_COUNT
   };
   #undef ANA_IN_ENTRY

   struct AnaInfo
   {
      uint32_t port;
      uint16_t pin;
   };

   static void Init(AnaInfo ins[]);
   static uint16_t Get(AnaIn::AnaIns);

private:


   static const AnaInfo ins[];
   static uint16_t values[];

   static uint8_t AdcChFromPort(uint32_t command_port, int command_bit);
   static int median3(int a, int b, int c);
};

#define ANA_IN_ENTRY(name, port, pin) { port, pin },
/** Usage: AnaIn::AnaInfo analogInputs[] = ANA_IN_ARRAY;
 * AnaIn::Init(analogInputs); */
#define ANA_IN_ARRAY { ANA_IN_LIST }


#endif // ANAIO_H_INCLUDED
