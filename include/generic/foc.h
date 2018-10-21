#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#include "my_fp.h"

class FOC
{
   public:
      static void ParkClarke(s32fp il1, s32fp il2, uint16_t angle);
      static void InvParkClarke(s32fp id, s32fp iq, uint16_t angle);
      static void SetDirection(int dir);
      static s32fp id;
      static s32fp iq;
      static uint32_t DutyCycles[3];

   protected:
   private:
};

#endif // FOC_H
