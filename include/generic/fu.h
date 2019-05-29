#ifndef FU_H_INCLUDED
#define FU_H_INCLUDED

#include <stdint.h>
#include "my_fp.h"

class MotorVoltage
{
public:
   enum minFrqMode
   {
      SETZERO, RAMPDOWN, IGNORE
   };

   static void SetBoost(uint32_t boost);
   static void SetWeakeningFrq(u32fp frq);
   static void SetMinFrqMode(enum minFrqMode val);
   static void SetMaxAmp(uint32_t maxAmp);
   static void SetMinFrq(u32fp frq);
   static void SetMaxFrq(u32fp frq);
   static uint32_t GetAmp(u32fp frq);
   static uint32_t GetAmpPerc(u32fp frq, u32fp perc);

private:
   static void CalcFac();
   static uint32_t boost;
   static u32fp fac;
   static uint32_t maxAmp;
   static u32fp endFrq;
   static u32fp minFrq;
   static u32fp maxFrq;
   static enum minFrqMode minFrqMode;
};

#endif // FU_H_INCLUDED
