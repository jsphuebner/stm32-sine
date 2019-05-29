#include "fu.h"

#define FRQ_DRT_STR FP_FROMINT(20)

uint32_t MotorVoltage::boost;
u32fp MotorVoltage::fac;
uint32_t MotorVoltage::maxAmp;
u32fp MotorVoltage::endFrq = 1; //avoid division by 0 when not set
u32fp MotorVoltage::minFrq;
u32fp MotorVoltage::maxFrq;
enum MotorVoltage::minFrqMode MotorVoltage::minFrqMode = SETZERO;

/** Set 0 Hz boost to overcome winding resistance */
void MotorVoltage::SetBoost(uint32_t boost /**< amplitude in digit */)
{
   MotorVoltage::boost = boost;
   CalcFac();
}

/** Set frequency where the full amplitude is to be provided */
void MotorVoltage::SetWeakeningFrq(u32fp frq)
{
   endFrq = frq;
   CalcFac();
}

void MotorVoltage::SetMinFrqMode(enum minFrqMode val)
{
   minFrqMode = val;
}

/** Get amplitude for a given frequency */
uint32_t MotorVoltage::GetAmp(u32fp frq)
{
   return MotorVoltage::GetAmpPerc(frq, FP_FROMINT(100));
}

/** Get amplitude for given frequency multiplied with given percentage */
uint32_t MotorVoltage::GetAmpPerc(u32fp frq, u32fp perc)
{
   uint32_t amp = FP_MUL(perc, (FP_TOINT(FP_MUL(fac, frq)) + boost)) / 100;
   if (frq < minFrq)
   {
      switch (minFrqMode)
      {
      case SETZERO:
         amp = 0;
         break;
      case RAMPDOWN:
         amp = FP_MUL(perc, FP_TOINT(FP_MUL(fac, frq))) / 100;
         break;
      case IGNORE:
         break;
      }
   }
   if (amp > maxAmp)
   {
      amp = maxAmp;
   }
   if ((s32fp)frq > (s32fp)(maxFrq - FRQ_DRT_STR))
   {
      s32fp diff = maxFrq - frq;
      diff = diff < 0 ? 0 : diff;
      amp = FP_TOINT(FP_MUL(FP_FROMINT(amp), FP_DIV(diff, FRQ_DRT_STR)));
   }
   return amp;
}

void MotorVoltage::SetMaxAmp(uint32_t maxAmp)
{
   MotorVoltage::maxAmp = maxAmp;
   CalcFac();
}

void MotorVoltage::SetMinFrq(u32fp frq)
{
   minFrq = frq;
}

void MotorVoltage::SetMaxFrq(u32fp frq)
{
   maxFrq = frq;
}

/** Calculate slope of u/f */
void MotorVoltage::CalcFac()
{
   fac = FP_DIV(FP_FROMINT(maxAmp - boost), endFrq);
}
