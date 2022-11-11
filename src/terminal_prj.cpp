/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/memorymap.h>
#include "hwdefs.h"
#include "terminal.h"
#include "params.h"
#include "param_save.h"
#include "my_string.h"
#include "my_fp.h"
#include "printf.h"
#include "errormessage.h"
#include "pwmgeneration.h"
#include "stm32_can.h"
#include "terminalcommands.h"
#include "foc.h"
#include "binlog_list.h"

static void LoadDefaults(Terminal* term, char *arg);
static void GetAll(Terminal* term, char *arg);
static void PrintList(Terminal* term, char *arg);
static void PrintAtr(Terminal* term, char *arg);
static void StopInverter(Terminal* term, char *arg);
static void StartInverter(Terminal* term, char *arg);
static void Help(Terminal* term, char *arg);
static void PrintSerial(Terminal* term, char *arg);
static void PrintErrors(Terminal* term, char *arg);
static void SaveParameters(Terminal* term, char *arg);
static void BinLog(Terminal* term, char *arg);

static const char binlogJson[] = BINLOG_JSON;

extern "C" const TERM_CMD TermCmds[] =
{
  { "set", TerminalCommands::ParamSet },
  { "get", TerminalCommands::ParamGet },
  { "flag", TerminalCommands::ParamFlag },
  { "stream", TerminalCommands::ParamStream },
  { "binstream", TerminalCommands::ParamStreamBinary },
  { "json", TerminalCommands::PrintParamsJson },
  { "can", TerminalCommands::MapCan },
  { "save", SaveParameters },
  { "load", TerminalCommands::LoadParameters },
  { "reset", TerminalCommands::Reset },
  { "defaults", LoadDefaults },
  { "all", GetAll },
  { "list", PrintList },
  { "atr",  PrintAtr },
  { "stop", StopInverter },
  { "start", StartInverter },
  { "help", Help },
  { "serial", PrintSerial },
  { "errors", PrintErrors },
  { "binarylogging", BinLog },
  { NULL, NULL }
};

static void SaveParameters(Terminal* term, char *arg)
{
   arg = arg;
   if (Param::GetInt(Param::opmode) == 0)
   {
      Can::GetInterface(0)->Save();
      fprintf(term, "CANMAP stored\r\n");
      uint32_t crc = parm_save();
      fprintf(term, "Parameters stored, CRC=%x\r\n", crc);
   }
   else
   {
      fprintf(term, "Will not write to flash in run modes, please stop inverter before saving!\r\n");
   }
}

static void PrintList(Terminal* term, char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;
   term = term;

   printf("Available parameters and values\r\n");

   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);

      if ((Param::GetFlag((Param::PARAM_NUM)idx) & Param::FLAG_HIDDEN) == 0)
         printf("%s [%s]\r\n", pAtr->name, pAtr->unit);
   }
}

static void PrintAtr(Terminal* term, char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;
   term = term;

   printf("Parameter attributes\r\n");
   printf("Name\t\tmin - max [default]\r\n");

   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      /* Only display for params */
      if (Param::IsParam((Param::PARAM_NUM)idx) && (Param::GetFlag((Param::PARAM_NUM)idx) & Param::FLAG_HIDDEN) == 0)
      {
         printf("%s\t\t%f - %f [%f]\r\n", pAtr->name,pAtr->min,pAtr->max,pAtr->def);
      }
   }
}

static void LoadDefaults(Terminal* term, char *arg)
{
   arg = arg;
   term = term;
   Param::LoadDefaults();
   printf("Defaults loaded\r\n");
}

static void GetAll(Terminal* term, char *arg)
{
   const Param::Attributes *pAtr;

   arg = arg;
   term = term;

   for (uint32_t  idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
      printf("%s\t\t%f\r\n", pAtr->name, Param::Get((Param::PARAM_NUM)idx));
   }
}

static void StopInverter(Terminal* term, char *arg)
{
   arg = arg;
   term = term;
   Param::SetInt(Param::opmode, 0);
   printf("Inverter halted.\r\n");
}

static void StartInverter(Terminal* term, char *arg)
{
   term = term;
   arg = my_trim(arg);
   int val = my_atoi(arg);
   if (val < MOD_LAST)
   {
      Param::SetInt(Param::opmode, val);
      PwmGeneration::SetOpmode(FP_TOINT(val));
      printf("Inverter started\r\n");
   }
   else
   {
      printf("Invalid inverter mode");
   }
}

static void PrintErrors(Terminal* term, char *arg)
{
   term = term;
   arg = arg;
   ErrorMessage::PrintAllErrors();
}

static void PrintSerial(Terminal* term, char *arg)
{
   arg = arg;
   term = term;
   printf("%X%X%X\r\n", DESIG_UNIQUE_ID2, DESIG_UNIQUE_ID1, DESIG_UNIQUE_ID0);
}

static void Help(Terminal* term, char *arg)
{
   arg = arg;
   term = term;
}

static void BinLog(Terminal* term, char *arg)
{
   char dummy[] = "";
   LogStruct l;
   uint8_t *pBuff = (uint8_t *)&l;
   l.counter = 0;
   arg = arg;
   uint32_t ctr = 0;

   TerminalCommands::PrintParamsJson(term, dummy);
   printf(binlogJson);

   while (!term->KeyPressed())
   {
      PwmGeneration::WaitISR();

      /*l.counter++;
      l.angle = Param::GetInt(Param::angle) >> 2;
      l.idc = Param::Get(Param::idc) >> 3; //0.25A resolution, +-2000A
      l.il1 = Param::Get(Param::il1) >> 3;  //0.25A resolution, +-2000A
      l.il2 = Param::Get(Param::il2) >> 3;  //0.25A resolution, +-2000A
      l.pwm1 = FOC::DutyCycles[0] >> 2;
      l.opmode = Param::GetInt(Param::opmode);
      l.pwm2 = FOC::DutyCycles[1] >> 2;
      l.desat = PwmGeneration::Tripped();
      l.pwm3 = FOC::DutyCycles[2] >> 2;
      l.iqref = Param::Get(Param::iqref) >> 3;  //0.25A resolution, +-2000A
      l.idref = Param::Get(Param::idref) >> 3;  //0.25A resolution, +-2000A
      l.ifw = Param::Get(Param::ifw) >> 3;  //0.25A resolution, +-2000A
      l.uq = Param::GetInt(Param::uq) >> 1;
      l.ud = Param::GetInt(Param::ud) >> 1;

      uint8_t csum = 0;
      for(uint32_t j=0; j < sizeof(l); j++) csum += *pBuff++;
      l.csum = csum;

      term->SendBinary(pBuff, sizeof(l));*/
      term->SendBinary(&ctr, 4);
      ctr++;
   }
}
