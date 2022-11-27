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
   #if CONTROL == CTRL_SINE
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
   #else
   printf("Not implemented in FOC variant\r\n");
   #endif
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

