/*
 * This file is part of the tumanako_vc project.
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

#include "my_string.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include "terminal.h"
#include "hwdefs.h"
#include <stdarg.h>

static const TERM_CMD *CmdLookup(char *buf);
static void term_send(uint32_t usart, const char *str);
static void ResetDMA();

extern const TERM_CMD TermCmds[];
static char* inBuf;

void term_Init(char* termBuf)
{
   inBuf = termBuf;
}

/** Run the terminal */
void term_Run()
{
   char args[TERM_BUFSIZE];
   const TERM_CMD *pCurCmd = NULL;
   int lastIdx = 0;

   while (1)
   {
      int currentIdx = TERM_BUFSIZE - TERM_USART_CNDTR;

      if (0 == TERM_USART_CNDTR)
         ResetDMA();

      while (lastIdx < currentIdx) //echo
         usart_send_blocking(TERM_USART, inBuf[lastIdx++]);

      if (currentIdx > 0)
      {
         if (inBuf[currentIdx - 1] == '\n' || inBuf[currentIdx - 1] == '\r')
         {
            inBuf[currentIdx] = 0;
            lastIdx = 0;
            char *space = (char*)my_strchr(inBuf, ' ');

            if (0 == *space) //No args after command, look for end of line
            {
               space = (char*)my_strchr(inBuf, '\n');
               args[0] = 0;
            }
            else //There are arguments, copy everything behind the space
            {
               my_strcpy(args, space + 1);
            }

            if (0 == *space) //No \n found? try \r
               space = (char*)my_strchr(inBuf, '\r');

            *space = 0;
            pCurCmd = CmdLookup(inBuf);
            ResetDMA();

            if (NULL != pCurCmd)
            {
               pCurCmd->CmdFunc(args);
            }
            else if (currentIdx > 1)
            {
               term_send(TERM_USART, "Unknown command sequence\r\n");
            }
         }
         else if (inBuf[0] == '!' && NULL != pCurCmd)
         {
            ResetDMA();
            lastIdx = 0;
            pCurCmd->CmdFunc(args);
         }
      }
   } /* while(1) */
} /* term_Run */

int putchar(int c)
{
   usart_send_blocking(TERM_USART, c);
   return 0;
}

static void ResetDMA()
{
   dma_disable_channel(DMA1, TERM_USART_DMACHAN);
   dma_set_number_of_data(DMA1, TERM_USART_DMACHAN, TERM_BUFSIZE);
   dma_enable_channel(DMA1, TERM_USART_DMACHAN);
}

static const TERM_CMD *CmdLookup(char *buf)
{
   const TERM_CMD *pCmd = TermCmds;

   for (; NULL != pCmd->cmd; pCmd++)
   {
      if (0 == my_strcmp(buf, pCmd->cmd))
      {
         break;
      }
   }
   if (NULL == pCmd->cmd)
   {
      pCmd = NULL;
   }
   return pCmd;
}

static void term_send(uint32_t usart, const char *str)
{
   for (;*str > 0; str++)
       usart_send_blocking(usart, *str);
}


