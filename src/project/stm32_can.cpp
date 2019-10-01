/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2016 Nail Güzel
 * Johannes Huebner <dev@johanneshuebner.com>
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
#include <stdint.h>
#include "hwdefs.h"
#include "my_string.h"
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/rtc.h>
#include "stm32_can.h"
#include "printf.h"

#define MAX_MESSAGES          10
#define MAX_ITEMS_PER_MESSAGE 8
#define IDS_PER_BANK          4
#define SENDBUFFER_LEN        15
#define SDO_WRITE             0x40
#define SDO_READ              0x22
#define SDO_ABORT             0x80
#define SDO_WRITE_REPLY       0x23
#define SDO_READ_REPLY        0x43
#define SDO_ERR_INVIDX        0x06020000
#define SDO_ERR_RANGE         0x06090030
#define SENDMAP_ADDRESS       CANMAP_ADDRESS
#define RECVMAP_ADDRESS       (CANMAP_ADDRESS + sizeof(canSendMap))
#define CRC_ADDRESS           (CANMAP_ADDRESS + sizeof(canSendMap) + sizeof(canRecvMap))
#define SENDMAP_WORDS         (sizeof(canSendMap) / sizeof(uint32_t))
#define RECVMAP_WORDS         (sizeof(canRecvMap) / sizeof(uint32_t))
#define CANID_UNSET           0xffff
#define NUMBITS_LASTMARKER    -1
#define forEachCanMap(c,m) for (CANIDMAP *c = m; (c - m) < MAX_MESSAGES && c->canId < CANID_UNSET; c++)
#define forEachPosMap(c,m) for (CANPOS *c = m->items; (c - m->items) < MAX_ITEMS_PER_MESSAGE && c->numBits > 0; c++)

#if (2 *((MAX_ITEMS_PER_MESSAGE * 6 + 2) * MAX_MESSAGES + 2) + 4) > FLASH_PAGE_SIZE
#error CANMAP will not fit in one flash page
#endif

namespace Can
{

typedef struct
{
   uint16_t mapParam;
   s16fp gain;
   uint8_t offsetBits;
   int8_t numBits;
} CANPOS;

typedef struct
{
   uint16_t canId;
   CANPOS items[MAX_ITEMS_PER_MESSAGE];
} CANIDMAP;

typedef struct
{
   uint8_t cmd;
   uint16_t index;
   uint8_t subIndex;
   uint32_t data;
} __attribute__((packed)) CAN_SDO;

typedef struct
{
   uint32_t ts1;
   uint32_t ts2;
   uint32_t prescaler;
} CANSPEED;

typedef struct
{
   uint16_t id;
   uint32_t data[2];
} SENDBUFFER;

static void ProcessSDO(uint32_t data[2]);
static void Clear(CANIDMAP *canMap);
static int Remove(CANIDMAP *canMap, Param::PARAM_NUM param);
static int Add(CANIDMAP *canMap, Param::PARAM_NUM param, int canId, int offset, int length, s16fp gain);
static uint32_t SaveToFlash(uint32_t baseAddress, uint32_t* data, int len);
static int LoadFromFlash();
static CANIDMAP *FindById(CANIDMAP *canMap, int canId);
static int CopyIdMapExcept(CANIDMAP *source, CANIDMAP *dest, Param::PARAM_NUM param);
static void ReplaceParamEnumByUid(CANIDMAP *canMap);
static void ReplaceParamUidByEnum(CANIDMAP *canMap);
static void ConfigureFilters();

static CANIDMAP canSendMap[MAX_MESSAGES];
static CANIDMAP canRecvMap[MAX_MESSAGES];
static uint32_t lastRxTimestamp = 0;
static SENDBUFFER sendBuffer[SENDBUFFER_LEN];
static int sendCnt;

static const CANSPEED canSpeed[BaudLast] =
{
   { CAN_BTR_TS1_9TQ, CAN_BTR_TS2_6TQ, 9 }, //250kbps
   { CAN_BTR_TS1_4TQ, CAN_BTR_TS2_3TQ, 9 }, //500kbps
   { CAN_BTR_TS1_5TQ, CAN_BTR_TS2_3TQ, 5 }, //800kbps
   { CAN_BTR_TS1_6TQ, CAN_BTR_TS2_5TQ, 3 }, //1000kbps
};

int AddSend(Param::PARAM_NUM param, int canId, int offset, int length, s16fp gain)
{
   return Add(canSendMap, param, canId, offset, length, gain);
}

int AddRecv(Param::PARAM_NUM param, int canId, int offset, int length, s16fp gain)
{
   int res = Add(canRecvMap, param, canId, offset, length, gain);
   ConfigureFilters();
   return res;
}

bool FindMap(Param::PARAM_NUM param, int& canId, int& offset, int& length, s32fp& gain, bool& rx)
{
   rx = false;
   bool done = false;

   for (CANIDMAP *map = canSendMap; !done; map = canRecvMap)
   {
      forEachCanMap(curMap, map)
      {
         forEachPosMap(curPos, curMap)
         {
            if (curPos->mapParam == param)
            {
               canId = curMap->canId;
               offset = curPos->offsetBits;
               length = curPos->numBits;
               gain = curPos->gain;
               return true;
            }
         }
      }
      done = rx;
      rx = true;
   }
   return false;
}

void Save()
{
   uint32_t crc;
   crc_reset();

   flash_unlock();
   flash_set_ws(2);
   flash_erase_page(CANMAP_ADDRESS);

   ReplaceParamEnumByUid(canSendMap);
   ReplaceParamEnumByUid(canRecvMap);

   SaveToFlash(SENDMAP_ADDRESS, (uint32_t *)canSendMap, SENDMAP_WORDS);
   crc = SaveToFlash(RECVMAP_ADDRESS, (uint32_t *)canRecvMap, RECVMAP_WORDS);
   SaveToFlash(CRC_ADDRESS, &crc, 1);
   flash_lock();

   ReplaceParamUidByEnum(canSendMap);
   ReplaceParamUidByEnum(canRecvMap);
}

void SendAll()
{
   forEachCanMap(curMap, canSendMap)
   {
      uint32_t data[2] = { 0 }; //Had an issue with uint64_t, otherwise would have used that

      forEachPosMap(curPos, curMap)
      {
         s32fp val = FP_MUL(Param::Get((Param::PARAM_NUM)curPos->mapParam), curPos->gain);

         val &= ((1 << curPos->numBits) - 1);

         if (curPos->offsetBits > 31)
         {
            data[1] |= val << (curPos->offsetBits - 32);
         }
         else
         {
            data[0] |= val << curPos->offsetBits;
         }
      }

      Send(curMap->canId, data);
   }
}

void Clear(void)
{
   Clear(canSendMap);
   Clear(canRecvMap);
}

int Remove(Param::PARAM_NUM param)
{
   int removed = Remove(canSendMap, param);
   removed += Remove(canRecvMap, param);

   return removed;
}

void Init(enum baudrates baudrate)
{
   Clear();
   LoadFromFlash();

	AFIO_MAPR |= AFIO_MAPR_CAN1_REMAP_PORTA;

	// Configure CAN pin: RX (input pull-up).
	gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
	gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);

	// Configure CAN pin: TX.-
	gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);


	// Reset CAN
	can_reset(CAN1);

	SetBaudrate(baudrate);
   ConfigureFilters();
	// Enable CAN RX interrupt.
	can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

void SetBaudrate(enum baudrates baudrate)
{
	// CAN cell init.
	 // Setting the bitrate to 250KBit. APB1 = 36MHz,
	 // prescaler = 9 -> 4MHz time quanta frequency.
	 // 1tq sync + 9tq bit segment1 (TS1) + 6tq bit segment2 (TS2) =
	 // 16time quanto per bit period, therefor 4MHz/16 = 250kHz
	 //
	can_init(CAN1,
		     false,          // TTCM: Time triggered comm mode?
		     true,           // ABOM: Automatic bus-off management?
		     false,          // AWUM: Automatic wakeup mode?
		     false,          // NART: No automatic retransmission?
		     false,          // RFLM: Receive FIFO locked mode?
		     false,          // TXFP: Transmit FIFO priority?
		     CAN_BTR_SJW_1TQ,
		     canSpeed[baudrate].ts1,
		     canSpeed[baudrate].ts2,
		     canSpeed[baudrate].prescaler,				// BRP+1: Baud rate prescaler
		     false,
		     false);
}

uint32_t GetLastRxTimestamp()
{
   return lastRxTimestamp;
}

void Send(uint32_t canId, uint32_t data[2])
{
   can_disable_irq(CAN1, CAN_IER_TMEIE);

   if (can_transmit(CAN1, canId, false, false, 8, (uint8_t*)data) < 0 && sendCnt < SENDBUFFER_LEN)
   {
      /* enqueue in send buffer if all TX mailboxes are full */
      sendBuffer[sendCnt].id = canId;
      sendBuffer[sendCnt].data[0] = data[0];
      sendBuffer[sendCnt].data[1] = data[1];
      sendCnt++;
   }

   if (sendCnt > 0)
   {
      can_enable_irq(CAN1, CAN_IER_TMEIE);
   }
}

extern "C" void usb_lp_can_rx0_isr(void)
{
	uint32_t id;
	bool ext, rtr;
	uint8_t length, fmi;
	uint32_t data[2];

   for (int fifo = 0; fifo < 2; fifo++)
   {
      while (can_receive(CAN1, fifo, true, &id, &ext, &rtr, &fmi, &length, (uint8_t*)data, NULL) > 0)
      {
         //printf("fifo: %d, id: %x, len: %d, data[0]: %x, data[1]: %x\r\n", fifo, id, length, data[0], data[1]);
         if (id == 0x601 && length == 8) //SDO request, nodeid=1
         {
            ProcessSDO(data);
         }
         else if (length == 8)
         {
            CANIDMAP *recvMap = FindById(canRecvMap, id);

            if (0 != recvMap)
            {
               forEachPosMap(curPos, recvMap)
               {
                  s32fp val;

                  if (curPos->offsetBits > 31)
                  {
                     val = FP_FROMINT((data[1] >> (curPos->offsetBits - 32)) & ((1 << curPos->numBits) - 1));
                  }
                  else
                  {
                     val = FP_FROMINT((data[0] >> curPos->offsetBits) & ((1 << curPos->numBits) - 1));
                  }
                  val = FP_MUL(val, curPos->gain);

                  if (Param::IsParam((Param::PARAM_NUM)curPos->mapParam))
                     Param::Set((Param::PARAM_NUM)curPos->mapParam, val);
                  else
                     Param::SetFlt((Param::PARAM_NUM)curPos->mapParam, val);
               }
               lastRxTimestamp = rtc_get_counter_val();
            }
         }
      }
   }
}

extern "C" void usb_hp_can_tx_isr()
{
   if (sendCnt > 0)
   {
      sendCnt--;
      can_transmit(CAN1, sendBuffer[sendCnt].id, false, false, 8, (uint8_t*)sendBuffer[sendCnt].data);
   }
   else
   {
      can_disable_irq(CAN1, CAN_IER_TMEIE);
   }
}

//http://www.byteme.org.uk/canopenparent/canopen/sdo-service-data-objects-canopen/
static void ProcessSDO(uint32_t data[2])
{
   CAN_SDO *sdo = (CAN_SDO*)data;
   if (sdo->index == 0x2000 && sdo->subIndex < Param::PARAM_LAST)
   {
      if (sdo->cmd == SDO_WRITE)
      {
         if (Param::Set((Param::PARAM_NUM)sdo->subIndex, sdo->data) == 0)
         {
            sdo->cmd = SDO_WRITE_REPLY;
         }
         else
         {
            sdo->cmd = SDO_ABORT;
            sdo->data = SDO_ERR_RANGE;
         }
      }
      else if (sdo->cmd == SDO_READ)
      {
         sdo->data = Param::Get((Param::PARAM_NUM)sdo->subIndex);
         sdo->cmd = SDO_READ_REPLY;
      }
   }
   else if (sdo->index >= 0x3000 && sdo->index < 0x4800 && sdo->subIndex < Param::PARAM_LAST)
   {
      if (sdo->cmd == SDO_WRITE)
      {
         int result;
         int offset = sdo->data & 0xFF;
         int len = (sdo->data >> 8) & 0xFF;
         s32fp gain = sdo->data >> 16;

         if ((sdo->index & 0x4000) == 0x4000)
         {
            result = AddRecv((Param::PARAM_NUM)sdo->subIndex, sdo->index & 0x7FF, offset, len, gain);
         }
         else
         {
            result = AddSend((Param::PARAM_NUM)sdo->subIndex, sdo->index & 0x7FF, offset, len, gain);
         }

         if (result >= 0)
         {
            sdo->cmd = SDO_WRITE_REPLY;
         }
         else
         {
            sdo->cmd = SDO_ABORT;
            sdo->data = SDO_ERR_RANGE;
         }
      }
   }
   else
   {
      sdo->cmd = SDO_ABORT;
      sdo->data = SDO_ERR_INVIDX;
   }
   Send(0x581, data);
}

static void ConfigureFilters()
{
   uint16_t idList[IDS_PER_BANK] = { 0x601 << 5, 0, 0, 0 };
   int idIndex = 1;
   int filterId = 0;

   forEachCanMap(curMap, canRecvMap)
   {
      idList[idIndex] = curMap->canId << 5;
      idIndex++;

      if (idIndex == IDS_PER_BANK)
      {
         can_filter_id_list_16bit_init(
               filterId,
               idList[0],
               idList[1],
               idList[2],
               idList[3],
               filterId & 1,
               true);
         idIndex = 0;
         filterId++;
         idList[0] = idList[1] = idList[2] = idList[3] = 0;
      }
   }
   //loop terminates before adding last set of filters
   if (idIndex > 0)
   {
      can_filter_id_list_16bit_init(
            filterId,
            idList[0],
            idList[1],
            idList[2],
            idList[3],
            filterId & 1,
            true);
   }
}

static int LoadFromFlash()
{
   uint32_t* data = (uint32_t *)CANMAP_ADDRESS;
   uint32_t storedCrc = *(uint32_t*)CRC_ADDRESS;
   uint32_t crc;

   crc_reset();
   crc = crc_calculate_block(data, SENDMAP_WORDS + RECVMAP_WORDS);

   if (storedCrc == crc)
   {
      memcpy32((int*)canSendMap, (int*)SENDMAP_ADDRESS, SENDMAP_WORDS);
      memcpy32((int*)canRecvMap, (int*)RECVMAP_ADDRESS, RECVMAP_WORDS);
      ReplaceParamUidByEnum(canSendMap);
      ReplaceParamUidByEnum(canRecvMap);
      return 1;
   }
   return 0;
}

static int Remove(CANIDMAP *canMap, Param::PARAM_NUM param)
{
   CANIDMAP copyMap[MAX_MESSAGES];

   Clear(copyMap);
   int removed = CopyIdMapExcept(canMap, copyMap, param);
   Clear(canMap);
   CopyIdMapExcept(copyMap, canMap, param);

   return removed;
}

static int Add(CANIDMAP *canMap, Param::PARAM_NUM param, int canId, int offset, int length, s16fp gain)
{
   if (canId > 0x7ff) return CAN_ERR_INVALID_ID;
   if (offset > 63) return CAN_ERR_INVALID_OFS;
   if (length > 32) return CAN_ERR_INVALID_LEN;

   CANIDMAP *existingMap = FindById(canMap, canId);

   if (0 == existingMap)
   {
      existingMap = FindById(canMap, CANID_UNSET);
      if (0 == existingMap)
         return CAN_ERR_MAXMESSAGES;

      existingMap->canId = canId;
   }

   CANPOS* freeItem = existingMap->items;

   for (; freeItem->numBits > 0; freeItem++);

   if (freeItem->numBits == NUMBITS_LASTMARKER)
      return CAN_ERR_MAXITEMS;

   freeItem->mapParam = param;
   freeItem->gain = gain;
   freeItem->offsetBits = offset;
   freeItem->numBits = length;

   int count = 0;

   forEachCanMap(curMap, canMap)
      count++;

   return count;
}

static void Clear(CANIDMAP *canMap)
{
   for (int i = 0; i < MAX_MESSAGES; i++)
   {
      canMap[i].canId = CANID_UNSET;

      for (int j = 0; j < MAX_ITEMS_PER_MESSAGE; j++)
      {
         canMap[i].items[j].numBits = 0;
      }
   }
}

static CANIDMAP *FindById(CANIDMAP *canMap, int canId)
{
   for (int i = 0; i < MAX_MESSAGES; i++)
   {
      if (canMap[i].canId == canId)
         return &canMap[i];
   }
   return 0;
}

static uint32_t SaveToFlash(uint32_t baseAddress, uint32_t* data, int len)
{
   uint32_t crc = 0;

   for (int idx = 0; idx < len; idx++)
   {
      crc = crc_calculate(*data);
      flash_program_word(baseAddress + idx * sizeof(uint32_t), *data);
      data++;
   }

   return crc;
}

static int CopyIdMapExcept(CANIDMAP *source, CANIDMAP *dest, Param::PARAM_NUM param)
{
   int i = 0, removed = 0;

   forEachCanMap(curMap, source)
   {
      bool discardId = true;
      int j = 0;

      forEachPosMap(curPos, curMap)
      {
         if (curPos->mapParam != param)
         {
            discardId = false;
            dest[i].items[j] = *curPos;
            j++;
         }
         else
         {
            removed++;
         }
      }

      if (!discardId)
      {
         dest[i].canId = curMap->canId;
         i++;
      }
   }

   return removed;
}

static void ReplaceParamEnumByUid(CANIDMAP *canMap)
{
   forEachCanMap(curMap, canMap)
   {
      forEachPosMap(curPos, curMap)
      {
         const Param::Attributes* attr = Param::GetAttrib((Param::PARAM_NUM)curPos->mapParam);
         curPos->mapParam = (uint16_t)attr->id;
      }
   }
}

static void ReplaceParamUidByEnum(CANIDMAP *canMap)
{
   forEachCanMap(curMap, canMap)
   {
      forEachPosMap(curPos, curMap)
      {
         Param::PARAM_NUM param = Param::NumFromId(curPos->mapParam);
         curPos->mapParam = param;
      }
   }
}

}
