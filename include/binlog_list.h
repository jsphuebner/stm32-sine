#ifndef BINLOG_LIST_H_INCLUDED
#define BINLOG_LIST_H_INCLUDED

/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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
#include "my_string.h"

#define BINLOG_LIST \
   BINLOG_ENTRY(BL_UNSIGNED, counter, 8) \
   BINLOG_ENTRY(BL_SIGNED,   angle,  14) \
   BINLOG_ENTRY(BL_SIGNED,   idc,    14) \
   BINLOG_ENTRY(BL_SIGNED,   il1,    14) \
   BINLOG_ENTRY(BL_SIGNED,   il2,    14) \
   BINLOG_ENTRY(BL_UNSIGNED, pwm1,   14) \
   BINLOG_ENTRY(BL_UNSIGNED, opmode, 2)  \
   BINLOG_ENTRY(BL_UNSIGNED, pwm2,   14) \
   BINLOG_ENTRY(BL_UNSIGNED, desat,  1)  \
   BINLOG_ENTRY(BL_UNSIGNED, pwm3,   14) \
   BINLOG_ENTRY(BL_SIGNED,   iqref,  14) \
   BINLOG_ENTRY(BL_SIGNED,   idref,  14) \
   BINLOG_ENTRY(BL_SIGNED,   ifw,    14) \
   BINLOG_ENTRY(BL_SIGNED,   uq,     16) \
   BINLOG_ENTRY(BL_SIGNED,   ud,     16)
//   BINLOG_ENTRY(BL_UNSIGNED, csum,   8)  is included implicitely

#define BL_SIGNED int32_t
#define BL_UNSIGNED uint32_t
#define BINLOG_ENTRY(t,n,b) t n:b;
struct LogStruct
{
   BINLOG_LIST
   uint32_t csum:8;
} __attribute__((packed));
#undef BINLOG_ENTRY
#undef BL_SIGNED
#undef BL_UNSIGNED

#define BL_SIGNED true
#define BL_UNSIGNED false
#define BINLOG_ENTRY(t,n,b) {"name":#n,"size":b,"scale":1,"signed":t},
#define BINLOG_JSON "[" STRINGIFY(BINLOG_LIST)"{\"name\":\"csum\",\"size\":8,\"scale\":1,\"signed\":false}]\r\n"

#endif // BINLOG_LIST_H_INCLUDED
