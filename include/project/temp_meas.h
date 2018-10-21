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
#ifndef TEMP_MEAS_H_INCLUDED
#define TEMP_MEAS_H_INCLUDED

#include "my_fp.h"

class TempMeas
{
public:
   enum Sensors
   {
      TEMP_JCURVE = 0,
      TEMP_SEMIKRON = 1,
      TEMP_MBB600 = 2,
      TEMP_SEPARATOR = 9,
      TEMP_KTY83 = 12,
      TEMP_KTY84 = 13,
      TEMP_LEAF = 14,
      TEMP_TESLA_100K = 15,
      TEMP_TESLA_52K = 16,
      TEMP_LAST
   };

   static s32fp Lookup(int digit, Sensors sensorId);
};


#ifdef __TEMP_LU_TABLES
#define JCURVE \
57	,\
76	,\
100	,\
132	,\
171	,\
220	,\
280	,\
353	,\
440	,\
544	,\
665	,\
805	,\
963	,\
1141	,\
1338	,\
1551	,\
1779	,\
2019	,\
2268	,\
2523	,\
2779	,\
3032	,\
3279	,\
3519	,\
3748	,\
3964	,\
4167

#define MBB600 \
3971	,\
3632	,\
3292	,\
2959	,\
2641	,\
2343	,\
2069	,\
1819	,\
1596	,\
1397	,\
1222	,\
1069	,\
935	,\
819	,\
718	,\
631	,\
555	,\
489	,\
433	,\
383	,\
340	,\
303

#define LEAF \
49	,\
68	,\
91	,\
119	,\
153	,\
193	,\
239	,\
290	,\
348	,\
411	,\
479	,\
551	,\
628	,\
707	,\
788	,\
871	,\
955	,\
1039

#define SEMIKRON \
3767	,\
3685	,\
3605	,\
3528	,\
3456	,\
3385	,\
3317	,\
3254	,\
3191	,\
3130	,\
3074	,\
3018	,\
2963	,\
2913	,\
2862	,\
2813	,\
2767	,\
2722	,\
2678	,\
2636	,\
2595


#define KTY83 \
2035	,\
1968	,\
1901	,\
1835	,\
1769	,\
1705	,\
1643	,\
1582	,\
1522	,\
1465	,\
1409	,\
1356	,\
1304	,\
1255	,\
1208	,\
1162	,\
1119	,\
1077	,\
1037	,\
999	,\
962	,\
927	,\
894	,\

#define KTY84 \
2283	,\
2231	,\
2179	,\
2125	,\
2071	,\
2018	,\
1963	,\
1936	,\
1908	,\
1856	,\
1802	,\
1750	,\
1699	,\
1649	,\
1600	,\
1552	,\
1505	,\
1459	,\
1414	,\
1372	,\
1329	,\
1289	,\
1250	,\
1212	,\
1175	,\
1140	,\
1105	,\
1073	,\
1041	,\
1010	,\
980	,\
952	,\
924	,\
899	,\
876	,\
856

#define TESLA_100K \
4123	,\
3718	,\
3304	,\
2897	,\
2510	,\
2152	,\
1830	,\
1546	,\
1299	,\
1089	,\
911	,\
761	,\
637	,\
533	,\
447	,\
375	,\
316	,\
267	,\
226	,\
192	,\
163	,\
139	,\
119	,\
103	,\
88	,\
76	,\
66	,\
57	,\
50	,\
44	,\
38	,\
34	,\
29	,\
26	,\
23	,\
20	,\
18	,\
16	,\
14	,\
13	,\
11	,\
10	,\
9

#define TESLA_52K \
3416, \
2789, \
1882, \
1109, \
902, \
670, \
504, \
468, \
432, \
395, \
358


#endif

#endif // TEMP_MEAS_H_INCLUDED
