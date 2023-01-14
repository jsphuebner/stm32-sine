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

#define VER 5.25.R

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 148
//Next value Id: 2049
/*              category     name         unit       min     max     default id */

#define MOTOR_PARAMETERS_COMMON \
    PARAM_ENTRY(CAT_MOTOR,   polepairs,   "",        1,      16,     2,      32  ) \
    PARAM_ENTRY(CAT_MOTOR,   respolepairs,"",        1,      16,     1,      93  ) \
    PARAM_ENTRY(CAT_MOTOR,   sincosofs,   "dig",     1,      4096,   2048,   131 ) \
    PARAM_ENTRY(CAT_MOTOR,   encmode,     ENCMODES,  0,      5,      0,      75  ) \
    PARAM_ENTRY(CAT_MOTOR,   fmax,        "Hz",      21,     1000,   200,    9   ) \
    PARAM_ENTRY(CAT_MOTOR,   numimp,      "ppr",     8,      8192,   60,     15  ) \
    PARAM_ENTRY(CAT_MOTOR,   dirchrpm,    "rpm",     0,      20000,  100,    87  ) \
    PARAM_ENTRY(CAT_MOTOR,   dirmode,     DIRMODES,  0,      4,      1,      95  ) \
    PARAM_ENTRY(CAT_MOTOR,   snsm,        SNS_M,     12,     23,     12,     46  )

#define MOTOR_PARAMETERS_SINE \
    PARAM_ENTRY(CAT_MOTOR,   boost,       "dig",     0,      37813,  1700,   1   ) \
    PARAM_ENTRY(CAT_MOTOR,   fweak,       "Hz",      0,      1000,   90,     2   ) \
    PARAM_ENTRY(CAT_MOTOR,   fweakstrt,   "Hz",      0,      1000,   400,    134 ) \
    PARAM_ENTRY(CAT_MOTOR,   fconst,      "Hz",      0,      1000,   180,    99  ) \
    PARAM_ENTRY(CAT_MOTOR,   udcnom,      "V",       0,      1000,   0,      78  ) \
    PARAM_ENTRY(CAT_MOTOR,   fslipmin,    "Hz",      0.3,    10,     1,      37  ) \
    PARAM_ENTRY(CAT_MOTOR,   fslipmax,    "Hz",      0.3,    10,     3,      33  ) \
    PARAM_ENTRY(CAT_MOTOR,   fslipconstmax,"Hz",     0,      10,     5,      100 )

#define MOTOR_PARAMETERS_FOC \
    PARAM_ENTRY(CAT_MOTOR,   curkp,       "",        0,      20000,  32,     107 ) \
    PARAM_ENTRY(CAT_MOTOR,   curki,       "",        0,      100000, 20000,  108 ) \
    PARAM_ENTRY(CAT_MOTOR,   vlimflt,     "",        0,      16,     10,     145 ) \
    PARAM_ENTRY(CAT_MOTOR,   vlimmargin,  "dig",     0,      10000,  2500,   141 ) \
    PARAM_ENTRY(CAT_MOTOR,   fwcurmax,    "A",       -1000,  0,     -100,    144 ) \
    PARAM_ENTRY(CAT_MOTOR,   syncofs,     "dig",     0,      65535,  0,      70  ) \
    PARAM_ENTRY(CAT_MOTOR,   lqminusld,   "mH",      0,      1000,   0,      139 ) \
    PARAM_ENTRY(CAT_MOTOR,   fluxlinkage, "mWeber",  0,      1000,   90,     140 ) \
    PARAM_ENTRY(CAT_MOTOR,   syncadv,     "dig/Hz",  0,      65535,  10,     133 ) \

#define INVERTER_PARAMETERS_COMMON \
    PARAM_ENTRY(CAT_INVERTER,pwmfrq,      PWMFRQS,   0,      2,      1,      13  ) \
    PARAM_ENTRY(CAT_INVERTER,pwmpol,      PWMPOLS,   0,      1,      0,      52  ) \
    PARAM_ENTRY(CAT_INVERTER,deadtime,    "dig",     0,      255,    63,     14  ) \
    PARAM_ENTRY(CAT_INVERTER,ocurlim,     "A",       -65536, 65536,  100,    22  ) \
    PARAM_ENTRY(CAT_INVERTER,il1gain,     "dig/A",   -100,   100,    4.7,    27  ) \
    PARAM_ENTRY(CAT_INVERTER,il2gain,     "dig/A",   -100,   100,    4.7,    28  ) \
    PARAM_ENTRY(CAT_INVERTER,udcgain,     "dig/V",   0,      4095,   6.175,  29  ) \
    PARAM_ENTRY(CAT_INVERTER,udcofs,      "dig",     0,      4095,   0,      77  ) \
    PARAM_ENTRY(CAT_INVERTER,udclim,      "V",       0,      1000,   540,    48  ) \
    PARAM_ENTRY(CAT_INVERTER,snshs,       SNS_HS,    0,      7,      0,      45  )

#define INVERTER_PARAMETERS_FOC \
    PARAM_ENTRY(CAT_INVERTER,pinswap,     SWAPS,     0,      15,     0,      109 )

#define DERATE_PARAMETERS_COMMON \
    PARAM_ENTRY(CAT_DERATE,  bmslimhigh,  "%",       0,      100,    50,     55  ) \
    PARAM_ENTRY(CAT_DERATE,  bmslimlow,   "%",       -100,   0,      -1,     56  ) \
    PARAM_ENTRY(CAT_DERATE,  udcmin,      "V",       0,      1000,   450,    42  ) \
    PARAM_ENTRY(CAT_DERATE,  udcmax,      "V",       0,      1000,   520,    43  ) \
    PARAM_ENTRY(CAT_DERATE,  idcmax,      "A",       0,      5000,   5000,   96  ) \
    PARAM_ENTRY(CAT_DERATE,  idcmin,      "A",       -5000,  0,     -5000,   98  ) \
    PARAM_ENTRY(CAT_DERATE,  idckp,       "dig",     0.1,    20,    2,       130 ) \
    PARAM_ENTRY(CAT_DERATE,  idcflt,      "dig",     0,      11,    9,       132 ) \
    PARAM_ENTRY(CAT_DERATE,  tmphsmax,    "°C",      50,     150,   85,      125 ) \
    PARAM_ENTRY(CAT_DERATE,  tmpmmax,     "°C",      70,     300,   300,     127 ) \
    PARAM_ENTRY(CAT_DERATE,  throtmax,    "%",       0,      100,   100,     97  ) \
    PARAM_ENTRY(CAT_DERATE,  throtmin,    "%",       -100,   0,     -100,    119 )

#define DERATE_PARAMETERS_SINE \
    PARAM_ENTRY(CAT_DERATE,  iacmax,      "A",       0,      5000,   5000,   89  ) \
    PARAM_ENTRY(CAT_DERATE,  ifltrise,    "dig",     0,      32,    10,      91  ) \
    PARAM_ENTRY(CAT_DERATE,  ifltfall,    "dig",     0,      32,     3,      92  ) \

#define CHARGER_PARAMETERS \
    PARAM_ENTRY(CAT_CHARGER, chargemode,  CHARGEMODS,0,      4,      0,      74  ) \
    PARAM_ENTRY(CAT_CHARGER, chargecur,   "A",       0,      50,     0,      71  ) \
    PARAM_ENTRY(CAT_CHARGER, chargekp,    "dig",     -100,   100,    80,     72  ) \
    PARAM_ENTRY(CAT_CHARGER, chargeki,    "dig",     -100,   100,    10,     126 ) \
    PARAM_ENTRY(CAT_CHARGER, chargeflt,   "dig",     0,      10,     8,      73  ) \
    PARAM_ENTRY(CAT_CHARGER, chargepwmin, "%",       0,      99,     0,      128 ) \
    PARAM_ENTRY(CAT_CHARGER, chargepwmax, "%",       0,      99,     90,     79  )

#define THROTTLE_PARAMETERS_COMMON \
    PARAM_ENTRY(CAT_THROTTLE,potmin,      "dig",     0,      4095,   0,      17  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmax,      "dig",     0,      4095,   4095,   18  ) \
    PARAM_ENTRY(CAT_THROTTLE,pot2min,     "dig",     0,      4095,   4095,   63  ) \
    PARAM_ENTRY(CAT_THROTTLE,pot2max,     "dig",     0,      4095,   4095,   64  ) \
    PARAM_ENTRY(CAT_THROTTLE,potmode,     POTMODES,  0,      6,      0,      82  ) \
    PARAM_ENTRY(CAT_THROTTLE,throtramp,   "%/10ms",  0.1,    100,    100,    81  ) \
    PARAM_ENTRY(CAT_THROTTLE,throtramprpm,"rpm",     0,      20000,  20000,  85  )

#define THROTTLE_PARAMETERS_SINE \
    PARAM_ENTRY(CAT_THROTTLE,ampmin,      "%",       0,      100,    10,     4   ) \
    PARAM_ENTRY(CAT_THROTTLE,slipstart,   "%",       10,     100,    50,     90  ) \
    PARAM_ENTRY(CAT_THROTTLE,throtfilter, "dig",     0,      10,     4,      147 )

#define THROTTLE_PARAMETERS_FOC \
   PARAM_ENTRY(CAT_THROTTLE,throtcur,    "A/%",       0,     10,     1,     105  )

#define REGEN_PARAMETERS \
    PARAM_ENTRY(CAT_REGEN,   brakeregen,  "%",       -100,   0,      -50,    38  ) \
    PARAM_ENTRY(CAT_REGEN,   regenramp,   "%/10ms",  0.1,    100,    100,    68  ) \
    PARAM_ENTRY(CAT_REGEN,   regentravel, "%",       0,      100,    30,     19  ) \
    PARAM_ENTRY(CAT_REGEN,   offthrotregen,"%",      -100,   0,      -30,    49  ) \
    PARAM_ENTRY(CAT_REGEN,   cruiseregen, "%",       -100,   0,      -30,    124 ) \
    PARAM_ENTRY(CAT_REGEN,   regenrampstr,"Hz",      0,      400,    10,     39  ) \
    PARAM_ENTRY(CAT_REGEN,   brklightout, "%",       -100,   -1,     -50,    67  )

#define AUTOMATION_CONTACT_PWM_COMM_PARAMETERS \
    PARAM_ENTRY(CAT_AUTOM,   idlespeed,   "rpm",     -100,   10000,  -100,   54  ) \
    PARAM_ENTRY(CAT_AUTOM,   idlethrotlim,"%",       0,      100,    50,     65  ) \
    PARAM_ENTRY(CAT_AUTOM,   idlemode,    IDLEMODS,  0,      4,      3,      66  ) \
    PARAM_ENTRY(CAT_AUTOM,   holdkp,      "",        -100,   0,     -0.25,   138 ) \
    PARAM_ENTRY(CAT_AUTOM,   speedkp,     "",        0,      100,    0.25,   53  ) \
    PARAM_ENTRY(CAT_AUTOM,   speedflt,    "",        0,      16,     5,      57  ) \
    PARAM_ENTRY(CAT_AUTOM,   cruisemode,  CRUISEMODS,0,      3,      0,      62  ) \
    PARAM_ENTRY(CAT_CONTACT, udcsw,       "V",       0,      1000,   330,    20  ) \
    PARAM_ENTRY(CAT_CONTACT, udcswbuck,   "V",       0,      1000,   540,    80  ) \
    PARAM_ENTRY(CAT_CONTACT, tripmode,    TRIPMODES, 0,      3,      0,      86  ) \
    PARAM_ENTRY(CAT_CONTACT, bootprec,    ONOFF,     0,      1,      0,      135 ) \
    PARAM_ENTRY(CAT_PWM,     pwmfunc,     PWMFUNCS,  0,      3,      0,      58  ) \
    PARAM_ENTRY(CAT_PWM,     pwmgain,     "",        -100000,100000, 100,    40  ) \
    PARAM_ENTRY(CAT_PWM,     pwmofs,      "dig",     -65535, 65535,  0,      41  ) \
    PARAM_ENTRY(CAT_COMM,    canspeed,    CANSPEEDS, 0,      3,      1,      83  ) \
    PARAM_ENTRY(CAT_COMM,    canperiod,   CANPERIODS,0,      1,      0,      88  ) \
    PARAM_ENTRY(CAT_COMM,    nodeid,      "",        1,      63,     1,      129 ) \
    PARAM_ENTRY(CAT_TEST,    manualstart, ONOFF,     0,      1,      0,      0   ) \

#define VALUE_BLOCK1 \
    VALUE_ENTRY(version,     VERSTR,  2039 ) \
    VALUE_ENTRY(hwver,       HWREVS,  2036 ) \
    VALUE_ENTRY(opmode,      OPMODES, 2000 ) \
    VALUE_ENTRY(lasterr,     errorListString,  2038 ) \
    VALUE_ENTRY(status,      STATUS,  2044 ) \
    VALUE_ENTRY(udc,         "V",     2001 ) \
    VALUE_ENTRY(idc,         "A",     2002 ) \
    VALUE_ENTRY(il1,         "A",     2003 ) \
    VALUE_ENTRY(il2,         "A",     2004 )

#define VALUE_BLOCK2 \
    VALUE_ENTRY(fstat,       "Hz",    2011 ) \
    VALUE_ENTRY(speed,       "rpm",   2012 ) \
    VALUE_ENTRY(cruisespeed, "rpm",   2041 ) \
    VALUE_ENTRY(turns,       "",      2037 ) \
    VALUE_ENTRY(amp,         "dig",   2013 ) \
    VALUE_ENTRY(angle,       "°",     2014 ) \
    VALUE_ENTRY(pot,         "dig",   2015 ) \
    VALUE_ENTRY(pot2,        "dig",   2016 ) \
    VALUE_ENTRY(potnom,      "%",     2017 ) \
    VALUE_ENTRY(dir,         DIRS,    2018 ) \
    VALUE_ENTRY(tmphs,       "°C",    2019 ) \
    VALUE_ENTRY(tmpm,        "°C",    2020 ) \
    VALUE_ENTRY(uaux,        "V",     2021 ) \
    VALUE_ENTRY(pwmio,       "",      2045 ) \
    VALUE_ENTRY(canio,       CANIOS,  2022 ) \
    VALUE_ENTRY(din_cruise,  ONOFF,   2023 ) \
    VALUE_ENTRY(din_start,   ONOFF,   2024 ) \
    VALUE_ENTRY(din_brake,   ONOFF,   2025 ) \
    VALUE_ENTRY(din_mprot,   OKERR,   2026 ) \
    VALUE_ENTRY(din_forward, ONOFF,   2027 ) \
    VALUE_ENTRY(din_reverse, ONOFF,   2028 ) \
    VALUE_ENTRY(din_emcystop,OKERR,   2029 ) \
    VALUE_ENTRY(din_ocur,    OKERR,   2030 ) \
    VALUE_ENTRY(din_desat,   OKERR,   2031 ) \
    VALUE_ENTRY(din_bms,     ONOFF,   2032 ) \
    VALUE_ENTRY(cpuload,     "%",     2035 ) \

#define VALUES_SINE \
    VALUE_ENTRY(ilmax,       "A",     2005 ) \
    VALUE_ENTRY(uac,         "V",     2006 ) \
    VALUE_ENTRY(il1rms,      "A",     2007 ) \
    VALUE_ENTRY(il2rms,      "A",     2008 ) \
    VALUE_ENTRY(boostcalc,   "dig",   2009 ) \
    VALUE_ENTRY(fweakcalc,   "Hz",    2010 ) \

#define VALUES_FOC \
    VALUE_ENTRY(id,      "A",     2003 ) \
    VALUE_ENTRY(iq,      "A",     2004 ) \
    VALUE_ENTRY(ifw,     "A",     2048 ) \
    VALUE_ENTRY(ud,      "dig",   2046 ) \
    VALUE_ENTRY(uq,      "dig",   2047 ) \

#if CONTROL == CTRL_SINE
#define PARAM_LIST \
    MOTOR_PARAMETERS_SINE \
    MOTOR_PARAMETERS_COMMON \
    INVERTER_PARAMETERS_COMMON \
    THROTTLE_PARAMETERS_COMMON \
    THROTTLE_PARAMETERS_SINE \
    REGEN_PARAMETERS \
    DERATE_PARAMETERS_COMMON \
    DERATE_PARAMETERS_SINE \
    CHARGER_PARAMETERS \
    AUTOMATION_CONTACT_PWM_COMM_PARAMETERS \
    PARAM_ENTRY(CAT_TEST,    fslipspnt,   "Hz",      -100,   1000,   0,      0   ) \
    PARAM_ENTRY(CAT_TEST,    ampnom,      "%",       0,      100,    0,      0   ) \
    VALUE_BLOCK1 \
    VALUES_SINE \
    VALUE_BLOCK2

#elif CONTROL == CTRL_FOC
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    MOTOR_PARAMETERS_FOC \
    MOTOR_PARAMETERS_COMMON \
    INVERTER_PARAMETERS_COMMON \
    INVERTER_PARAMETERS_FOC \
    THROTTLE_PARAMETERS_COMMON \
    THROTTLE_PARAMETERS_FOC \
    REGEN_PARAMETERS \
    DERATE_PARAMETERS_COMMON \
    CHARGER_PARAMETERS \
    AUTOMATION_CONTACT_PWM_COMM_PARAMETERS \
    PARAM_ENTRY(CAT_TEST,    manualiq,    "A",       -400,   400,    0,      0  ) \
    PARAM_ENTRY(CAT_TEST,    manualid,    "A",       -400,   400,    0,      0  ) \
    VALUE_BLOCK1 \
    VALUES_FOC \
    VALUE_BLOCK2 \

#endif // CONTROL

/***** Enum String definitions *****/
#define OPMODES      "0=Off, 1=Run, 2=ManualRun, 3=Boost, 4=Buck, 5=Sine, 6=AcHeat"
#define PWMFRQS      "0=17.6kHz, 1=8.8kHz, 2=4.4KHz"
#define PWMPOLS      "0=ActHigh, 1=ActLow"
#define DIRS         "-1=Reverse, 0=Neutral, 1=Forward"
#define TRIPMODES    "0=AllOff, 1=DcSwOn, 2=PrechargeOn, 3=AutoResume"
#define SNS_HS       "0=JCurve, 1=Semikron, 2=MBB600, 3=KTY81, 4=PT1000, 5=NTCK45_2k2, 6=Leaf, 7=BMW-i3"
#define SNS_M        "12=KTY83-110, 13=KTY84-130, 14=Leaf, 15=KTY81-110, 16=Toyota, 21=OutlanderFront, 22=EpcosB57861-S, 23=ToyotaGen2"
#define PWMFUNCS     "0=tmpm, 1=tmphs, 2=speed, 3=speedfrq"
#define CRUISEMODS   "0=Button, 1=Switch, 2=CAN, 3=ThrottlePot"
#define DIRMODES     "0=Button, 1=Switch, 2=ButtonReversed, 3=SwitchReversed, 4=DefaultForward"
#define IDLEMODS     "0=Always, 1=NoBrake, 2=Cruise, 3=Off, 4=HillHold"
#define ONOFF        "0=Off, 1=On, 2=na"
#define OKERR        "0=Error, 1=Ok, 2=na"
#define CHARGEMODS   "0=Off, 3=Boost, 4=Buck"
#define ENCMODES     "0=Single, 1=AB, 2=ABZ, 3=SPI, 4=Resolver, 5=SinCos"
#define POTMODES     "0=SingleRegen, 1=DualChannel, 2=CAN, 3=CANDual, 4=BiDir, 6=CANBiDir"
#define CANSPEEDS    "0=250k, 1=500k, 2=800k, 3=1M"
#define CANIOS       "1=Cruise, 2=Start, 4=Brake, 8=Fwd, 16=Rev, 32=Bms"
#define CANPERIODS   "0=100ms, 1=10ms"
#define HWREVS       "0=Rev1, 1=Rev2, 2=Rev3, 3=Tesla, 4=BluePill, 5=Prius"
#define SWAPS        "0=None, 1=Currents12, 2=SinCos, 4=PWMOutput13, 8=PWMOutput23"
#define STATUS       "0=None, 1=UdcLow, 2=UdcHigh, 4=UdcBelowUdcSw, 8=UdcLim, 16=EmcyStop, 32=MProt, 64=PotPressed, 128=TmpHs, 256=WaitStart"
#define CAT_MOTOR    "Motor"
#define CAT_INVERTER "Inverter"
#define CAT_THROTTLE "Throttle"
#define CAT_REGEN    "Regen"
#define CAT_AUTOM    "Automation"
#define CAT_DERATE   "Derating"
#define CAT_PWM      "Aux PWM"
#define CAT_CONTACT  "Contactor Control"
#define CAT_TEST     "Testing"
#define CAT_CHARGER  "Charger"
#define CAT_COMM     "Communication"

/***** enums ******/

#define CAN_PERIOD_100MS    0
#define CAN_PERIOD_10MS     1

#if CONTROL == CTRL_SINE
#define VERSTR STRINGIFY(4=VER-sine)
#elif CONTROL == CTRL_FOC
#define VERSTR STRINGIFY(4=VER-foc)
#endif // CONTROL

enum cruisemodes
{
   CRUISE_BUTTON = 0,
   CRUISE_SWITCH = 1,
   CRUISE_CAN = 2,
   CRUISE_POT = 3
};

enum _potmodes
{
   POTMODE_REGENADJ = 0,
   POTMODE_DUALCHANNEL = 1,
   POTMODE_CAN = 2,
   POTMODE_BIDIR = 4
};

enum _pwmfuncs
{
   PWM_FUNC_TMPM = 0,
   PWM_FUNC_TMPHS,
   PWM_FUNC_SPEED,
   PWM_FUNC_SPEEDFRQ
};

enum _idlemodes
{
   IDLE_MODE_ALWAYS = 0,
   IDLE_MODE_NOBRAKE,
   IDLE_MODE_CRUISE,
   IDLE_MODE_OFF,
   IDLE_MODE_HILLHOLD
};

enum _modes
{
   MOD_OFF = 0,
   MOD_RUN,
   MOD_MANUAL,
   MOD_BOOST,
   MOD_BUCK,
   MOD_SINE,
   MOD_ACHEAT,
   MOD_LAST
};

enum _tripmodes
{
   TRIP_ALLOFF = 0,
   TRIP_DCSWON,
   TRIP_PRECHARGEON,
   TRIP_AUTORESUME
};

enum _dirmodes
{
   DIR_BUTTON = 0,
   DIR_SWITCH = 1,
   DIR_REVERSED = 2, //used as a flag
   DIR_DEFAULTFORWARD = 4
};

enum _canio
{
   CAN_IO_CRUISE = 1,
   CAN_IO_START = 2,
   CAN_IO_BRAKE = 4,
   CAN_IO_FWD = 8,
   CAN_IO_REV = 16,
   CAN_IO_BMS = 32
};

enum _swap
{
   SWAP_CURRENTS = 1,
   SWAP_RESOLVER = 2,
   SWAP_PWM13 = 4,
   SWAP_PWM23 = 8
};

enum status
{
   STAT_NONE = 0,
   STAT_UDCLOW = 1,
   STAT_UDCHIGH = 2,
   STAT_UDCBELOWUDCSW = 4,
   STAT_UDCLIM = 8,
   STAT_EMCYSTOP = 16,
   STAT_MPROT = 32,
   STAT_POTPRESSED = 64,
   STAT_TMPHS = 128,
   STAT_WAITSTART = 256
};

//Generated enum-string for possible errors
extern const char* errorListString;

