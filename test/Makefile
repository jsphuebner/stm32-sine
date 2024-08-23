CC		= gcc
CPP	= g++
LD		= g++
CP		= cp
CFLAGS    = -std=c99 -ggdb -DSTM32F1 -I../include -I../libopeninv/include -I../libopencm3/include
CPPFLAGS    = -ggdb -DSTM32F1 -DCONTROL_FOC=1 -DCONTROL_SINE=0 -DCONTROL=DCONTROL_FOC -I../include -I../libopeninv/include -I../libopencm3/include
LDFLAGS     = -g
BINARY		= test_sine
OBJS		= test_main.o fu.o test_fu.o test_fp.o test_vcu.o my_fp.o my_string.o params.o vehiclecontrol.o \
			  test_throttle.o throttle.o sine_core.o temp_meas.o
VPATH = ../src ../libopeninv/src

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS)

%.o: ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: ../%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY)
