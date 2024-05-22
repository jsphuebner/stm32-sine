CC		= gcc
CPP	= g++
LD		= g++
CP		= cp
CFLAGS    = -std=c99 -ggdb -DSTM32F1 -I../include -I../libopeninv/include -I../libopencm3/include
CPPFLAGS    = -ggdb -DSTM32F1 -DCTRL_FOC=1 -DCTRL_SINE=0 -DCTRL=CTRL_FOC -I../include -I../libopeninv/include -I../libopencm3/include
LDFLAGS     = -g
BINARY		= test_sine
OBJS		= test_main.o fu.o test_fu.o test_fp.o test_vcu.o my_fp.o my_string.o params.o vehiclecontrol.o \
			  test_throttle.o throttle.o sine_core.o temp_meas.o
VPATH = ../src ../libopeninv/src

# Check if the variable GITHUB_RUN_NUMBER exists. When running on the github actions running, this
# variable is automatically available.
# Create a compiler define with the content of the variable. Or, if it does not exist, use replacement value 99999.
CPPFLAGS += $(shell \
    if [ -z "$$GITHUB_RUN_NUMBER" ]; then echo "-DGITHUB_RUN_NUMBER=0"; else echo "-DGITHUB_RUN_NUMBER=$$GITHUB_RUN_NUMBER"; fi )

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS)

%.o: ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: ../%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY)
