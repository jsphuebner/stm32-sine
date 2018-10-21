CC		= gcc
CPP	= g++
LD		= g++
CP		= cp
CFLAGS    = -std=c99 -g -I../include/generic -I../include/project
CPPFLAGS    = -g -I../include/generic -I../include/project
LDFLAGS     = -g
BINARY		= test_sine
OBJS		= test_main.o fu.o test_fu.o test_fp.o my_fp.o my_string.o test_throttle.o throttle.o sine_core.o
VPATH = ../src/project ../src/generic

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS)

%.o: ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: ../%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY)
