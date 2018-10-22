# stm32-sine
Main firmware of the Huebner inverter project

# Compiling
The only external depedency if libopencm3 which I forked.
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads

Compile libopencm3 and copy the output to your compiler directory.
Example: if compiler resides in /home/user/bin/gcc-arm
cp -r include/libopencm3 /home/user/bin/gcc-arm/arm-none-eabi/include
cp lib/libopencm3_stm32f1.a /home/user/bin/gcc-arm/arm-none-eabi/lib

Now you can compile stm32-sine and upload it to your board using a
JTAG/SWD adapter, the updater.py script or the esp8266 web interface
