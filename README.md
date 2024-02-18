[![Build status](../../actions/workflows/CI-build.yml/badge.svg)](../../actions/workflows/CI-build.yml)

# stm32-sine
Main firmware of the Huebner inverter project
This firmware runs on any revision of the "Huebner" hardware https://github.com/jsphuebner/inverter-hardware as well as any derivatives as the Open Source Tesla controller https://github.com/damienmaguire

# Goals
The main goal of this firmware is well-drivable control of electric 3-phase motors with as little software complexity as possible. We do not rely on virtual control methods such as FOC (field oriented control) or DTC (direct torque control). This makes tuning more intuitive, as only real physical quantities are parametrized.
The same principle is applied to the hardware design, keeping component count low and therefor minimize cost and failure modes.
To fine tune the driving experience and adapt to different flavours of power stages, over 60 parameters can be customized.

# Motor Control Concept
The idea is that the dynamics of any 3-phase asynchronous motor are controlled by the amplitude of the sythesized sine wave and its frequency offset to the rotor speed (slip). 
For 3-phase synchronous motors a similar control method did not prove practical. Therefor a FOC version of the software has been created. It shares 95% of the code.

# Inverter charging
A unique feature of this software is to re-purpose the drivetrain hardware as a programmable battery charger. One of the motor phase windings is being used as a high current capable inductor and one of the phase switches as a buck or boost converter. This has practically proven to replace a separate charging unit and further reduce complexity of electric vehicles.

# Further reading
A comprehensive guide to the Huebner inverter system can be found here: https://openinverter.org/docs

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
On Ubuntu type

`sudo apt-get install git gcc-arm-none-eabi`

The only external depedency is libopencm3 which I forked. You can download and build this dependency by typing

`make get-deps`

Now you can compile stm32-sine by typing

`make`

or

`CONTROL=FOC make`

to build the FOC version for synchronous motors.

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface
