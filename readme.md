# Summary
The TrackFox is an autonomous GPS tracker. The main goals of the project were the following:
* Design a portable device with credit card format, which can be easily integrated in various assets (car, bike, hiking backpack, etc...).
* Embed an accelerometer to detect start and stop events autonomously.
* Achieve the minimum power consumption.
* Use battery-less energy harvesting such as solar panel or dynamo source with supercapacitor.
* Send data over long range IoT networks such as Sigfox.

# Hardware
The board was designed on **Circuit Maker V1.3**. Hardware documentation and design files are available @ https://circuitmaker.com/Projects/Details/Ludovic-Lesur/TKFXHW1-1

# Embedded software

## Environment
The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target
The TrackFox board is based on the **STM32L041K6U6** of the STMicroelectronics L0 family microcontrollers. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected target.

## Structure
The project is organized as follow:
* `inc` and `src`: **source code** split in 6 layers:
    * `registers`: MCU **registers** adress definition.
    * `peripherals`: internal MCU **peripherals** drivers.
    * `utils`: **utility** functions.
    * `components`: external **components** drivers.
    * `sigfox`: **Sigfox library** API and low level implementation.
    * `applicative`: high-level **application** layers.
* `lib`: **Sigfox protocol library** files.
* `startup`: MCU **startup** code (from ARM).
* `linker`: MCU **linker** script (from ARM).

## Sigfox library

Sigfox technology is very well suited for this application for 3 main reasons:
* Data quantity is low, position and monitoring data can be packaged on a few bytes and does not require high speed transmission.
* Low power communication enable energy harvesting (solar cell + supercap in this case), so that the device is autonomous.
* The tracker can operate is very isolated places (mountains, etc...) thanks to the long range performance.

The Sigfox library is a compiled middleware which implements Sigfox protocol regarding framing, timing and RF frequency computation. It is based on low level drivers which depends on the hardware architecture (MCU and transceiver). Once implemented, the high level API exposes a simple interface to send messages over Sigfox network.

Last version of Sigfox library can be downloaded @ https://build.sigfox.com/sigfox-library-for-devices

For this project, the Cortex-M0+ version compiled with GCC is used.
