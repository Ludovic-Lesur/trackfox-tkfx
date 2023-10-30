# Description

The **TrackFox** is an autonomous GPS tracker. The main goals of the project were the following:

* Design a portable device with credit card format, which can be easily integrated in various assets (car, bike, hiking backpack, etc...).
* Embed an accelerometer to detect start and stop events autonomously.
* Achieve the minimum power consumption.
* Use battery-less energy harvesting such as solar panel or dynamo source with supercapacitor.
* Send data over long range IoT networks such as Sigfox.

# Hardware

The boards were designed on **Circuit Maker V1.3**. Below is the list of hardware revisions:

| Hardware revision | Description | Status |
|:---:|:---:|:---:|
| [TKFX HW1.0](https://365.altium.com/files/CB5EF2D6-C92D-11EB-A2F6-0A0ABF5AFC1B) | Initial version. | :x: |
| [TKFX HW1.1](https://365.altium.com/files/C69B8131-C92D-11EB-A2F6-0A0ABF5AFC1B) | Connect S2LP shutdown pin to MCU to fix startup issues.<br>No more connection from RF TCXO to MCU. | :white_check_mark: |

# Embedded software

## Environment

The embedded software was developed under **Eclipse IDE** version 2019-06 (4.12.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target

The TrackFox boards are based on the **STM32L041K6U6** microcontroller of the STMicroelectronics L0 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

## Structure

The project is organized as follow:

* `inc` and `src`: **source code** split in 6 layers:
    * `registers`: MCU **registers** adress definition.
    * `peripherals`: internal MCU **peripherals** drivers.
    * `utils`: **utility** functions.
    * `components`: external **components** drivers.
    * `sigfox`: **Sigfox EP library** API and low level implementation.
    * `applicative`: high-level **application** layers.
* `lib`: **Sigfox EP_LIB and ADDON_RFP** submodules.
* `startup`: MCU **startup** code (from ARM).
* `linker`: MCU **linker** script (from ARM).

## Sigfox library

**Sigfox technology** is very well suited for this application for 3 main reasons:

* **Data quantity is low**, position and monitoring data can be packaged on a few bytes and does not require high speed transmission.
* **Low power** communication enable **energy harvesting** (solar cell + supercap in this case), so that the device is autonomous.
* The tracker can operate is very isolated places (mountains, etc...) thanks to the **long range** performance.

The project is based on the [Sigfox end-point open source library](https://github.com/sigfox-tech-radio/sigfox-ep-lib) which is embedded as a **Git submodule**.
