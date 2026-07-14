# Description

The **TrackFox** is an autonomous GPS tracker. The main goals of the project were the following:

* Design a portable device with credit card format, which can be easily integrated in various assets (car, bike, hiking backpack, etc...).
* Embed an accelerometer to detect start and stop events autonomously.
* Achieve the minimum power consumption.
* Use battery-less energy harvesting such as solar panel or dynamo source with supercapacitor.
* Send data over long range IoT networks such as Sigfox.

# Hardware

The boards were designed on **Circuit Maker V1.3**. Below is the list of hardware revisions:

| Hardware revision | Description | `cmake_hw_version` | Status |
|:---:|:---:|:---:|:---:|
| [TKFX HW1.0](https://365.altium.com/files/CB5EF2D6-C92D-11EB-A2F6-0A0ABF5AFC1B) | Initial version. | `HW1_0` | :x: |
| [TKFX HW1.1](https://365.altium.com/files/C69B8131-C92D-11EB-A2F6-0A0ABF5AFC1B) | Connect S2LP shutdown pin to MCU to fix startup issues.<br>No more connection from RF TCXO to MCU. | `HW1_1` | :x: |
| [TKFX HW2.0](https://365.altium.com/files/679595FA-B106-4994-9DB4-DEC1CDA1A002) | Upgrade based on new radio front-end with passive WiFi scan feature, new GPS module and improved power management compatible with battery or supercap. | `HW2_0` | :white_check_mark: |

# Embedded software

## Environment

As of version `sw0.0.9` the embedded software is developed under **Eclipse IDE** version 2024-09 (4.33.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

> [!WARNING]
> To compile any version under `sw4.0`, the `git_version.sh` script must be patched when `sscanf` function is called: the `SW` prefix must be replaced by `sw` since Git tags have been renamed in this way.

## Target

The TrackFox boards are based on the **STM32L051x8U6** microcontroller of the STMicroelectronics L0 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

## Structure

The project is organized as follow:

* `drivers` :
    * `device` : MCU **startup** code and **linker** script.
    * `registers` : MCU **registers** address definition.
    * `registers` : MCU **registers** address definition.
    * `peripherals` : internal MCU **peripherals** drivers.
    * `components` : external **components** drivers.
    * `utils` : **utility** functions.
* `middleware` :
    * `analog` : High level **analog measurements** driver.
    * `cli` : **AT commands** implementation.
    * `gps` : High level **GPS** driver.
    * `power` : Board **power tree** manager.
    * `sigfox` : **Sigfox EP_LIB** and **ADDON_RFP** submodules and low level implementation.
    * `wifi` : High level **WiFi scan** driver.
* `application` : Main **application**.

## Sigfox library

**Sigfox technology** is very well suited for this application for 3 main reasons:

* **Data quantity is low**, position and monitoring data can be packaged on a few bytes and does not require high speed transmission.
* **Low power** communication enable **energy harvesting** (solar cell + supercap in this case), so that the device is autonomous.
* The tracker can operate is very isolated places (mountains, etc...) thanks to the **long range** performance.

The project is based on the [Sigfox end-point open source library](https://github.com/sigfox-tech-radio/sigfox-ep-lib) which is embedded as a **Git submodule**.

## Build

The project can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="script/cmake-arm-none-eabi/toolchain.cmake" \
      -DTOOLCHAIN_PATH="<arm_none_eabi_gcc_path>" \
      -DTKFX_HW_VERSION="<cmake_hw_version>" \
      -DTKFX_MODE_CLI=OFF \
      -DTKFX_MODE_CAR=ON \
      -DTKFX_MODE_BIKE=OFF \
      -DTKFX_MODE_HIKING=OFF \
      -DTKFX_MODE_MOTO=OFF \
      -DTKFX_MODE_BATTERY=ON \
      -DTKFX_MODE_SUPERCAPACITOR=OFF \
      -G "Unix Makefiles" ..
make all
```

## Flash

### Preparation

* **Build** the desired version (with IDE or `cmake`) or **download** a specific [firmware release](https://github.com/Ludovic-Lesur/trackfox-tkfx/releases) (expand the `Assets` menu, download the corresponding artifact and extract the binary files from the `zip`).
* Connect the flashing tool to the **P4** (HW1.x) or **P2** (HW2.0) **connector** located in the edge of the PCB (standard SWD pinout).

### ST-Link on Nucleo board

* Make sure that the ST-LINK/NUCLEO jumpers (generally designated by **CN2**) are not fitted, in order to **select the external programming connector** instead of the internal MCU.
* An **MSC disk** named `NODE_XXXXXX` should be mounted by the system after USB plugging. If not, download the [ST Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) software which will install the required drivers. If the MSC disk is still not mounted, follow the ST-Link probe procedure thereafter.
* **Copy/paste** or **click/drop** the `bin` file into the disk.

### ST-Link probe

* Download the [ST Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) software.
* Launch the software (it might be necessary to run it as **root** or to install specific **USB rules** for the probe to be recognized).
* In the right panel, select `ST-LINK` and click `Connect`.
* Click on the `Open file` tab and select the `hex` file to flash.
* Click on the `Download` button.
* Perform a **memory check** with the `Verify` button located under the `Download` button menu.
* If the operation completed successfully, click on `Disconnect` in the right panel.

### Segger J-Link probe

* Download the [Segger J-Link](https://www.segger.com/downloads/jlink/) software.
* Launch the `JFlashLite` tool.
* Set target device to **STM32L051K8** (HW1.x) or **STM32L051CB** (HW2.0), target interface to **SWD**, speed to **4000kHz** and click `OK`.
* Open the `hex` file to flash.
* Click on the `Program Device` button.

### Final steps

* Check on the platform if the tracker has properly rebooted with the **expected firmware version**.
