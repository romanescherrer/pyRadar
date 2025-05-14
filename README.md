# ADC/UART data capturing using xWR1843/AWR2243 with DCA1000

- for xWR1843: capture both raw ADC IQ data and processed UART point cloud data simultaneously in Python and C(pybind11) without mmwaveStudio
- for AWR2243: capture raw ADC IQ data in Python and C(pybind11) without mmwaveStudio

## cfg file

chirpCfg <`chirpStartIdx`> <`chirpEndIdx`> <`profileId`> <`startFreqVar`> <`freqSlopeVar`> <`idleTimeVar`> <`adcStartTimeVar`> <`txEnable`>

## Introduction

This module mainly consists of two parts: mmwave and fpga_udp:

- mmwave is modified from [OpenRadar](https://github.com/PreSenseRadar/OpenRadar) and is responsible for configuration file reading, serial data transmission and reception, and raw data parsing.
- fpga_udp is modified from the [pybind11 example](https://github.com/pybind/python_example) and [mmWave-DFP-2G](https://www.ti.com/tool/MMWAVE-DFP). It handles high-speed raw data reception from the DCA1000 through Ethernet using C socket programming. For AWR2243, which lacks an on-chip DSP and ARM core, it also implements firmware writing and parameter configuration via SPI over FTDI USB.

TI's mmWave radars are categorized into two types: those with only RF front-end and those with an on-chip ARM, DSP, and HWA.

- RF front-end only models: AWR1243, AWR2243, etc.
- On-chip ARM and DSP models: xWR1443, xWR6443, xWR1843, xWR6843, AWR2944, etc.

## Prerequisites

### Hardware

#### for xWR1843

- Connect the micro-USB port (UART) on the xWR1843 to your system
- Connect the xWR1843 to a 5V barrel jack
- Set power connector on the DCA1000 to RADAR_5V_IN
- boot in Functional Mode: SOP[2:0]=001
  - either place jumpers on pins marked as SOP0 or toggle SOP0 switches to ON, all others remain OFF
- Connect the RJ45 to your system
- Set a fixed IP to the local interface: 192.168.33.30

#### for AWR2243

- Connect the micro-USB port (FTDI) on the DCA1000 to your system
- Connect the AWR2243 to a 5V barrel jack
- Set power connector on the DCA1000 to RADAR_5V_IN
- Put the device in SOP0
  - Jumper on SOP0, all others disconnected
- Connect the RJ45 to your system
- Set a fixed IP to the local interface: 192.168.33.30

### Software

#### Windows

- Microsoft Visual C++ 14.0 or greater is required.
  - Get it with "[Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)"(Standalone MSVC compiler) or "[Visual Studio](https://visualstudio.microsoft.com/downloads/)"(IDE) and choose "Desktop development with C++"
- FTDI D2XX driver and DLL is needed.
  - Download version [2.12.36.4](https://www.ftdichip.com/Drivers/CDM/CDM%20v2.12.36.4%20WHQL%20Certified.zip) or newer from [official website](https://ftdichip.com/drivers/d2xx-drivers/).
  - Unzip it and install `.\ftdibus.inf` by right-clicking this file.
  - Copy `.\amd64\ftd2xx64.dll` to `C:\Windows\System32\` and rename it to `ftd2xx.dll`. For 32-bit system, just copy `.\i386\ftd2xx.dll` to that directory.

#### Linux

- `sudo apt install python3-dev`
- FTDI D2XX driver and .so lib is needed. Download version 1.4.27 or newer from [official website](https://ftdichip.com/drivers/d2xx-drivers/) based on your architecture, e.g. [X86](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-x86_32-1.4.27.tgz), [X64](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-x86_64-1.4.27.tgz), [armv7](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-arm-v7-hf-1.4.27.tgz), [aarch64](https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-arm-v8-1.4.27.tgz), etc.
- Then you'll need to install the library:
  - ```
    tar -xzvf libftd2xx-arm-v8-1.4.27.tgz
    cd release
    sudo cp ftd2xx.h /usr/local/include
    sudo cp WinTypes.h /usr/local/include
    cd build
    sudo cp libftd2xx.so.1.4.27 /usr/local/lib
    sudo chmod 0755 /usr/local/lib/libftd2xx.so.1.4.27
    sudo ln -sf /usr/local/lib/libftd2xx.so.1.4.27 /usr/local/lib/libftd2xx.so
    sudo ldconfig -v
    ```

## Installation

- clone this repository
- for Windows:
  - `python3 -m pip install --upgrade pip`
  - `python3 -m pip install --upgrade setuptools`
  - `python3 -m pip install ./fpga_udp`
- for Linux:
  - `sudo python3 -m pip install --upgrade pip`
  - `sudo python3 -m pip install --upgrade setuptools`
  - `sudo python3 -m pip install ./fpga_udp`

## Instructions for Use

#### General

Follow the Prerequisites to set up the environment.

Install the required libraries as described in Installation.

Run the necessary modules as per requirement.

1.  Follow the [Prerequisites](#prerequisites) to set the environment
2.  Install the required libraries as described in [Installation](#installation)
3.  Run the necessary modules as per requirement.

#### For xWR1843

1.  Flash the `xwr18xx_mmw_demo` firmware as per the [mmwave SDK](https://www.ti.com/tool/MMWAVE-SDK) instructions
2.  Configure parameters using [mmWave_Demo_Visualizer](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/) and save the `.cfg` file.
3.  Modify the [captureAll.py](#captureallpy) with the saved `.cfg` file to start capture.
4.  Use [testDecode.ipynb](#testdecodeipynb) or [testDecodeADCdata.mlx](#testdecodeadcdatamlx) to analyze the captured data.
5.  Adjust parameters as needed using [testParam.ipynb](#testparamipynb) or by `mmWave_Demo_Visualizer`

#### For AWR2243

1.  Flash the firmware to external flash (this needs to be done only once):

- for Windows: `python3 -c "import fpga_udp;fpga_udp.AWR2243_firmwareDownload()"`
- for Linux: `sudo python3 -c "import fpga_udp;fpga_udp.AWR2243_firmwareDownload()"`
- If you see "MSS Patch version [ 2. 2. 2. 0]", the flashing was successful.

2.  Modify and run [captureADC_AWR2243.py](#captureadc_awr2243py) with the required `.txt` config file
3.  Use [testDecode_AWR2243.ipynb](#testdecode_awr2243ipynb) to analyse the captured data
4.  Adjust parameters as needed using [testParam_AWR2243.ipynb](#testparam_awr2243ipynb)

## Example

### **_captureAll.py_**

Example code for simultaneously capturing raw ADC sampled IQ data and processed point cloud data from the onboard DSP via UART (only for xWR1843).

#### 1. General workflow for capturing raw data

1. Reset the radar and DCA1000 (`reset_radar`, `reset_fpga`).
2. Initialize the radar and configure parameters via UART (`TI`, `setFrameCfg`).
3. _(Optional)_ Create a process to receive processed data from the onboard DSP via UART (`create_read_process`).
4. Send FPGA configuration commands via UDP (`config_fpga`).
5. Send record configuration commands via UDP (`config_record`).
6. _(Optional)_ Start the UART reception process (only for buffer clearing) (`start_read_process`).
7. Send a start streaming command via UDP (`stream_start`).
8. Start a UDP data reception thread (`fastRead_in_Cpp_async_start`).
9. Start the radar via UART (or theoretically via FTDI (USB-to-SPI), currently only implemented for AWR2243) (`startSensor`).
10. Wait for the UDP data reception thread to complete and parse the raw data (`fastRead_in_Cpp_async_wait`).
11. Save the raw data to a file for offline processing (`tofile`).
12. _(Optional)_ Send a stop streaming command via UDP (`stream_stop`).
13. Stop the radar via UART (`stopSensor`) or reset it via UDP (`reset_radar`).
14. _(Optional)_ Stop receiving UART data (`stop_read_process`).
15. _(Optional)_ Parse processed point cloud data received from UART (`post_process_data_buf`).

#### 2. Requirements for "\*.cfg" mmWave radar configuration file

- The default profile in Visualizer disables LVDS streaming.
- To enable it, export the chosen profile and set the appropriate enable bits.
- `adcbufCfg` must be set as follows, and the third parameter of `lvdsStreamCfg` must be set to 1. Refer to _mmwave_sdk_user_guide.pdf_ for details:
  - `adcbufCfg -1 0 1 1 1`
  - `lvdsStreamCfg -1 0 1 0`

#### 3. Requirements for "cf.json" data capture card configuration file

- For detailed information, refer to _TI_DCA1000EVM_CLI_Software_UserGuide.pdf_.
- **LVDS Mode:**
  - Specifies the lane configuration for LVDS. Valid only when `dataTransferMode` is set to `"LVDSCapture"`.
  - Valid options:
    - `1` (4 lanes)
    - `2` (2 lanes)
- **Packet Delay:**
  - By default, Ethernet throughput varies up to 325 Mbps with a 25-µs Ethernet packet delay.
  - The user can modify the Ethernet packet delay between 5 µs and 500 µs to achieve different throughputs:
    - `"packetDelay_us":  5 (us)   ~   706 Mbps`
    - `"packetDelay_us": 10 (us)   ~   545 Mbps`
    - `"packetDelay_us": 25 (us)   ~   325 Mbps`
    - `"packetDelay_us": 50 (us)   ~   193 Mbps`

### **_captureADC_AWR2243.py_**

Example code for capturing raw ADC sampled IQ data (only for AWR2243).

#### 1. General workflow for capturing raw data with AWR2243

1. Reset the radar and DCA1000 (`reset_radar`, `reset_fpga`).
2. Initialize the radar via SPI and configure parameters (`AWR2243_init`, `AWR2243_setFrameCfg`). (_Requires root privileges on Linux_)
3. Send FPGA configuration commands via UDP (`config_fpga`).
4. Send record configuration commands via UDP (`config_record`).
5. Send a start streaming command via UDP (`stream_start`).
6. Start a UDP data reception thread (`fastRead_in_Cpp_async_start`).
7. Start the radar via SPI (`AWR2243_sensorStart`).
8. _(Optional, required if `numFrame == 0`)_ Stop the radar via SPI (`AWR2243_sensorStop`).  
   _(Optional, required if `numFrame != 0`)_ Wait for the radar to complete data capture (`AWR2243_waitSensorStop`).
9. _(Optional, required if `numFrame == 0`)_ Send a stop streaming command via UDP (`stream_stop`).
10. Wait for the UDP data reception thread to complete and parse the raw data (`fastRead_in_Cpp_async_wait`).
11. Save the raw data to a file for offline processing (`tofile`).
12. Power off the radar and clear configurations via SPI (`AWR2243_poweroff`).

#### 2. Requirements for "mmwaveconfig.txt" mmWave radar configuration file

- TBD

#### 3. Requirements for "cf.json" data capture card configuration file

- Same as above.

### **_realTimeProc.py_**

Example code for real-time looped acquisition of raw ADC sampled IQ data and online processing (only for xWR1843).

#### 1. General workflow for capturing raw data

1. Reset the radar and DCA1000 (`reset_radar`, `reset_fpga`).
2. Initialize the radar via UART and configure parameters (`TI`, `setFrameCfg`).
3. Send FPGA configuration commands via UDP (`config_fpga`).
4. Send record configuration commands via UDP (`config_record`).
5. Send a start streaming command via UDP (`stream_start`).
6. Start the radar via UART (`startSensor`).  
   *(*In theory, it can also be controlled via FTDI (USB-to-SPI), but this is currently only implemented for AWR2243.)\*
7. **Loop:** Receive UDP data packets + parse raw data + real-time data processing (`fastRead_in_Cpp`, `postProc`).
8. Stop the radar via UART (`stopSensor`).
9. Send stop commands via UDP (`fastRead_in_Cpp_thread_stop`, `stream_stop`).

#### 2. Requirements for "mmwaveconfig.txt" mmWave radar configuration file

- Omitted.

#### 3. Requirements for "cf.json" data capture card configuration file

- Omitted.

### **_realTimeProc_AWR2243.py_**

Example code for real-time cyclic collection of raw ADC sampled IQ data and online processing (only for AWR2243).

#### 1. General process for collecting raw data with AWR2243

- Omitted

#### 2. "mmwaveconfig.txt" mmWave radar configuration file requirements

- TBD

#### 3. "cf.json" data capture card configuration file requirements

- Omitted

### **_testDecode.ipynb_**

Example code for parsing raw ADC sampled data and UART data (only for IWR1843).  
Requires Jupyter (recommended: install Jupyter plugin in VS Code).

#### 1. Parsing ADC raw IQ data received via LVDS

##### Using NumPy to parse ADC raw IQ data received via LVDS

- Load necessary libraries.
- Set corresponding parameters.
- Load and parse the saved bin data.
- Plot time-domain IQ waveform.
- Compute Range-FFT.
- Compute Doppler-FFT.
- Compute Azimuth-FFT.

##### Using functions provided by `mmwave.dsp` to parse ADC raw IQ data received via LVDS

#### 2. Parsing UART-received processed data from the on-chip DSP (point cloud, Doppler, etc.)

- Load necessary libraries.
- Load the saved UART-parsed data.
- Display configuration file settings.
- Show on-chip processing time (can be used to determine if frame rate adjustments are needed).
- Display temperatures of each antenna.
- Display packet information.
- Display point cloud data.
- Compute range labels and Doppler velocity labels.
- Display range profile and noise floor profile.
- Display Doppler Bins.
- Display Azimuth (Angle) Bins.

---

### **_testDecode_AWR2243.ipynb_**

Example code for parsing raw ADC sampled data (only for AWR2243).  
Requires Jupyter (recommended: install Jupyter plugin in VS Code).

#### 1. Parsing ADC raw IQ data received via LVDS

##### Using NumPy to parse ADC raw IQ data received via LVDS

- Load necessary libraries.
- Set corresponding parameters.
- Load and parse the saved bin data.
- Plot time-domain IQ waveform.
- Compute Range-FFT.
- Compute Doppler-FFT.
- Compute Azimuth-FFT.

---

### **_testParam.ipynb_**

Verification of mmWave radar configuration parameters for IWR1843.  
Requires Jupyter (recommended: install Jupyter plugin in VS Code).

- Mainly verifies whether the parameters in the mmWave radar `.cfg` file and the DCA capture board `cf.json` file are configured correctly.
- Constraints are based on the IWR1843's device characteristics. Refer to the IWR1843 datasheet, mmWave SDK user manual, and chirp programming guide.
- If parameters meet constraints, debugging information is displayed in cyan; if not, messages appear in purple or yellow.
- Note: The constraints in this program are not perfectly accurate. Even if all parameters meet the constraints, there is still a chance the system might not operate correctly in special cases.

---

### **_testParam_AWR2243.ipynb_**

Same as above, but for verifying AWR2243 mmWave radar configuration parameters.

---

### **_testDecodeADCdata.mlx_**

MATLAB example code for parsing raw ADC sampled data.

- Set corresponding parameters.
- Load and parse saved bin raw ADC data.
- Reconstruct data format based on parameters.
- Plot time-domain IQ waveform.
- Compute Range-FFT (1D FFT + static clutter removal).
- Compute Doppler-FFT.
- Apply 1D-CA-CFAR Detector on Range-FFT.
- Compute Azimuth-FFT.

### **_testGtrack.py_**

利用 cppyy 测试 C 语言写的 gTrack 算法。该算法为 TI 的群目标跟踪算法，输入点云，输出轨迹。

The algorithm is designed to track multiple targets, where each target is represented by a set of measurement points.
Each measurement point carries detection informations, for example, range, azimuth, elevation (for 3D option), and radial velocity.

Instead of tracking individual reflections, the algorithm predicts and updates the location and dispersion properties of the group.

The group is defined as the set of measurements (typically, few tens; sometimes few hundreds) associated with a real life target.

Algorithm supports tracking targets in two or three dimensional spaces as a build time option:

- When built with 2D option, algorithm inputs range/azimuth/doppler information and tracks targets in 2D cartesian space.
- When built with 3D option, algorithm inputs range/azimuth/elevation/doppler information and tracks targets in 3D cartesian space.

#### Input/output

- Algorithm inputs the Point Cloud. For example, few hundreds of individual measurements (reflection points).
- Algorithm outputs a Target List. For example, an array of few tens of target descriptors. Each descriptor carries a set of properties for a given target.
- Algorithm optionally outputs a Target Index. If requested, this is an array of target IDs associated with each measurement.

#### Features

- Algorithm uses extended Kalman Filter to model target motion in Cartesian coordinates.
- Algorithm supports constant velocity and constant acceleartion models.
- Algorithm uses 3D/4D Mahalanobis distances as gating function and Max-likelihood criterias as scoring function to associate points with an existing track.

## Software Architecture

### TBD.py

TBD
