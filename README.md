# Prey Escape System

This repository is source code of the automated prey escape system.

`MainProgram.py` processes camera frames and sends a serial signal to Arduino to trigger motor controls.

`MotorControl.ino` is the program controls a motor on Arduino

Please refer to our preprint about this system.

Preprint: https://www.biorxiv.org/content/10.1101/2023.07.02.547369v1

## System Details

This system was developed using a UVC (DMK33UX287, The Imaging Source Co., Ltd., Bremen, Germany) and Arduino UNO R3. 

The main program (MainProgram.py) works on Windows 10, Python 3.7.7, Opencv-contrib 3.4.10.37, pyserial 3.4, numpy 1.18.2

The main program was developed using [the IC Imaging Control SDK](https://www.theimagingsource.com/en-jp/support/download/) (The Imaging Source Co., Ltd., Bremen, Germany)

For more details about IC Imaging Control SDK, please refer to [IC_IMAGEING_README.md](https://github.com/YuukiKawabata-Lab/PreyEscapeSystem/blob/main/IC_IMAGEING_README.md)

## Usage

1. Connect a UVC camera and Arduino, which is connected to a motor, to your Windows 10 PC (refer to our preprint for details).

2. Download or clone this repository.

3. Install Python 3.7 and the required Python libraries (see system details).

4. Upload `MotorControl.ino` to Arduino using the Arduino IDE. If you want to change motor settings, please modify lines 27-33.

5. Run `MainProgram.py`. If you want to change program settings, please modify lines 7-50.

6. Press the "E" key to activate the program and trigger the motor. Press the "N" key to continue processing camera frames without triggering the motor. Press the "Q" key to stop the program and exit.
