# Prey Escape System

This repository is source code of the automated prey escape system.

`MainProgram.py` processes camera frames and sends a serial signal to Arduino to trigger motor controls.

`MotoControl.ino` is the program controls a motor on Arduino

Please reference our preprint about this system.

## System Details

This system developed with a UVC (DMK33UX287, The Imaging Source Co., Ltd., Bremen, Germany) and Arduino UNO R3. 

The main program (MainProgram.py) works on Windows 10, Python 3.7.7, Opencv-contrib 3.4.10.37, pyserial 3.4, numpy 1.18.2

The main program developed using [IC Imaging Control SDK](https://www.theimagingsource.com/en-jp/support/download/) (The Imaging Source Co., Ltd., Bremen, Germany)

For more details about IC Imaging Control SDK, please read IC_IMAGEING_README.md and 

## Usage

1. Connect a UVC and Arduino connected with motor on your Windows 10 PC (see our preprint).

2. Download or clone this repository.

3. Install Python 3.7 and required Python library (see system details).

4. Upload `MotoControl.ino` to Arduino with Arduino IDE. If you want to change motor settings, please rewrite Line 27-33.

5. Run `MainProgram.py`. If you want to change program settings, please rewrite Line 7-50.

6. Press "E" key, the program is ready to trigger motor. Press "N" key, the program continue to process camera frame but don't trigger motor. Press "Q" key, the program stop and exit. 






