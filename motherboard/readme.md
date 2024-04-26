# Motherboard

The "motherboard" is the PCB (circuit board) that allows our codebase to communicate between 6 PWM-compatible devices (such as servo motors), 6 stepper motors, control the rover status-lights, and communicate with the GPS module. 

Since these things do not have a convient USB port sticking out of them to connect to our main computer for us to use, we use the Raspberry Pi Pico microcontroller to act as the middle-man between these low level devices and our codebase. All communications are made through the USB port on the Pi Pico. 


## Initial Setup
When working on the code for the Pi Pico, you must have the pic-sdk downloaded in order to be able to compile the code. We must download this.


In the root directory, `./urc-2023`, run the following commands to download the pico-sdk. 

`git submodule init` -> This initlaizes your local configration file

`git submodule update` -> Fetches all the data from the pico-sdk repo

#

Now set up the development environment in the `/motherboard` directory. Run the following commands.

`./setup_cmake.sh` -> A folder by the name of `build` should be created under the `motherboard` directory

## Organization
All source files, such as .c or .cpp, are to be under `/src`. The main file should be alone, while all other source files are to be under folders.

All header files, such as .h or .hpp, are to be under `/header`. The folders there should mimic the folder structure of the source folder.

Example folder structure
```
motherboard\
└──src\
    ├──main.cpp
    ├── folderA\
    │   ├── sourceA.cpp
    │   └── sourceAA.c
    └── folderB\
        ├── sourceB.cpp
        └── sourceBB.c
└──header\
    ├──folderA\
    │   ├── sourceA.hpp
    │   └── sourceAA.h
    └── folderB\
        ├── sourceB.hpp
        └── sourceBB.h

```


## Uploading Code to Raspberry Pi Pico

After finishing writing your amazin code, build the code by running the following commands.

`./build.sh`

If you added source/header files, you must add the individual files in `CMakeLists.txt` and execute `setup_cmake.sh` again. You might have to delete the `/build` folder to properly add files. 

# 

Grab a USB-MicroUSB cable, connect one end of the USB to the computer in which you built your code. Connect the other end of the USB while holding the button on the Pi Pico. Holding the button forces the Pi Pico into flashing mode. Your computer should have a folder called `RPI-RP2 (G:)` in your connected devices.

In your WSL shell, change your current working directory to the motherboard.
Execute the following lines.

`explorer.exe .` -> Opens the WSL Linux directory in your File Explorer.

Open the `build` folder, and drag out the `pico_motherboard.uf2` file into the `RPI-RP2 (G:)` folder. This will then flash the Pi Pico. 