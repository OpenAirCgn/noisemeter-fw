# Noisemeter firmware

This repository contains the Noisemeter firmware. The software reads an analog microphone connected throught the onboard OPAMP with a sampling rate of 48kHz, 16x oversampling. This results in a nominal resolution of 48KHz, 16 bit. Samples are buffered using DMA, then A- and C-weighted using CMSIS-DSP biquads (float variant). The results are collected to 1/8s chunks, then the signal RMS is calculated and converted to decibels. The results can be read through I2C.

## Building

The software framework was generated with ST CubeMX, but it does not need to be re-generated. The arm-none-eabi toolchain is required. It can be built with "make". If dfu-util is installed, the device can be connected to the host via USB in DFU mode (hold BOOT button during connect or reset) and firmware can be flashed with "make flash".

