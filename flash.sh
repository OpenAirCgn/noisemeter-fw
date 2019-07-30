#!/bin/bash

dfu-util -d 0x0483:0xdf11 -a 0 -s 0x8000000 -D build/noisemeter-usbaudio.bin

