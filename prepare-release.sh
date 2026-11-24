#!/bin/bash

# Script to compile all released environments and copy build artifacts to firmware/targets/ for easy upload to github releases.

mkdir -p firmware/targets

pio run -e m328p_core_lcd
cp .pio/build/m328p_core_lcd/firmware.hex firmware/targets/openevse_core_lcd.hex

pio run -e samd
cp .pio/build/samd/firmware.bin firmware/targets/arm_samd_openevse_nxt.bin

ls firmware/targets/

echo "Done"
