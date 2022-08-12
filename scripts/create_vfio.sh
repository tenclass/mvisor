#!/bin/sh
# echo 1 > /sys/bus/mdev/devices/c2e088ba-954f-11ec-8584-525400666f2b/remove
echo "c2e088ba-954f-11ec-8584-525400666f2b" > /sys/class/mdev_bus/0000:3b:00.0/mdev_supported_types/nvidia-231/create
