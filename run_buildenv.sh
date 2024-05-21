#!/bin/bash

docker run -it --rm --privileged \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /run/udev:/run/udev \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /home/gustavo/uros_esp32_project/uros_espidf:/esp32_project \
    -w /esp32_project \
    espressif/idf:v5.1.4

