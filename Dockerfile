FROM node:gallium-bullseye-slim

RUN apt update && \
    apt install -y git gcc cmake gcc-arm-none-eabi cmake-curses-gui build-essential python3 && \
    git clone --recursive -b master https://github.com/raspberrypi/pico-sdk.git && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /tmp/*

