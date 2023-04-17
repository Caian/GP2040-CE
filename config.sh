rm -rf build && \
mkdir  build && \
cd     build && \
ccmake -DPICO_SDK_PATH=/pico-sdk/ -DPICO_BOARD=pico ..
