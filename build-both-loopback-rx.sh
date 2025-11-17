#!/bin/sh
# This script can run on a machine with the cross-development tools installed,
# or it can run inside a container that provides the cross-development tools.
# To build the example script inside the provided container, use:
#     docker run --platform=linux/amd64 --volume .:/repo build-application /bin/sh build-both-loopback-rx.sh

arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D RF_LOOPBACK -o bin/msk_test-xmit msk_rx_init.c -lpthread -liio -lm -Wall -Wextra
arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D RX_ACTIVE -o bin/msk_test-recv msk_rx_init.c -lpthread -liio -lm -Wall -Wextra
