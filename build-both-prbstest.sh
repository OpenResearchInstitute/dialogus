#!/bin/sh
# This script can run on a machine with the cross-development tools installed,
# or it can run inside a container that provides the cross-development tools.
# To build the example script inside the provided container, use:
#     docker run --platform=linux/amd64 --volume .:/repo build-application /bin/sh build-both-loopback-rx.sh

arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D NO_INIT_ON_SUCCESS -D RF_LOOPBACK -o bin/msk_prbstest-xmit opulent_voice.c -lpthread -liio -lm -Wall -Wextra
arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D NO_INIT_ON_SUCCESS -D RX_ACTIVE -o bin/msk_prbstest-recv opulent_voice.c -lpthread -liio -lm -Wall -Wextra
