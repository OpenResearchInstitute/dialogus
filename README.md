# pluto-msk-application
C-code applications for the PLUTO MSK implementation 

## how-to:

arm-linux-gnueabihf-gcc -mfloat-abi=hard  --sysroot=$HOME/pluto-0.38.sysroot -std=gnu99 -g -o msk_test msk_test.c -lpthread -liio -lm -Wall -Wextra

scp msk_test root@pluto.local:/tmp/msk_test

ssh -t root@pluto.local /tmp/msk_test

## What is what

Mainline development is in msk_rx_init.c

