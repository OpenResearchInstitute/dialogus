# pluto-msk-application
C-code applications for the PLUTO MSK implementation 

## how-to:

See section below for how to build the application.

To move the built application (`msk_test`) to the Pluto:
```
scp msk_test root@pluto.local:/tmp/msk_test
```

To execute the application (`msk_test`) on the Pluto with output streamed to your terminal:
```
ssh -t root@pluto.local /tmp/msk_test
```

The above assumes that your host has direct or network access to the Pluto. If the Pluto is connected to another host you do have access to, you can try using the `-J jumphost` command line option for scp and ssh.

## What is what

Mainline development is in msk_rx_init.c

## Building the Application on Ubuntu (cross-platform)

This worked for me on a fresh Ubuntu 24.10 VM created by OrbStack.

```
cd $HOME
sudo apt install wget xz-utils git

# The cross toolchain should match the firmware build:
wget http://releases.linaro.org/components/toolchain/binaries/7.5-2019.12/arm-linux-gnueabihf/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf.tar.xz
tar xvf gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf.tar.xz
export PATH=$HOME/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin/:$PATH
# make the path change permanent by adding the line above to an appropriate login script file

wget https://github.com/analogdevicesinc/plutosdr-fw/releases/download/v0.38/sysroot-v0.38.tar.gz
tar xvfz sysroot-v0.38.tar.gz 
mv staging pluto-0.38.sysroot

# git clone <url for this repo>
cd pluto-msk-application
# copy msk_top_regs.h from firmware build to this directory

arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D RF_LOOPBACK -o msk_test-xmit msk_test.c -lpthread -liio -lm -Wall -Wextra && arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D RX_ACTIVE -o msk_test-recv msk_test.c -lpthread -liio -lm -Wall -Wextra
# The above line builds two versions of the application.
# The `xmit` version has `RF_LOOPBACK` enabled; it transmits PRBS forever.
# The `recv` version has `RX_ACTIVE` enabled; it does not transmit.
# Both versions try to receive and validate the data using PRBS Mon.
```

Notes:

1. Newer versions of the cross-development toolchain are unlikely to work without
some extra fiddling. You want to use this one, that matches the toolchain that built
the standard plutoSDR firmware, unless you're working with custom firmware built
with a different toolchain.

2. You don't need to install any IIO-related packages on the build host, but it's
harmless if you do. The `iio.h` include file and the `libiio` link library both
come out of the sysroot in this build.

3. You need the file `msk_top_regs.h` from the custom HDL build that's loaded into
the Pluto's FPGA. It defines the register access map.

4. The application code can build several different versions, depending on the
definition of preprocessor symbols such as `RF_LOOPBACK` and `RX_ACTIVE`. On the
command line shown above, I'm assuming none of those symbols is defined in the
source code. We define the ones we want on the command line with the `-D` flag.

5. You don't have to build two versions on the same command line as shown here.
That's just a convenience for the case we are dealing with as I write this:
debugging over-the-air operation using two Plutos, one transmitting PRBS into a
signal splitter connected to both Pluto receive ports. For this case, we always
need those two versions.

6. If you're using the specified toolchain, you should not need `-std=gnu99` on
the compiler command line. It might be useful with some other toolchains.