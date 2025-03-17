# pluto-msk-application
C-code applications for the PLUTO MSK implementation 

## how-to:

See section below for how to build the application.

To move the built application (here, `msk_test`) to the Pluto:
```
scp msk_test root@pluto.local:/tmp/msk_test
```

Then, to execute the application (here, `msk_test`) on the Pluto with output streamed to your terminal:
```
ssh -t root@pluto.local /tmp/msk_test
```

The above assumes that your host has direct or network access to the Pluto. If the Pluto is connected to another host you do have access to, you can try using the `-J jumphost` command line option for scp and ssh.

To execute the application (here, `msk_test`) on the Pluto with output streamed to your terminal and also captured to a file for later analysis:
```
ssh -t root@pluto.local /tmp/msk_test | tee logfile.txt
```


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
the Pluto's FPGA. It defines the register access map. The application you build
won't work on a Pluto with an FPGA image having an incompatible register map.

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

## Buiding with a Container

If your local host machine is not running Linux, or if you don't want to clutter
up your machine with the cross-platform toolchain, you may find it convenient to
do the cross-platform building in a (Docker) container. For example, I've used
this method to build the application on a Mac.

`Dockerfile` contains the steps to build the container image:

```
docker build --platform linux/amd64 -t build-application .
```

Create a shell script to perform the build operation(s) you need. The commands
are just like the ones we used in the previous section. The file
`build-both-loopback-rx.sh` is an example of such a script.

To run the script in the container, use a command like this:

```
docker run --platform=linux/amd64 --volume .:/repo build-application /bin/sh build-both-loopback-rx.sh
```

Better yet, put that command into a shell script of its own, as we have done
in `conbuild-both-loopback-rx.sh`, and run the shell script (with no arguments
needed) whenever you want to rebuild the application(s).

If all goes well, the cross-platform compiler running inside the container will
have written the Pluto executable(s) into the current directory on your host.
