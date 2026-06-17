# Dialogus (formerly pluto-msk-application)
C-code applications for the PLUTO MSK implementation 

## how-to:

### Build the application

See sections below for how to build the application.

### Transfer the application to the target

The ADALM Pluto target relies on a single USB port for all digital
connectivity, whereas the LibreSDR target has a fast hardware Ethernet
port. So the procedures differ somewhat.

When things settle down, we will probably build Dialogus as a part
of the `pluto_msk` build, so it will always be available on the target.
In the meantime, it's more convenient to build it separately and copy
the executable to the target.

Suppose we have built Dialogus in your current working directory.
By convention, we append the git hash to the filename to help
identify its version, so let's say the filename is `dialogus_1234567`.

#### If the target is an ADALM Pluto

To move the built application to the ADALM Pluto, working around the
many limitations of the Pluto's networking, is a four-step process:

1. Move the executable to the host which is directly connected
to the Pluto by USB cable. Assuming that host is named `raspi`
and your account on that host is named `ori`:
```
$ scp dialogus_1234567 ori@raspi.local:dialogus_1234567
```

2. Log into 'raspi' (from your computer):
```
$ ssh ori@raspi.local
```

3. Log into the Pluto (from raspi). Assuming the Pluto is using
the default IP address:
```
ori@raspi $ ssh root@192.168.2.1
root@192.168.2.1's password: 
______ _       _              ___________ _____ 
| ___ \ |     | |            |  _  | ___ \_   _|
| |_/ / |_   _| |_ ___ ______| | | | |_/ / | |  
|  __/| | | | | __/ _ \______| | | |    /  | |  
| |   | | |_| | || (_) |     \ \_/ / |\ \ _| |_ 
\_|   |_|\__,_|\__\___/       \___/\_| \_|\___/ 
3d89 F5OEO (2024)

[pluto:~]#
```

4. From the Pluto, use SCP to transfer the built application
from raspi to the Pluto:
```
[pluto:~]# scp ori@192.168.2.10:dialogus_1234567 .
ori@192.168.2.10's password:
```

You may be prompted for a password at each step. There are
two different passwords involved. The password for root
on the Pluto is `analog` unless you changed it at image
build time.

If you're familiar with SSH configuration, you can make
some of these steps easier and/or more scriptable. However,
the Pluto doesn't (by default) have any persistent storage,
so any trick that involves configuring the Pluto is likely
to be difficult.

#### If the target is a LibreSDR

To move the built application to the LibreSDR is a bit easier,
provided the LibreSDR has been configured to use a static IP
address on its Ethernet port, and that static IP address is
accessible from your development host computer. Let's say that
the static IP address is 10.73.6.7. You'll also need to know
the IP address of your own computer, lets's say it's 10.73.6.10,
and your username on that computer, let's say it's `ori`.

1. Log into the LibreSDR from your computer:
```
$ ssh root@10.73.6.7
root@10.73.6.7's password: 
______ _       _              ___________ _____ 
| ___ \ |     | |            |  _  | ___ \_   _|
| |_/ / |_   _| |_ ___ ______| | | | |_/ / | |  
|  __/| | | | | __/ _ \______| | | |    /  | |  
| |   | | |_| | || (_) |     \ \_/ / |\ \ _| |_ 
\_|   |_|\__,_|\__\___/       \___/\_| \_|\___/ 
3d89 F5OEO (2024)

[libresdr5:~]#
```

2. From the LibreSDR, use SCP to transfer the built application
from your computer to the LibreSDR:
```
[libresdr5:~]# scp ori@10.73.6.10:/path/to/dialogus_1234567 .
ori@192.168.2.10's password:
```

You may be prompted for a password at each step. There are
two different passwords involved. The password for root
on the LibreSDR is `analog` unless you changed it at image
build time.

If you're familiar with SSH configuration, you can make
some of these steps easier and/or more scriptable.

### Execute the application on the target

#### If the target is an ADALM Pluto

Next, we need to run the application on the Pluto.

You could just run it from the Pluto's command line, but
this is __not recommended__. You'll find that printing out
frequent debug messages that way is enough to bog down the
Pluto's processor, and there's no way to capture a large
log file on the Pluto itself.

Instead, you should stream the application's output to
a terminal on a more powerful machine. If you have a
reasonably powerful computer connected directly to the
Pluto, this works:
```
ssh -t root@192.168.2.1 dialogus_1234567
```

We often prefer to work with a tiny computer like a
Raspberry Pi connected directly to the Pluto. Depending
on the model of Raspberry Pi, it might also be a little
slow for this purpose, especially if you try to capture
a lot of debug output to a slow SD card. In that case,
we have had good luck using the Raspberry Pi as a
_jump host_ for the SSH connection from a powerful
desktop or laptop to the Pluto. To do this entirely
from the command line:
```
ssh -J ori@raspi.local -t root@192.168.3.2 /tmp/msk_test
```

#### If the target is a LibreSDR

The LibreSDR has a high-performance Ethernet port,
so we can just SSH to the LibreSDR over Ethernet and
run Dialogus on the LibreSDR over that connection.

Using either the LibresDR's USB ports for this purpose
is __not recommended___. They are not fast enough or
reliable enough.

## What is what

Mainline development has split into two paths. A stream-oriented
test-and-debug path is maintained in `opulent_voice.c`, with
various #define switches configuring the test implementation.
This version has been static for a long while now, and has fallen
behind the development.

The main forward path is now rooted in `dialogus.c`, and implements
the fully frame-oriented flow we intend to use for communicating
with the MSK modulator in a realistic manner. Instead of being a
single file, there are numerous smaller modules contained in their
own `.c` and `.h` files.

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

arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D NO_INIT_ON_SUCCESS -D RF_LOOPBACK -o msk_test-xmit msk_rx_init.c -lpthread -liio -lm -Wall -Wextra && arm-linux-gnueabihf-gcc -mfloat-abi=hard --sysroot=$HOME/pluto-0.38.sysroot -g -D ENDLESS_PRBS -D NO_INIT_ON_SUCCESS -D RX_ACTIVE -o msk_test-recv msk_rx_init.c -lpthread -liio -lm -Wall -Wextra
# The above line builds two versions of the application.
# The `xmit` version has `RF_LOOPBACK` enabled; it transmits PRBS forever.
# The `recv` version has `RX_ACTIVE` enabled; it does not transmit.
# Both versions try to receive and validate the data using PRBS Mon.
# Both versions continue to run without re-inits if all is going well.q
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
the target's FPGA. It defines the register access map. The application you build
won't work on a target with an FPGA image having an incompatible register map.
(Don't add `msk_top_regs.h` to this repository. It belongs to the HDL build.
When you clone this repo, copy `msk_top_regs.h` from the HDL build repo
you will be working with.)

4. The streaming test code in `opulent_voice.c` can build several different versions, depending on the
definition of preprocessor symbols such as `RF_LOOPBACK` and `RX_ACTIVE` and
`OPV_FRAME_MODE`. None of these should be #defined in the source code.
We define the ones we want on the command line with the `-D` flag.

5. You don't have to build two versions on the same command line as shown here.
That's just a convenience for the case we are dealing with as I write this:
debugging over-the-air operation using two Plutos, one transmitting PRBS into a
signal splitter connected to both Pluto receive ports. For this case, we always
need those two versions.

6. If you're using the specified toolchain, you should not need `-std=gnu99` on
the compiler command line. It might be useful with some other toolchains.

## Building with a Container

If your local host machine is not running Linux, or if you don't want to clutter
up your machine with the cross-platform toolchain, you may find it convenient to
do the cross-platform building in a (Docker) container. For example, I have used
this method routinely to build the application on a Mac.

### Building with a Command Line in a Container (Old and Busted)

This is the OLD WAY of building Dialogus, last used with opulent_voice.c.
Please don't use this way for new development.

`Dockerfile.script` contains the steps to build the container image. Here's how:

```
docker build -f Dockerfile.script --platform linux/amd64 -t build-application .
```

Create a shell script to perform the build operation(s) you need. The commands
are just like the ones we used to build manually. The file
`build-both-loopback-rx.sh` is an example of such a script.

To run the script in the container, use a command like this:

```
docker run --platform=linux/amd64 --volume .:/repo build-application /bin/sh build-both-loopback-rx.sh
```

Better yet, put that command into a shell script of its own, as we have done
in `conbuild-both-loopback-rx.sh`, and run the shell script (with no arguments
needed) whenever you want to rebuild the application(s).

If all goes well, the cross-platform compiler running inside the container will
have written the Pluto executable(s) into the CURRENT DIRECTORY on your host.

### Building with a Makefile in a Container (New Hotness)

We now build Dialogus using a Makefile. This replaces the above method.
Please use this method for new development. It's much handier for builds
involving more than a few source files.

`Dockerfile` contains the steps to build the container image. Here's how:

```
docker build --platform linux/amd64 -t build-from-makefile .
```

You don't need to name `Dockerfile` explicitly because that's the
default name. Notice that this builds a different container image than
the one we used before. It now includes `make` and `git`.

Create a makefile to perform the build operation(s) you need.
You'll probably want to start with `Makefile.dialogus`, which contains a few
nice tricks, including extracting a version description from the current
git commit hash and tags. The version description goes into the binary
filename, and also into a #define DIALOGUS_VERSION that can be used to
print out Dialogus version information from inside the program. (This
works best if you commit, at least locally, before building.)

The makefile also keeps the object and target binary files out of your
way in separate directories, `obj` and `bin`.

To build, you can use the script `conbuild-makefile.sh`. Just provide it
with the name of the makefile you want to use. Like this:

```
./conbuild-makefile.sh Makefile.dialogus
```

That builds the default target, which is of course the binary executable.

If you want to build some other target(s), such as `clean`, you can append
those target names at the end, like this:

```
./conbuild-makefile.sh Makefile.dialogus clean
```

That will not build the default target (unless you name it), but it will
try to build each target you mention.

Remember, this Makefile is being interpreted inside the container, so
it can't do things like run the executable on the target, or copy files
to other parts of the host's filesystem (unless you add such a mapping),
etc. That's why there's no `run` target, for instance.

## Runtime Configuration of Dialogus

In the past, Dialogus had all of its configuration hard-coded. In order
to build a version that used, say, a different frequency, you'd need to
edit the source code and recompile. That has now started to change.

Certain parameters can now be set when the program runs. The settings can
come from several difference sources, but they always have the same
format. The format consists of a keyword, an equals sign, and the value
for that keyword, repeated as many times as needed. There can be no
spaces around the equals sign, or anywhere else in each keyword assignment.

As I write this, there are only three keywords implemented. Keyword RXFREQ sets
the center frequency of the receive channel, in integer Hz. Keyword TXFREQ sets
the center frequency of the transmit channel, in integer Hz. Keyword FREQ sets
both frequencies. More keywords can be added as the need arises.

Two sources of configuration are implemented: the firmware environment variables
(these are the ones set using `fw_setenv` and read with `fw_printenv`) and the
Dialogus command line. The environment is checked first, then, the command line.
The last encountered setting overwrites any previous settings it may overlap
with. In the future, we may also implement reading configuration from a file
on the target, or even over the network.

So, for example, we may have two LibreSDRs cross-connected transmit to receive,
with each connection on a different frequency, for full-duplex testing. Using
the command line, you might run this on one LibreSDR:
```
# dialogus_1234567 RXFREQ=905050000 TXFREQ=431350000
```
and this on the other LibreSDR:
```
# dialogus_1234567 RXFREQ=431350000 TXFREQ=905050000
```
Both LibreSDRs can be running the exact same build of Dialogus, or any
pair of mutually-compatible versions. The command line frequency settings
override any defaults or environment variable settings for frequencies.

You could also do the same thing using environment variables, like so:
```
[libresdr1:~]# fw_setenv dialogus libre RXFREQ=905050000 TXFREQ=431350000
[libresdr1:~]# dialogus_1234567
```
and
```
[libresdr2:~]# fw_setenv dialogus libre RXFREQ=431350000 TXFREQ=905050000
[libresdr2:~]# dialogus_1234567
```
In this case, you would only need to run the fw_setenv commands once, and
the values are remembered in non-volatile memory inside the LibreSDRs.

You could then leave those environment variables alone, as defaults, and
still use those two LibreSDR devices on other frequencies by overriding
the frequency settings on the command line.

## Selecting a Channel Number at Runtime

There's also a shell script `scripts/on_channel.sh`, that makes it easy
to choose one the channels demodulated by the polyphase channelizer in
the satellite or groundsat. The shell script knows the band plan for
the channelizer (assuming it's kept up to date when the band plan changes).

Run it like this:
```
[libresdr5:~]# ./on_channel.sh 27 ./dialogus_1234567
```
This executes the version of Dialogus you mentioned, setting FREQ (i.e., 
both transmit and receive channel center frequencies) to the frequency
corresponding to channel 27.

You can also add other keywords on the command line, once we have defined
some other keywords. Something like this:
```
[libresdr5:~]# ./on_channel.sh 27 ./dialogus_1234567 PARAM1=1 PARAM2=42
```
In either case, the script appends a FREQ keyword assignment at the end
of the command line, overriding any environment variables or other
keywords on the command line you typed.
