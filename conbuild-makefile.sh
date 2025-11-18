#!/bin/sh

MAKEFILE=${1:-"Makefile"}
echo Building with $MAKEFILE
docker run --platform=linux/amd64 --volume .:/repo build-from-makefile /usr/bin/make -f $MAKEFILE ${@:2}