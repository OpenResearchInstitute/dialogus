#!/bin/sh

docker run --platform=linux/amd64 --volume .:/repo build-application /bin/sh build-tx-opulent-voice.sh
