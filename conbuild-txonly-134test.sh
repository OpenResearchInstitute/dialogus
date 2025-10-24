#!/bin/sh
docker run --platform=linux/amd64 --volume .:/repo build-application /bin/sh build-txonly-134test.sh