#!/bin/sh
docker run --platform=linux/amd64 --volume .:/repo build-application /bin/sh build-txonly-streamrx-134test.sh