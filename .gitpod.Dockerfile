FROM ubuntu:14.04
RUN apt-get update && apt-get install -y build-essential git mingw32 mingw32-binutils mingw32-runtime gettext zip python python-newt
RUN cd /opt && curl --retry 10 --retry-max-time 120 -L 'https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2018q4/gcc-arm-none-eabi-8-2018-q4-major-linux.tar.bz2' | tar xfj -

# Install custom tools, runtimes, etc.
# For example "bastet", a command-line tetris clone:
# RUN brew install bastet
#
# More information: https://www.gitpod.io/docs/config-docker/
