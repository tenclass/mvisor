#!/bin/sh
meson compile -C ../build
gdb -ex 'handle SIG34 nostop pass' -ex 'run' ../build/mvisor

