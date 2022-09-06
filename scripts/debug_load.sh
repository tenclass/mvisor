#!/bin/sh
meson compile -C ../build
gdb -ex 'handle SIG34 nostop pass' -ex 'run -c /tmp/save/configuration.yaml --load /tmp/save/' ../build/mvisor
