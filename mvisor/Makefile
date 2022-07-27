PROTOC = protoc
INCLUDE_DIRS := ./include /usr/include
LIBS := stdc++ stdc++fs
LIBS += pthread SDL2 yaml-cpp uuid protobuf zstd z asound x264 yuv opus jpeg
MKDIR_P = mkdir -p

VERSION = "1.4.2"
VERSION_FILE = "./include/version.h"

# FIXME: Add -g only if debug mode is on
CFLAGS = $(addprefix -I, $(INCLUDE_DIRS)) -Wall -Werror -g
CCFLAGS = $(CFLAGS) -std=c++17 -mavx2
BUILD_DIR = ../build

MV_PROTOBUF_SOURCE := $(wildcard */*.proto)
MV_PROTOBUF_SOURCE += $(wildcard devices/*/*.proto)
MV_PROTOBUF_CC := $(MV_PROTOBUF_SOURCE:%.proto=$(BUILD_DIR)/%.pb.cc)


EXECUTABLE = $(BUILD_DIR)/mvisor
MV_SOURCE := $(wildcard *.cc)
MV_SOURCE += $(wildcard */*.cc)
MV_SOURCE += $(wildcard devices/*/*.cc)
MV_SOURCE += $(wildcard networks/*/*.cc)
MV_SOURCE += $(MV_PROTOBUF_CC)
MV_OBJECTS := $(MV_SOURCE:%.cc=$(BUILD_DIR)/%.o)

$(shell mkdir -p $(dir $(MV_OBJECTS)))

.PHONY: run all clean
run: all
	time $(EXECUTABLE)

# Press F2 to save VM to /tmp/save and make load to restore the VM
# Remember to use snapshot=Yes in disk configuration
load: all
	$(EXECUTABLE) -config /tmp/save/configuration.yaml -load /tmp/save

sweet: all
	$(EXECUTABLE) -sweet /tmp/sweet.sock -pidfile /tmp/pid

debug_sweet: all
	gdb -ex "handle SIG34 nostop pass" -ex "run -sweet /var/run/sfcloud/5ffdf526b0321206714888c7_6254e6303d4cd2dd150d7a80/sweet.sock" $(EXECUTABLE)

debug_load: all
	gdb -ex "handle SIG34 nostop pass" -ex "run -config /tmp/save/configuration.yaml -load /tmp/save" $(EXECUTABLE)

debug: all
	gdb -ex "handle SIG34 nostop pass" -ex "run" $(EXECUTABLE)

install: all
	cp -f $(EXECUTABLE) /mnt/server/opt/mvisor/build/bin/

image:
	qemu-img create -f qcow2 -F qcow2 -b /data/win10.qcow2 /data/hd.qcow2

vfio: all
	-echo 1 > /sys/bus/mdev/devices/c2e088ba-954f-11ec-8584-525400666f2b/remove
	echo "c2e088ba-954f-11ec-8584-525400666f2b" > /sys/class/mdev_bus/0000:3b:00.0/mdev_supported_types/nvidia-231/create
	$(EXECUTABLE)

gvt:
	echo "c2e088ba-954f-11ec-8584-525400666f2b" > /sys/class/mdev_bus/0000:00:02.0/mdev_supported_types/i915-GVTg_V5_4/create

sweet_js:
	$(PROTOC) --js_out=import_style=commonjs,binary:. sweet/sweet.proto
	mv sweet/sweet_pb.js /mnt/server/sfcloud/sfspice/

all: version $(EXECUTABLE)

version:
	@if [ -e $(VERSION_FILE) ]; then $(RM) -rf $(VERSION_FILE) else touch $(VERSION_FILE); fi
	@echo "#define VERSION \"$(VERSION)\"" > $(VERSION_FILE)

proto: $(MV_PROTOBUF_CC)

$(EXECUTABLE): $(MV_OBJECTS) $(BUILD_DIR)/sdl/keymap.o
	$(CC) -o $@ $^ $(addprefix -l, $(LIBS))

clean:
	$(RM) -rf $(BUILD_DIR)/*
	$(RM) -rf $(VERSION_FILE)

$(MV_OBJECTS): $(BUILD_DIR)/%.o: %.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

$(BUILD_DIR)/sdl/keymap.o: $(BUILD_DIR)/sdl/%.o: sdl/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(MV_PROTOBUF_CC): $(BUILD_DIR)/%.pb.cc: %.proto
	$(PROTOC) -I$(dir $<) --cpp_out=$(dir $@) $<
	cp $(subst .cc,.h,$@) ./include/pb/

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<
