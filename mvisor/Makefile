PROTOC = protoc
INCLUDE_DIRS := ./include /usr/include
LIBS := stdc++ stdc++fs
LIBS += pthread SDL2 yaml-cpp uuid protobuf zstd
MKDIR_P = mkdir -p

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

load: all
	$(EXECUTABLE) -f /tmp/save/configuration.yaml

debug: all
	gdb -ex "handle SIG34 nostop pass" -ex "run" $(EXECUTABLE)

image:
	qemu-img create -f qcow2 -F qcow2 -b /data/win10.qcow2 /data/hd.qcow2

vfio: all
	-echo 1 > /sys/bus/mdev/devices/c2e088ba-954f-11ec-8584-525400666f2b/remove
	echo "c2e088ba-954f-11ec-8584-525400666f2b" > /sys/class/mdev_bus/0000:3b:00.0/mdev_supported_types/nvidia-231/create
	$(EXECUTABLE)

all: $(EXECUTABLE)

proto: $(MV_PROTOBUF_CC)

$(EXECUTABLE): $(MV_OBJECTS) $(BUILD_DIR)/ui/keymap.o
	$(CC) -o $@ $^ $(addprefix -l, $(LIBS))

clean:
	$(RM) -rf $(BUILD_DIR)/*

$(MV_OBJECTS): $(BUILD_DIR)/%.o: %.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

$(BUILD_DIR)/ui/keymap.o: $(BUILD_DIR)/ui/%.o: ui/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(MV_PROTOBUF_CC): $(BUILD_DIR)/%.pb.cc: %.proto
	$(PROTOC) -I$(dir $<) --cpp_out=$(dir $@) $<
	cp $(subst .cc,.h,$@) ./include/states/

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<
