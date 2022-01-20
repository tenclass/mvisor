INCLUDE_DIRS := ./include /usr/include
LIBS := stdc++
LIBS += pthread aio SDL2 yaml-cpp
MKDIR_P = mkdir -p

# FIXME: Add -g only if debug mode is on
CFLAGS = $(addprefix -I, $(INCLUDE_DIRS)) -Wall -Werror -g
CCFLAGS = $(CFLAGS) -std=c++17
BUILD_DIR = ../build

EXECUTABLE = $(BUILD_DIR)/mvisor
MV_SOURCE := $(wildcard *.cc)
MV_SOURCE += $(wildcard */*.cc)
MV_SOURCE += $(wildcard devices/*/*.cc)
MV_SOURCE += $(wildcard networks/*/*.cc)
MV_OBJECTS := $(MV_SOURCE:%.cc=$(BUILD_DIR)/%.o)

$(shell mkdir -p $(dir $(MV_OBJECTS)))

.PHONY: run all clean
run: all
	$(EXECUTABLE)

debug: all
	gdb $(EXECUTABLE)

image:
	qemu-img create -f qcow2 -F qcow2 -b /data/win10.qcow2 /data/hd.qcow2

all: $(EXECUTABLE)

$(EXECUTABLE): $(MV_OBJECTS) $(BUILD_DIR)/ui/keymap.o
	$(CC) -o $@ $^ $(addprefix -l, $(LIBS))

clean:
	$(RM) -rf $(BUILD_DIR)/*

$(MV_OBJECTS): $(BUILD_DIR)/%.o: %.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

$(BUILD_DIR)/ui/keymap.o: $(BUILD_DIR)/ui/%.o: ui/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<

