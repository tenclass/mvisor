INCLUDE_DIRS := ./include /usr/include
LIBS := stdc++
LIBS += pthread aio SDL2
MKDIR_P = mkdir -p

# FIXME: Add -g only if debug mode is on
CCFLAGS = $(addprefix -I, $(INCLUDE_DIRS)) -Wall -Werror -fno-exceptions -g
BUILD_DIR = ../build

EXECUTABLE = $(BUILD_DIR)/mvisor
MV_SOURCE := $(wildcard *.cc)
MV_SOURCE += $(wildcard */*.cc)
MV_SOURCE += $(wildcard devices/*/*.cc)
MV_OBJECTS := $(MV_SOURCE:%.cc=$(BUILD_DIR)/%.o)

$(shell mkdir -p $(dir $(MV_OBJECTS)))

.PHONY: run all clean
run: all
	$(EXECUTABLE)

debug: all
	gdb $(EXECUTABLE)

all: $(EXECUTABLE)

$(EXECUTABLE): $(MV_OBJECTS) $(BUILD_DIR)/ui/keymap.o
	$(CC) -o $@ $^ $(addprefix -l, $(LIBS))

clean:
	$(RM) -rf $(BUILD_DIR)/*

$(MV_OBJECTS): $(BUILD_DIR)/%.o: %.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

$(BUILD_DIR)/ui/keymap.o: $(BUILD_DIR)/ui/%.o: ui/%.c
	$(CC) $(CCFLAGS) -c -o $@ $<

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<

