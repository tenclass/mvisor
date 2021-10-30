INCLUDE_DIRS := ./mvisor
LIBS := stdc++
LIBS += pthread
MKDIR_P = mkdir -p

CCFLAGS = $(addprefix -I, $(INCLUDE_DIRS)) -Wall -Werror -fno-exceptions -O2 -g
BUILD_DIR = ./build
$(shell mkdir -p $(BUILD_DIR)/devices)

EXECUTABLE = build/mvisor
MV_SOURCE := $(wildcard mvisor/*.cc)
MV_SOURCE += $(wildcard mvisor/devices/*.cc)
MV_OBJECTS := $(MV_SOURCE:mvisor/%.cc=$(BUILD_DIR)/%.o)

.PHONY: run all clean
run: all
	./build/mvisor

all: $(EXECUTABLE)

$(EXECUTABLE): $(MV_OBJECTS) $(BUILD_DIR)/main.o
	$(CC) -o $@ $^ $(addprefix -l, $(LIBS))

$(BUILD_DIR)/main.o: main.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

clean:
	$(RM) -rf build/*

$(MV_OBJECTS): $(BUILD_DIR)/%.o: mvisor/%.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<
