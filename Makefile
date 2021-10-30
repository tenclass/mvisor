INCLUDE_DIRS := .
LIBS := stdc++
LIBS += pthread

CCFLAGS = $(addprefix -I, $(INCLUDE_DIRS)) -Wall -Werror -fno-exceptions -O2 -g
BUILD_DIR = ./build

EXECUTABLE = build/mvisor
MV_SOURCE := $(wildcard mvisor/*.cc)
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
	$(RM) build/*

$(MV_OBJECTS): $(BUILD_DIR)/%.o: mvisor/%.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<
