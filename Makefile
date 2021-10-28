CCFLAGS = -I. -Wall -Wextra -Werror -fno-exceptions -O2 -g
LIBS = stdc++ pthread
BUILD_DIR = ./build

EXECUTABLE = build/mvisor
MV_SOURCE := $(wildcard mvisor/*.cc)
MV_OBJECTS := $(MV_SOURCE:mvisor/%.cc=$(BUILD_DIR)/%.o)

.PHONY: run all clean
run: all
	./build/mvisor

all: $(EXECUTABLE)

$(EXECUTABLE): $(MV_OBJECTS) $(BUILD_DIR)/payload.o $(BUILD_DIR)/main.o
	$(CC) -o $@ $^ $(addprefix -l, $(LIBS))

$(BUILD_DIR)/payload.o: payload.ld $(BUILD_DIR)/guest16.o
	$(LD) -o $@ -T $<

$(BUILD_DIR)/guest16.o: guest16.s
	$(AS) -o $@ $<

$(BUILD_DIR)/main.o: main.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

clean:
	$(RM) build/*

$(MV_OBJECTS): $(BUILD_DIR)/%.o: mvisor/%.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<
