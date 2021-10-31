INCLUDE_DIRS := ./include
LIBS := stdc++
LIBS += pthread
MKDIR_P = mkdir -p

CCFLAGS = $(addprefix -I, $(INCLUDE_DIRS)) -Wall -Werror -fno-exceptions -O2 -g
BUILD_DIR = ../build
$(shell mkdir -p $(BUILD_DIR)/devices)
$(shell mkdir -p $(BUILD_DIR)/core)
$(shell mkdir -p $(BUILD_DIR)/arch)
$(shell mkdir -p $(BUILD_DIR)/utils)

EXECUTABLE = $(BUILD_DIR)/mvisor
MV_SOURCE := main.cc
MV_SOURCE += $(wildcard core/*.cc)
MV_SOURCE += $(wildcard arch/*.cc)
MV_SOURCE += $(wildcard devices/*.cc)
MV_SOURCE += $(wildcard utils/*.cc)
MV_OBJECTS := $(MV_SOURCE:%.cc=$(BUILD_DIR)/%.o)

.PHONY: run all clean
run: all
	$(EXECUTABLE)

all: $(EXECUTABLE)

$(EXECUTABLE): $(MV_OBJECTS)
	$(CC) -o $@ $^ $(addprefix -l, $(LIBS))

clean:
	$(RM) -rf $(BUILD_DIR)/*

$(MV_OBJECTS): $(BUILD_DIR)/%.o: %.cc
	$(CC) $(CCFLAGS) -c -o $@ $<

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<
