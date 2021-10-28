CCFLAGS = -I. -Wall -Wextra -Werror -fno-exceptions -O2

OBJECTS := main.o
OBJECTS += $(patsubst %.cc,%.o, $(wildcard mvisor/*.cc))

.PHONY: run
run: build/mvisor
	./build/mvisor

build/mvisor: $(OBJECTS) payload.o
	$(CC) $^ -lstdc++ -lpthread -o $@

payload.o: payload.ld guest16.o
	$(LD) -T $< -o $@

.PHONY: clean
clean:
	$(RM) build/mvisor payload.o guest16.o $(OBJECTS)

.cc.o:
	$(CC) $(CCFLAGS) -c -o $@ $<
