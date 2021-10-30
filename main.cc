#include <cstdio>
#include <string>

#include "mvisor/machine.h"

using namespace std;

int main()
{
  const uint64_t ram_size = 8LL * (1 << 30);
  Machine machine(ram_size);
  return machine.Run();
}
