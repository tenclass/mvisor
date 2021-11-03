#include <cstdio>
#include <string>

#include "machine.h"

using namespace std;

int main()
{
  const int vcpus = 1;
  const uint64_t ram_size = 8LL * (1 << 30);
  Machine machine(vcpus, ram_size);
  return machine.Run();
}
