#include <cstdio>
#include <string>

#include "machine.h"
#include "viewer.h"

using namespace std;

int main()
{
  const int vcpus = 1;
  const uint64_t ram_size = 8LL * (1 << 30);
  Machine* machine = new Machine(vcpus, ram_size);
  Viewer* viewer = new Viewer(machine);
  machine->Run();
  int ret = viewer->MainLoop();
  delete viewer;
  // delete machine;
  return ret;
}
