#include <cstdio>
#include <string>

#include "mvisor/machine.h"

using namespace std;

int main()
{
  mvisor::Machine machine;
  return machine.Run();
}
