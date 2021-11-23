/* 
 * MVisor
 * Copyright (C) 2021 Terrence <terrence@tenclass.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <cstdio>
#include <string>

#include "machine.h"
#include "viewer.h"

using namespace std;

int main()
{
  const int vcpus = 4;
  const uint64_t ram_size = 4LL * (1 << 30);
  Machine* machine = new Machine(vcpus, ram_size);
  Viewer* viewer = new Viewer(machine);
  machine->Run();

  int ret = viewer->MainLoop();
  delete viewer;
  delete machine;
  return ret;
}
