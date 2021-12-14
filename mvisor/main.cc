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
#include <unistd.h>

#include "machine.h"
#include "viewer.h"

using namespace std;

void print_help() {
  printf("mvisor -f [config_path]\n");
}

int main(int argc, char* argv[])
{
  std::string config_path = "../config/default.yaml";
  int option;
  while ((option = getopt(argc, argv, "f:h")) != -1) {
    switch (option)
    {
    case 'f':
      config_path = optarg;
      break;
    case 'h':
      print_help();
      return 0;
    case '?':
      fprintf(stderr, "Unknown option: %c\n", (char)optopt);
      return -1;
    }
  }

  Machine* machine = new Machine(config_path);
  Viewer* viewer = new Viewer(machine);
  machine->Run();

  int ret = viewer->MainLoop();
  delete viewer;
  delete machine;
  return ret;
}
