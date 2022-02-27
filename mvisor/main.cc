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
#include <uuid/uuid.h>
#include <getopt.h>

#include "machine.h"
#include "viewer.h"

using namespace std;

/* For vfio mdev, -uuid xxx is necessary */
void IntializeArguments(int argc, char* argv[]) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-uuid") == 0) {
      /* Found uuid */
      return;
    }
  }

  char uuid_string[40];
  uuid_t uuid;
  uuid_generate(uuid);
  uuid_unparse(uuid, uuid_string);

  char* new_argv[argc + 3];
  for (int i = 0; i < argc; i++) {
    new_argv[i] = argv[i];
  }
  new_argv[argc] = (char*)"-uuid";
  new_argv[argc + 1] = uuid_string;
  new_argv[argc + 2] = nullptr;
  execv("/proc/self/exe", new_argv);
  perror("Failed to restart process with uuid");
  exit(1);
}

void print_help() {
  printf("mvisor -f [config_path]\n");
}

static struct option long_options[] = {
  { "uuid", required_argument, 0, 0 }
};

int main(int argc, char* argv[])
{
  IntializeArguments(argc, argv);

  std::string config_path = "../config/default.yaml";
  int option, option_index = 0;
  while ((option = getopt_long_only(argc, argv, "f:h", long_options, &option_index)) != -1) {
    switch (option)
    {
    case 'f':
      config_path = optarg;
      break;
    case 'h':
      print_help();
      return 0;
    case '?':
      break;
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
