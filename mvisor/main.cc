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
#include "sweet/server.h"

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

void PrintHelp() {
  printf("mvisor -config [config_path]\n");
}

static struct option long_options[] = {
  {"help", no_argument, 0, 'h'},
  {"uuid", required_argument, 0, 0},
  {"name", required_argument, 0, 0},
  {"config", required_argument, 0, 'c'},
  {"sweet", required_argument, 0, 0},
  {"pidfile", required_argument, 0, 0}
};

static Machine*     machine = nullptr;
static SweetServer* sweet_server = nullptr;
static Viewer*      viewer = nullptr;

int main(int argc, char* argv[])
{
  IntializeArguments(argc, argv);

  std::string config_path = "../config/default.yaml";
  std::string vm_uuid, vm_name;
  std::string sweet_path;
  std::string pid_path;

  int option_index = 0;
  while (getopt_long_only(argc, argv, "hc:", long_options, &option_index) != -1) {
    switch (option_index)
    {
    case 0:
      PrintHelp();
      return 0;
    case 1:
      vm_uuid = optarg;
      break;
    case 2:
      vm_name = optarg;
      break;
    case 3:
      config_path = optarg;
      break;
    case 4:
      sweet_path = optarg;
      break;
    case 5:
      pid_path = optarg;
      break;
    }
  }

  /* write pid to file if path specified */
  if (!pid_path.empty()) {
    FILE* fp = fopen(pid_path.c_str(), "wb");
    fprintf(fp, "%d\n", getpid());
    fclose(fp);
  }

  int ret;
  machine = new Machine(config_path);
  machine->set_vm_uuid(vm_uuid);
  machine->set_vm_name(vm_name.empty() ? vm_uuid : vm_name);

  /* There are two modes to control the virtual machine,
   * the GUI mode is to start a SDL viewwer,
   * and the non-GUI mode is to start a sweet server
   */
  if (!sweet_path.empty()) {
    sweet_server = new SweetServer(machine, sweet_path);
    signal(SIGINT, [](int signo) {
      machine->Quit();
      sweet_server->Close();
    });
    machine->Run();
    ret = sweet_server->MainLoop();
    delete sweet_server;
  } else {
    /* SDL handles default signals */
    viewer = new Viewer(machine);
    machine->Run();
    ret = viewer->MainLoop();
    delete viewer;
  }

  delete machine;

  if (!pid_path.empty()) {
    unlink(pid_path.c_str());
  }
  return ret;
}
