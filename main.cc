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

#include <filesystem>

#include "version.h"
#include "machine.h"


static Machine*     machine = nullptr;

#ifdef HAS_SWEET_SERVER
#include "sweet-server/server.h"
static SweetServer* sweet_server = nullptr;
#endif

#ifdef HAS_SDL
#include "gui/sdl/viewer.h"
static Viewer*      viewer = nullptr;
#endif


/* For vfio mdev, -uuid xxx is necessary */
static void IntializeArguments(int argc, char* argv[]) {
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

static void PrintHelp() {
  printf("Usage: mvisor [option]\n");
  printf("Options\n");
  printf("  -h, --help            Display this information.\n");
  printf("  -v, --version         Display mvisor version information.\n");
  printf("  -u, --uuid            Specified mvisor uuid information.\n");
  printf("  -n, --name            Specified mvisor name information.\n");
  printf("  -c, --config          Specified mvisor config file path.\n");
  printf("  -s, --sweet           Specified mvisor socket file path.\n");
  printf("  -p, --pidfile         Specified mvisor pid file path.\n");
  printf("  -l, --load            Load mvisor snapshot information.\n");
}

static void PrintVersion() {
  printf("MVisor: %s\n", VERSION);
  printf("Copyright (C) 2022 Terrence <terrence@tenclass.com>.\n");
  printf("License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>\n");
  printf("This is free software: you are free to change and redistribute it.\n");
  printf("There is NO WARRANTY, to the extent permitted by law\n");
}

static struct option long_options[] = {
  {"help", no_argument, 0, 'h'},
  {"uuid", required_argument, 0, 'u'},
  {"name", required_argument, 0, 'n'},
  {"config", required_argument, 0, 'c'},
  {"sweet", required_argument, 0, 's'},
  {"pidfile", required_argument, 0, 'p'},
  {"load", required_argument, 0, 'l'},
  {"version", no_argument, 0, 'v'},
  {NULL, 0, 0, 0}
};

int main(int argc, char* argv[]) {
  IntializeArguments(argc, argv);

  std::string config_path = "../config/default.yaml";
  std::string vm_uuid, vm_name;
  std::string sweet_path;
  std::string pid_path;
  std::string load_path;

  int c, option_index = 0;
  while ((c = getopt_long_only(argc, argv, "hvc:u:n:s:p:l:", long_options, &option_index)) != -1) {
    switch (c)
    {
    case 'h':
      PrintHelp();
      return 0;
    case 'u':
      vm_uuid = optarg;
      break;
    case 'n':
      vm_name = optarg;
      break;
    case 'c':
      config_path = optarg;
      break;
    case 's':
      sweet_path = optarg;
      break;
    case 'p':
      pid_path = optarg;
      break;
    case 'l':
      load_path = optarg;
      break;
    case 'v':
      PrintVersion();
      return 0;
    case '?':
      PrintHelp();
      return 0;
    }
  }

  /* write pid to file if path specified */
  if (!pid_path.empty()) {
    FILE* fp = fopen(pid_path.c_str(), "wb");
    fprintf(fp, "%d\n", getpid());
    fclose(fp);
  }

  int ret = 0;
  machine = new Machine(config_path);
  machine->set_vm_uuid(vm_uuid);
  machine->set_vm_name(vm_name.empty() ? vm_uuid : vm_name);

#ifdef HAS_SWEET_SERVER
  /* There are two modes to control the virtual machine,
   * the GUI mode is to start a SDL viewwer,
   * and the non-GUI mode is to start a sweet server
   */
  if (!sweet_path.empty()) {
    sweet_server = new SweetServer(machine, sweet_path);
    auto quit_callback = [](int signum) {
      MV_UNUSED(signum);

      machine->Quit();
      sweet_server->Close();
    };
    signal(SIGINT, quit_callback);
    signal(SIGTERM, quit_callback);

    if (!load_path.empty()) {
      std::thread([&load_path]() {
        machine->Load(load_path);
      }).detach();
    }

    ret = sweet_server->MainLoop();
    delete sweet_server;
  }
#endif

#ifdef HAS_SDL
  if (sweet_path.empty()) {
    /* SDL handles default signals */
    viewer = new Viewer(machine);

    if (!load_path.empty())
      machine->Load(load_path);
    machine->Resume();

    ret = viewer->MainLoop();
    delete viewer;
  }
#endif

  delete machine;

  if (!pid_path.empty()) {
    unlink(pid_path.c_str());
  }
  return ret;
}
