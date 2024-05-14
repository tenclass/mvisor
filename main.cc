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
#include <getopt.h>

#include <filesystem>

#include "version.h"
#include "machine.h"
#include "utilities.h"
#include "gui/vnc/server.h"

static Machine*     machine = nullptr;
static VncServer*   vnc_server = nullptr;
static std::thread  vnc_server_thread;

#ifdef HAS_SWEET_SERVER
#include "sweet-server/server.h"
static SweetServer* sweet_server = nullptr;
static std::thread  sweet_server_thread;
#endif

#ifdef HAS_SDL
#include "gui/sdl/viewer.h"
static Viewer*      viewer = nullptr;
static std::thread  viewer_thread;
#endif


/* For vfio mdev, -uuid xxx is necessary */
static void IntializeArguments(int argc, char* argv[]) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-uuid") == 0) {
      /* Found uuid */
      return;
    }
  }

  char uuid_string[40] = {0};
  // Generate a random uuid string like "550e8400-e29b-41d4-a716-446655440000"
  for (int i = 0; i < 36; i++) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      uuid_string[i] = '-';
    } else {
      uuid_string[i] = "0123456789abcdef"[rand() % 16];
    }
  }

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
  printf("  -c, --config          Specified mvisor config file path.\n");
  printf("  -h, --help            Display this information.\n");
  printf("  -l, --load            Load mvisor snapshot information.\n");
  printf("  -m, --migration       Start mvisor witn port from migration.\n");
  printf("  -n, --name            Specified mvisor name information.\n");
  printf("  -p, --pidfile         Specified mvisor pid file path.\n");
  printf("  -s, --sweet           Specified mvisor socket file path.\n");
  printf("  -u, --uuid            Specified mvisor uuid information.\n");
  printf("  -v, --version         Display mvisor version information.\n");
  printf("  -vnc [port]           Start a VNC server at specified port.\n");
}

static void PrintVersion() {
  printf("MVisor: %s\n", VERSION);
  printf("Copyright (C) 2022 Terrence <terrence@tenclass.com>.\n");
  printf("License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>\n");
  printf("This is free software: you are free to change and redistribute it.\n");
  printf("There is NO WARRANTY, to the extent permitted by law\n");
}

static struct option long_options[] = {
  {"config", required_argument, 0, 'c'},
  {"help", no_argument, 0, 'h'},
  {"load", required_argument, 0, 'l'},
  {"migration", required_argument, 0, 'm'},
  {"name", required_argument, 0, 'n'},
  {"pidfile", required_argument, 0, 'p'},
  {"sweet", required_argument, 0, 's'},
  {"uuid", required_argument, 0, 'u'},
  {"version", no_argument, 0, 'V'},
  {"vnc", required_argument, 0, 'v'},
  {NULL, 0, 0, 0}
};

int main(int argc, char* argv[]) {
  IntializeArguments(argc, argv);
  SetThreadName("mvisor-main");

  std::string config_path = "../config/default.yaml";
  std::string vm_uuid, vm_name;
  std::string sweet_path;
  std::string pid_path;
  std::string load_path;
  std::string migration_port;
  uint16_t vnc_port = 0;

  int c, option_index = 0;
  while ((c = getopt_long_only(argc, argv, "hVc:u:n:s:p:l:", long_options, &option_index)) != -1) {
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
    case 'm':
      migration_port = optarg;
      break;
    case 'v':
      vnc_port = atoi(optarg);
      break;
    case 'V':
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

  machine = new Machine(config_path);
  machine->set_vm_uuid(vm_uuid);
  machine->set_vm_name(vm_name.empty() ? vm_uuid : vm_name);

  /* Register CTRL+C signal handler */
  auto quit_callback = [](int signum) {
    MV_UNUSED(signum);
    machine->Quit();
  };
  signal(SIGINT, quit_callback);
  signal(SIGTERM, quit_callback);

  /* SweetServer is used in Tenclass products */
  if (!sweet_path.empty()) {
#ifdef HAS_SWEET_SERVER
    sweet_server = new SweetServer(machine, sweet_path);
    sweet_server_thread = std::thread([]() {
      sweet_server->MainLoop();
    });

    if (!migration_port.empty()) {
      std::thread([&migration_port]() {
        machine->Load(atoi(migration_port.c_str()));
      }).detach();
    } else if (!load_path.empty()) {
      std::thread([&load_path]() {
        machine->Load(load_path);
      }).detach();
    }
#else
    MV_ERROR("Sweet server is not supported in this build");
#endif
  } else {
    /* Automatically start the SDL viewer if the GUI is enabled */
    if (!migration_port.empty()) {
      machine->Load(atoi(migration_port.c_str()));
    } else if (!load_path.empty()) {
      machine->Load(load_path);
    }

    const char* displayVar = std::getenv("DISPLAY");
    if (displayVar == nullptr) {
      if (vnc_port == 0) {
        vnc_port = 5901;
      }
    } else {
#ifdef HAS_SDL
      /* SDL handles default signals */
      viewer = new Viewer(machine);
      viewer_thread = std::thread([]() {
        viewer->MainLoop();
      });
#endif
    }

    machine->Resume();
  }

  if (vnc_port) {
    vnc_server = new VncServer(machine, vnc_port);
    vnc_server_thread = std::thread([]() {
      vnc_server->MainLoop();
    });
  }

  machine->WaitToQuit();
  if (!pid_path.empty()) {
    unlink(pid_path.c_str());
  }

  if (vnc_server) {
    vnc_server->Close();
    vnc_server_thread.join();
    delete vnc_server;
  }

#ifdef HAS_SWEET_SERVER
  if (sweet_server) {
    sweet_server->Close();
    sweet_server_thread.join();
    delete sweet_server;
  }
#endif

#ifdef HAS_SDL
  if (viewer) {
    viewer_thread.join();
    delete viewer;
  }
#endif
  delete machine;
  return 0;
}
