/* 
 * MVisor - Sweet Server
 * Copyright (C) 2022 Terrence <terrence@tenclass.com>
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


#ifndef _MVISOR_SWEET_SERVER_H
#define _MVISOR_SWEET_SERVER_H

#include <list>
#include <string>

#include "machine.h"

class SweetConnection;
class SweetServer {
 public:
  SweetServer(Machine* machine, std::string unix_path);
  ~SweetServer();

  int MainLoop();
  void Close();

  inline Machine* machine() { return machine_; }
 private:
  Machine*                    machine_;
  std::list<SweetConnection*> connections_;
  std::string                 unix_path_;
  int                         server_fd_ = -1;
};

#endif // _MVISOR_SWEET_SERVER_H
