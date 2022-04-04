/* 
 * MVisor - Sweet Connections
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


#ifndef _MVISOR_SWEET_CONNECTION_H
#define _MVISOR_SWEET_CONNECTION_H

#include "machine.h"
#include "sweet/server.h"

class SweetConnection {
 public:
  SweetConnection(SweetServer* server, int fd);
  ~SweetConnection();

 private:
  Machine*      machine_;
  SweetServer*  server_;
  int           fd_;
};

#endif // _MVISOR_SWEET_CONNECTION_H
