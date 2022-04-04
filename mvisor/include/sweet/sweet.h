/* 
 * MVisor - Sweet Protocol Constants
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


#ifndef _MVISOR_SWEET_SWEET_H
#define _MVISOR_SWEET_SWEET_H

enum SweetCommand {
  kSweetCmdQueryStatus = 0x10,
  /* Machine operations */
  kSweetCmdResetMachine = 0x20,
  kSweetCmdStartupMachine,
  kSweetCmdShutdownMachine,
  kSweetCmdPauseMachine,
  kSweetCmdResumeMachine,
  kSweetCmdSaveMachine,
  /* Disk operations */
  kSweetCmdFreezeDisk = 0x30,
  kSweetCmdThawDisk,
  kSweetCmdSaveDisk,
  /* Input */
  kSweetCmdSendKeyboardInput = 0x40,
  kSweetCmdSendPointerInput,
  /* Display */
  kSweetCmdConfigMonitors = 0x50,
  kSweetCmdQueryScreenshot,
  kSweetCmdStartDisplayStream,
  kSweetCmdStopDisplayStream,
  kSweetCmdRefreshDisplayStream,
  /* Playback */
  kSweetCmdStartPlaybackStream = 0x60,
  kSweetCmdStopPlaybackStream,
  /* Record */
  kSweetCmdStartRecordStream = 0x70,
  kSweetCmdStopRecordStream,
  kSweetCmdSendRecordStreamData,
  /* Clipboard */
  kSweetCmdCopyToGuest = 0x80,
  kSweetCmdPasteFromGuest,
  /* USB */
  kSweetCmdConnectUsb = 0x90,
  kSweetCmdDisconnectUsb,
  kSweetCmdSendUsbData,
  /* Network */
  kSweetCmdStartTcpService = 0xA0,
  kSweetCmdConnectTcp,
  kSweetCmdCloseTcp,
  kSweetCmdSendTcpData,
};

enum SweetResponseAndEvent {
  kSweetResQueryStatus = 0x110,

  kSweetResQueryScreenshot = 0x150,
  kSweetEventSetCursor,
  kSweetEventDisplayStreamStart,
  kSweetEventDisplayStreamStop,
  kSweetEventDisplayStreamData,

  kSweetEventPlaybackStart = 0x160,
  kSweetEventPlaybackStop,
  kSweetEventPlaybackData
};


#endif // _MVISOR_SWEET_SWEET_H
