#ifndef _MVISOR_SWEET_SWEET_H
#define _MVISOR_SWEET_SWEET_H

enum SweetCommand {
  kSweetCmdQueryStatus = 1,
  kSweetCmdSendKeyboardInput,
  kSweetCmdSendPointerInput,
  kSweetCmdQueryScreenshot,
  kSweetCmdConfigMonitors,
  kSweetCmdConnectDisplay,
  kSweetCmdRefreshDisplay,
  kSweetCmdConnectPlayback
};

enum SweetResponseAndEvent {
  kSweetResQueryStatus = 1,
  kSweetResQueryScreenshot,

  kSweetEventSetCursor,
  kSweetEventDisplayStreamStart,
  kSweetEventDisplayStreamStop,
  kSweetEventDisplayStreamData,
  kSweetEventPlaybackStart,
  kSweetEventPlaybackStop,
  kSweetEventPlaybackData
};

#endif // _MVISOR_SWEET_SWEET_H
