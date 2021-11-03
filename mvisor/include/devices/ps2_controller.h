#ifndef _MVISOR_DEVICES_I8042_H
#define _MVISOR_DEVICES_I8042_H

#include "device.h"

#define QUEUE_SIZE  128
/*
 * This represents the current state of the PS/2 keyboard system,
 * including the AUX device (the mouse)
 */
struct kbd_state {
  uint8_t   kq[QUEUE_SIZE]; /* Keyboard queue */
  int       kread, kwrite; /* Indexes into the queue */
  int       kcount;  /* number of elements in queue */

  uint8_t   mq[QUEUE_SIZE];
  int       mread, mwrite;
  int       mcount;

  uint8_t   mstatus; /* Mouse status byte */
  uint8_t   mres;  /* Current mouse resolution */
  uint8_t   msample; /* Current mouse samples/second */

  uint8_t   mode;  /* i8042 mode register */
  uint8_t   status;  /* i8042 status register */
  /*
   * Some commands (on port 0x64) have arguments;
   * we store the command here while we wait for the argument
   */
  uint8_t   write_cmd;
};

class Ps2ControllerDevice : public Device {
 public:
  Ps2ControllerDevice(DeviceManager* manager);
  void Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);
  void Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size);

 private:
  void kbd_update_irq(void);
  void mouse_queue(uint8_t c);
  void kbd_queue(uint8_t c);
  void kbd_write_command(uint8_t val);
  uint8_t kbd_read_data(void);
  uint8_t kbd_read_status(void);
  void kbd_write_data(uint8_t val);
  void kbd_reset(void);
  void aux_command(uint8_t val);
  void ps2_command(uint8_t val);

  struct kbd_state state_;
};

#endif // _MVISOR_DEVICES_I8042_H
