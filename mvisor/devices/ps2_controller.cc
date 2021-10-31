#include "devices/ps2_controller.h"
#include "logger.h"
#include "machine.h"
#include "device_manager.h"


/*
 * IRQs
 */
#define KBD_IRQ			1
#define AUX_IRQ			12

/*
 * Registers
 */
#define I8042_DATA_REG		0x60
#define I8042_PORT_B_REG	0x61
#define I8042_COMMAND_REG	0x64
#define I8042_A20_GATE    0x92

/*
 * Commands
 */
#define I8042_CMD_CTL_RCTR	0x20
#define I8042_CMD_CTL_WCTR	0x60
#define I8042_CMD_AUX_LOOP	0xD3
#define I8042_CMD_AUX_SEND	0xD4
#define I8042_CMD_AUX_TEST	0xA9
#define I8042_CMD_AUX_DISABLE	0xA7
#define I8042_CMD_AUX_ENABLE	0xA8
#define I8042_CMD_SYSTEM_RESET	0xFE

#define RESPONSE_ACK		0xFA

#define MODE_DISABLE_AUX	0x20

#define AUX_ENABLE_REPORTING	0x20
#define AUX_SCALING_FLAG	0x10
#define AUX_DEFAULT_RESOLUTION	0x2
#define AUX_DEFAULT_SAMPLE	100

/*
 * Status register bits
 */
#define I8042_STR_AUXDATA	0x20
#define I8042_STR_KEYLOCK	0x10
#define I8042_STR_CMDDAT	0x08
#define I8042_STR_MUXERR	0x04
#define I8042_STR_OBF		0x01

#define KBD_MODE_KBD_INT	0x01
#define KBD_MODE_SYS		0x02

/*
 * If there are packets to be read, set the appropriate IRQs high
 */
void Ps2ControllerDevice::kbd_update_irq(void)
{
	uint8_t klevel = 0;
	uint8_t mlevel = 0;

	/* First, clear the kbd and aux output buffer full bits */
	state_.status &= ~(I8042_STR_OBF | I8042_STR_AUXDATA);

	if (state_.kcount > 0) {
		state_.status |= I8042_STR_OBF;
		klevel = 1;
	}

	/* Keyboard has higher priority than mouse */
	if (klevel == 0 && state_.mcount != 0) {
		state_.status |= I8042_STR_OBF | I8042_STR_AUXDATA;
		mlevel = 1;
	}

	machine_->Interrupt(KBD_IRQ, klevel);
	machine_->Interrupt(AUX_IRQ, mlevel);
}

/*
 * Add a byte to the mouse queue, then set IRQs
 */
void Ps2ControllerDevice::mouse_queue(uint8_t c)
{
	if (state_.mcount >= QUEUE_SIZE)
		return;

	state_.mq[state_.mwrite++ % QUEUE_SIZE] = c;

	state_.mcount++;
	kbd_update_irq();
}

/*
 * Add a byte to the keyboard queue, then set IRQs
 */
void Ps2ControllerDevice::kbd_queue(uint8_t c)
{
	if (state_.kcount >= QUEUE_SIZE)
		return;

	state_.kq[state_.kwrite++ % QUEUE_SIZE] = c;

	state_.kcount++;
	kbd_update_irq();
}

void Ps2ControllerDevice::kbd_write_command(uint8_t val)
{
	switch (val) {
	case I8042_CMD_CTL_RCTR:
		kbd_queue(state_.mode);
		break;
	case I8042_CMD_CTL_WCTR:
	case I8042_CMD_AUX_SEND:
	case I8042_CMD_AUX_LOOP:
		state_.write_cmd = val;
		break;
	case I8042_CMD_AUX_TEST:
		/* 0 means we're a normal PS/2 mouse */
		mouse_queue(0);
		break;
	case I8042_CMD_AUX_DISABLE:
		state_.mode |= MODE_DISABLE_AUX;
		break;
	case I8042_CMD_AUX_ENABLE:
		state_.mode &= ~MODE_DISABLE_AUX;
		break;
	case I8042_CMD_SYSTEM_RESET:
    MV_PANIC("I8042_CMD_SYSTEM_RESET");
		break;
	default:
		break;
	}
}

/*
 * Called when the OS reads from port 0x60 (PS/2 data)
 */
uint8_t Ps2ControllerDevice::kbd_read_data(void)
{
	uint8_t ret;
	int i;

	if (state_.kcount != 0) {
		/* Keyboard data gets read first */
		ret = state_.kq[state_.kread++ % QUEUE_SIZE];
		state_.kcount--;
		machine_->Interrupt(KBD_IRQ, 0);
		kbd_update_irq();
	} else if (state_.mcount > 0) {
		/* Followed by the mouse */
		ret = state_.mq[state_.mread++ % QUEUE_SIZE];
		state_.mcount--;
		machine_->Interrupt(AUX_IRQ, 0);
		kbd_update_irq();
	} else {
		i = state_.kread - 1;
		if (i < 0)
			i = QUEUE_SIZE;
		ret = state_.kq[i];
	}
	return ret;
}

/*
 * Called when the OS read from port 0x64, the command port
 */
uint8_t Ps2ControllerDevice::kbd_read_status(void)
{
	return state_.status;
}

/*
 * Called when the OS writes to port 0x60 (data port)
 * Things written here are generally arguments to commands previously
 * written to port 0x64 and stored in state_.write_cmd
 */
void Ps2ControllerDevice::kbd_write_data(uint8_t val)
{
	switch (state_.write_cmd) {
	case I8042_CMD_CTL_WCTR:
		state_.mode = val;
		kbd_update_irq();
		break;
	case I8042_CMD_AUX_LOOP:
		mouse_queue(val);
		mouse_queue(RESPONSE_ACK);
		break;
	case I8042_CMD_AUX_SEND:
		/* The OS wants to send a command to the mouse */
		mouse_queue(RESPONSE_ACK);
		switch (val) {
		case 0xe6:
			/* set scaling = 1:1 */
			state_.mstatus &= ~AUX_SCALING_FLAG;
			break;
		case 0xe8:
			/* set resolution */
			state_.mres = val;
			break;
		case 0xe9:
			/* Report mouse status/config */
			mouse_queue(state_.mstatus);
			mouse_queue(state_.mres);
			mouse_queue(state_.msample);
			break;
		case 0xf2:
			/* send ID */
			mouse_queue(0); /* normal mouse */
			break;
		case 0xf3:
			/* set sample rate */
			state_.msample = val;
			break;
		case 0xf4:
			/* enable reporting */
			state_.mstatus |= AUX_ENABLE_REPORTING;
			break;
		case 0xf5:
			state_.mstatus &= ~AUX_ENABLE_REPORTING;
			break;
		case 0xf6:
			/* set defaults, just fall through to reset */
		case 0xff:
			/* reset */
			state_.mstatus = 0x0;
			state_.mres = AUX_DEFAULT_RESOLUTION;
			state_.msample = AUX_DEFAULT_SAMPLE;
			break;
		default:
			break;
		}
		break;
	case 0:
		/* Just send the ID */
		kbd_queue(RESPONSE_ACK);
		kbd_queue(0xab);
		kbd_queue(0x41);
		kbd_update_irq();
		break;
	default:
		/* Yeah whatever */
		break;
	}
	state_.write_cmd = 0;
}

void Ps2ControllerDevice::kbd_reset(void)
{
	state_ = (struct kbd_state) {
		.mres		= AUX_DEFAULT_RESOLUTION,
		.msample	= AUX_DEFAULT_SAMPLE,
		.mode		= KBD_MODE_KBD_INT | KBD_MODE_SYS, /* 0x3 */
		.status		= I8042_STR_MUXERR | I8042_STR_CMDDAT | I8042_STR_KEYLOCK, /* 0x1c */
	};
}

Ps2ControllerDevice::Ps2ControllerDevice(DeviceManager* manager)
  : Device(manager) {
  name_ = "ps2";
  machine_ = manager_->machine();

  AddIoResource(kIoResourceTypePio, I8042_A20_GATE, 1);
  AddIoResource(kIoResourceTypePio, I8042_DATA_REG, 2);
  AddIoResource(kIoResourceTypePio, I8042_COMMAND_REG, 2);
}

void Ps2ControllerDevice::Read(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {

	switch (ir.base + offset) {
	case I8042_COMMAND_REG:
		*data = kbd_read_status();
		break;
	case I8042_DATA_REG:
		*data = kbd_read_data();
		break;
	case I8042_PORT_B_REG:
		*data = 0x20;
		break;
  case I8042_A20_GATE:
		// Always returns enabled
    *data = 2;
	default:
		return;
	}
}

void Ps2ControllerDevice::Write(const IoResource& ir, uint64_t offset, uint8_t* data, uint32_t size) {
  uint8_t value = *data;

	switch (ir.base + offset) {
	case I8042_COMMAND_REG:
		kbd_write_command(value);
		break;
	case I8042_DATA_REG:
		kbd_write_data(value);
		break;
	default:
		return;
	}
}
