#include "device.h"
#include "logger.h"

#define APPLESMC_DEFAULT_IOBASE        0x300
#define APPLESMC_MAX_DATA_LENGTH       32

enum {
    APPLESMC_DATA_PORT               = 0x00,
    APPLESMC_CMD_PORT                = 0x04,
    APPLESMC_ERR_PORT                = 0x1e,
    APPLESMC_NUM_PORTS               = 0x20,
};

enum {
    APPLESMC_READ_CMD                = 0x10,
    APPLESMC_WRITE_CMD               = 0x11,
    APPLESMC_GET_KEY_BY_INDEX_CMD    = 0x12,
    APPLESMC_GET_KEY_TYPE_CMD        = 0x13,
};

enum {
    APPLESMC_ST_CMD_DONE             = 0x00,
    APPLESMC_ST_DATA_READY           = 0x01,
    APPLESMC_ST_BUSY                 = 0x02,
    APPLESMC_ST_ACK                  = 0x04,
    APPLESMC_ST_NEW_CMD              = 0x08,
};

enum {
    APPLESMC_ST_1E_CMD_INTRUPTED     = 0x80,
    APPLESMC_ST_1E_STILL_BAD_CMD     = 0x81,
    APPLESMC_ST_1E_BAD_CMD           = 0x82,
    APPLESMC_ST_1E_NOEXIST           = 0x84,
    APPLESMC_ST_1E_WRITEONLY         = 0x85,
    APPLESMC_ST_1E_READONLY          = 0x86,
    APPLESMC_ST_1E_BAD_INDEX         = 0xb8,
};


#define DATA_PORT (APPLESMC_DEFAULT_IOBASE + APPLESMC_DATA_PORT)
#define CMD_PORT (APPLESMC_DEFAULT_IOBASE + APPLESMC_CMD_PORT)
#define ERROR_PORT (APPLESMC_DEFAULT_IOBASE + APPLESMC_ERR_PORT)


class AppleSmc : public Device {
 private:
  uint8_t status_;
  uint8_t status_1e_;
  uint8_t command_;
  int     pos_;
  char    key_[4];
  uint8_t data_[256];
  uint8_t data_len_;
  uint8_t data_pos_;
  std::map<std::string, std::string> smc_data_;

  void WriteCommand(uint8_t data) {
    if (data == APPLESMC_READ_CMD) {
      if (status_ == APPLESMC_ST_CMD_DONE || status_ == APPLESMC_ST_NEW_CMD) {
          command_ = data;
          status_ = APPLESMC_ST_NEW_CMD | APPLESMC_ST_ACK;
      } else {
        status_ = APPLESMC_ST_NEW_CMD;
        status_1e_ = APPLESMC_ST_1E_CMD_INTRUPTED;
      }
    } else {
      if (debug_) {
        MV_WARN("not implemented command 0x%x", data);
      }
      status_ = APPLESMC_ST_NEW_CMD;
      status_1e_ = APPLESMC_ST_1E_BAD_CMD;
    }
    pos_ = 0;
    data_pos_ = 0;
  }

  void WriteData(uint8_t data) {
    if (command_ == APPLESMC_READ_CMD) {
      if (pos_ < 4) {
        key_[pos_++] = (char)data;
        status_ = APPLESMC_ST_ACK;
      } else {
        auto it = smc_data_.find(std::string(key_, 4));
        if (it != smc_data_.end()) {
          data_len_ = it->second.size();
          data_pos_ = 0;
          memcpy(data_, it->second.data(), data_len_);
          status_ = APPLESMC_ST_ACK | APPLESMC_ST_DATA_READY;
          status_1e_ = APPLESMC_ST_CMD_DONE;
        } else {
          if (debug_) {
            MV_WARN("key %.4s not found", key_);
          }
          status_ = APPLESMC_ST_CMD_DONE;
          status_1e_ = APPLESMC_ST_1E_NOEXIST;
        }
      }
    } else {
      MV_PANIC("invalid command 0x%x", command_);
      status_1e_ = APPLESMC_ST_1E_STILL_BAD_CMD;
    }
  }

  uint8_t ReadData() {
    uint8_t ret = 0;
    if (command_ == APPLESMC_READ_CMD) {
      if (status_ & APPLESMC_ST_DATA_READY) {
        if (data_pos_ < data_len_) {
          ret = data_[data_pos_++];
          if (data_pos_ == data_len_) {
            status_ = APPLESMC_ST_CMD_DONE;
          } else {
            status_ = APPLESMC_ST_ACK | APPLESMC_ST_DATA_READY;
          }
        }
      } else {
        MV_PANIC("no data ready");
      }
    } else {
      MV_PANIC("invalid command 0x%x", command_);
      status_1e_ = APPLESMC_ST_1E_STILL_BAD_CMD;
    }
    return ret;
  }

 public:
  AppleSmc() {
    set_default_parent_class("Ich9Lpc", "Piix3");

    AddIoResource(kIoResourceTypePio, DATA_PORT, 1, "Apple SMC Data");
    AddIoResource(kIoResourceTypePio, CMD_PORT, 1, "Apple SMC Command");
    AddIoResource(kIoResourceTypePio, ERROR_PORT, 1, "Apple SMC Error");
  }

  virtual void Reset() override {
    status_ = 0;
    status_1e_ = 0;
    command_ = 0;
    pos_ = 0;

    std::string osk = std::string("This is a dummy key!", 64);
    if (has_key("osk")) {
      osk = std::get<std::string>(key_values_["osk"]);
    }

    smc_data_["NATJ"] = std::string("\0", 1);
    smc_data_["MSSP"] = std::string("\0", 1);
    smc_data_["MSSD"] = std::string("\0x3", 1);
    smc_data_["REV "] = std::string("\x01\x13\x0f\x00\x00\x03", 6);
    smc_data_["OSK0"] = osk.substr(0, 32);
    if (osk.size() > 32) {
      smc_data_["OSK1"] = osk.substr(32, 32);
    } else {
      smc_data_["OSK1"] = std::string("\0", 1);
    }
  }

  virtual void Read(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) override {
    MV_ASSERT(offset == 0);
    MV_ASSERT(size == 1);
    if (resource->base == CMD_PORT) {
      *data = status_;
    } else if (resource->base == DATA_PORT) {
      *data = ReadData();
    } else if (resource->base == ERROR_PORT) {
      *data = status_1e_;
    } else {
      Device::Read(resource, offset, data, size);
    }
  }

  virtual void Write(const IoResource* resource, uint64_t offset, uint8_t* data, uint32_t size) override {
    MV_ASSERT(offset == 0);
    MV_ASSERT(size == 1);
    if (resource->base == CMD_PORT) {
      WriteCommand(*data);
    } else if (resource->base == DATA_PORT) {
      WriteData(*data);
    } else {
      Device::Write(resource, offset, data, size);
    }
  }
};

DECLARE_DEVICE(AppleSmc);
