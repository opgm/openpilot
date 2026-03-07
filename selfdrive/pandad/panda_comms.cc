#include "selfdrive/pandad/panda_comms.h"

#include <memory>
#include <stdexcept>

#include <libusb-1.0/libusb.h>

namespace {

bool is_panda_device(const libusb_device_descriptor &desc) {
  const bool valid_vid = desc.idVendor == 0xbbaa || desc.idVendor == 0x3801;
  const bool valid_pid = desc.idProduct == 0xddee || desc.idProduct == 0xddcc;
  return valid_vid && valid_pid;
}

libusb_context *init_usb_ctx() {
  libusb_context *context = nullptr;
  if (libusb_init(&context) != 0) {
    return nullptr;
  }
  return context;
}

}  // namespace

PandaUsbHandle::PandaUsbHandle(std::string serial) {
  ssize_t num_devices;
  libusb_device **dev_list = nullptr;
  ctx = init_usb_ctx();
  if (ctx == nullptr) {
    goto fail;
  }

  num_devices = libusb_get_device_list(ctx, &dev_list);
  if (num_devices < 0) {
    goto fail;
  }

  for (size_t i = 0; i < static_cast<size_t>(num_devices); ++i) {
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev_list[i], &desc) != 0 || !is_panda_device(desc)) {
      continue;
    }

    libusb_device_handle *candidate = nullptr;
    int ret = libusb_open(dev_list[i], &candidate);
    if (ret < 0 || candidate == nullptr) {
      continue;
    }

    unsigned char desc_serial[26] = {0};
    ret = libusb_get_string_descriptor_ascii(candidate, desc.iSerialNumber, desc_serial, sizeof(desc_serial));
    if (ret < 0) {
      libusb_close(candidate);
      continue;
    }

    std::string candidate_serial(reinterpret_cast<char *>(desc_serial), ret);
    if (!serial.empty() && serial != candidate_serial) {
      libusb_close(candidate);
      continue;
    }

    dev_handle = candidate;
    hw_serial = candidate_serial;
    break;
  }

  if (dev_handle == nullptr) {
    goto fail;
  }

  libusb_free_device_list(dev_list, 1);
  dev_list = nullptr;

  libusb_set_auto_detach_kernel_driver(dev_handle, 1);
  if (libusb_claim_interface(dev_handle, 0) != 0) {
    goto fail;
  }

  return;

fail:
  if (dev_list != nullptr) {
    libusb_free_device_list(dev_list, 1);
  }
  cleanup();
  throw std::runtime_error("Error connecting to panda over USB");
}

PandaUsbHandle::~PandaUsbHandle() {
  std::lock_guard lk(hw_lock);
  cleanup();
  connected = false;
}

void PandaUsbHandle::cleanup() {
  if (dev_handle != nullptr) {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    dev_handle = nullptr;
  }

  if (ctx != nullptr) {
    libusb_exit(ctx);
    ctx = nullptr;
  }
}

std::vector<std::string> PandaUsbHandle::list() {
  static std::unique_ptr<libusb_context, decltype(&libusb_exit)> context(init_usb_ctx(), libusb_exit);
  std::vector<std::string> serials;
  libusb_device **dev_list = nullptr;

  if (!context) {
    return serials;
  }

  ssize_t num_devices = libusb_get_device_list(context.get(), &dev_list);
  if (num_devices < 0) {
    return serials;
  }

  for (size_t i = 0; i < static_cast<size_t>(num_devices); ++i) {
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev_list[i], &desc) != 0 || !is_panda_device(desc)) {
      continue;
    }

    libusb_device_handle *handle = nullptr;
    int ret = libusb_open(dev_list[i], &handle);
    if (ret < 0 || handle == nullptr) {
      continue;
    }

    unsigned char desc_serial[26] = {0};
    ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, desc_serial, sizeof(desc_serial));
    libusb_close(handle);
    if (ret <= 0) {
      continue;
    }

    std::string serial(reinterpret_cast<char *>(desc_serial), ret);
    if (serial.size() == 24) {
      serials.push_back(serial);
    }
  }

  libusb_free_device_list(dev_list, 1);
  return serials;
}

void PandaUsbHandle::handle_usb_issue(int err, const char[]) {
  if (err == LIBUSB_ERROR_NO_DEVICE) {
    connected = false;
  }
}

int PandaUsbHandle::control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout) {
  int err;
  const uint8_t request_type = static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT) |
                               static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                               static_cast<uint8_t>(LIBUSB_RECIPIENT_DEVICE);

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, request_type, request, param1, param2, nullptr, 0, timeout);
    if (err < 0) {
      handle_usb_issue(err, __func__);
    }
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout) {
  int err;
  const uint8_t request_type = static_cast<uint8_t>(LIBUSB_ENDPOINT_IN) |
                               static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_VENDOR) |
                               static_cast<uint8_t>(LIBUSB_RECIPIENT_DEVICE);

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, request_type, request, param1, param2, data, length, timeout);
    if (err < 0) {
      handle_usb_issue(err, __func__);
    }
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::bulk_write(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
    if (err == LIBUSB_ERROR_TIMEOUT) {
      break;
    } else if (err != 0 || transferred != length) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected);

  return transferred;
}

int PandaUsbHandle::bulk_read(unsigned char endpoint, unsigned char *data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
    if (err == LIBUSB_ERROR_TIMEOUT) {
      break;
    } else if (err == LIBUSB_ERROR_OVERFLOW) {
      comms_healthy = false;
    } else if (err != 0) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected);

  return transferred;
}
