#include "selfdrive/pandad/panda.h"

#include <cassert>
#include <memory>
#include <stdexcept>

#include <libusb-1.0/libusb.h>

static libusb_context *init_usb_ctx() {
  libusb_context *context = nullptr;
  int err = libusb_init(&context);
  if (err != 0) {
    return nullptr;
  }
  return context;
}

PandaUsbHandle::PandaUsbHandle(std::string serial) {
  ssize_t num_devices;
  libusb_device **dev_list = NULL;
  int err = 0;
  ctx = init_usb_ctx();
  if (!ctx) goto fail;

  num_devices = libusb_get_device_list(ctx, &dev_list);
  if (num_devices < 0) goto fail;
  for (size_t i = 0; i < (size_t)num_devices; ++i) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev_list[i], &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      int ret = libusb_open(dev_list[i], &dev_handle);
      if (dev_handle == NULL || ret < 0) goto fail;

      unsigned char desc_serial[26] = {0};
      ret = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      if (ret < 0) goto fail;

      hw_serial = std::string((char *)desc_serial, ret);
      if (serial.empty() || serial == hw_serial) break;

      libusb_close(dev_handle);
      dev_handle = NULL;
    }
  }
  if (dev_handle == NULL) goto fail;
  libusb_free_device_list(dev_list, 1);
  dev_list = nullptr;

  err = libusb_claim_interface(dev_handle, 0);
  if (err != 0) goto fail;

  return;

fail:
  if (dev_list != NULL) libusb_free_device_list(dev_list, 1);
  cleanup();
  throw std::runtime_error("Error connecting to panda");
}

PandaUsbHandle::~PandaUsbHandle() {
  std::lock_guard lk(hw_lock);
  cleanup();
  connected = false;
}

void PandaUsbHandle::cleanup() {
  if (dev_handle) {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
    dev_handle = NULL;
  }

  if (ctx) {
    libusb_exit(ctx);
    ctx = NULL;
  }
}

std::vector<std::string> PandaUsbHandle::list() {
  static std::unique_ptr<libusb_context, decltype(&libusb_exit)> context(init_usb_ctx(), libusb_exit);
  ssize_t num_devices;
  libusb_device **dev_list = NULL;
  std::vector<std::string> serials;
  if (!context) return serials;

  num_devices = libusb_get_device_list(context.get(), &dev_list);
  if (num_devices < 0) goto finish;
  for (size_t i = 0; i < (size_t)num_devices; ++i) {
    libusb_device *device = dev_list[i];
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(device, &desc);
    if (desc.idVendor == 0x3801 && desc.idProduct == 0xddcc) {
      libusb_device_handle *handle = NULL;
      int ret = libusb_open(device, &handle);
      if (ret < 0) goto finish;

      unsigned char desc_serial[26] = {0};
      ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      libusb_close(handle);
      if (ret < 0) goto finish;

      serials.push_back(std::string((char *)desc_serial, ret));
    }
  }

finish:
  if (dev_list != NULL) libusb_free_device_list(dev_list, 1);
  return serials;
}

void PandaUsbHandle::handle_usb_issue(int err, const char[]) {
  if (err == LIBUSB_ERROR_NO_DEVICE) {
    connected = false;
  }
}

int PandaUsbHandle::control_write(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

  if (!connected) return LIBUSB_ERROR_NO_DEVICE;

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, NULL, 0, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::control_read(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout) {
  int err;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

  if (!connected) return LIBUSB_ERROR_NO_DEVICE;

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout);
    if (err < 0) handle_usb_issue(err, __func__);
  } while (err < 0 && connected);

  return err;
}

int PandaUsbHandle::bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) return 0;

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);
    if (err == LIBUSB_ERROR_TIMEOUT) {
      break;
    } else if (err != 0 || length != transferred) {
      handle_usb_issue(err, __func__);
    }
  } while (err != 0 && connected);

  return transferred;
}

int PandaUsbHandle::bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  int err;
  int transferred = 0;

  if (!connected) return 0;

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
