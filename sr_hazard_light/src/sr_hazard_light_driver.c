#include <libusb-1.0/libusb.h>
#include "sr_hazard_light/sr_hazard_light_driver.h"
#include <stdint.h>
#include <stdio.h>

#define PATLITE_VID 0x191A
#define PATLITE_PID 0x8003
#define PATLITE_ENDPOINT 1

static libusb_device_handle *patlite_handle = 0;

/*
 * Patlite is controlled with USB interrrupt OUT messages.
 * 8 bytes of data which contains status for all lights&buzzer.
 *
 * Nibbles:
 *  0  always zero
 *  1  always zero
 *  2  always zero
 *  3  always zero
 *  4  always zero
 *  5  buzzer type (0 off, 1-5 on/patterns, 8 no change)
 *  6  buzzer tone a (0 none, 1-13 on: 1760Hz - 3520Hz, 15 no change)
 *  7  buzzer tone b (0 none, 1-13 on: 1760Hz - 3520Hz, 15 no change)
 *  8     red light (0 off, 1 static, 2-5 patterns, 8 no change)
 *  9  yellow light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 10   green light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 11    blue light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 12   clear light (0 off, 1 static, 2-5 patterns, 8 no change)
 * 13  always zero
 * 14  always zero
 * 15  always zero
 *
 * tone Hz increases by 5.9463%
 */

int patlite_lights(int red, int yellow, int green, int blue, int clear) {
  uint8_t buf[8] = {0x00, 0x00, 0x08, 0xff, (red<<4) + yellow, (green<<4) + blue, (clear<<4), 0x00};

  if (red > 9 || yellow > 9 || green > 9 || blue > 9 || clear > 9)
    return 1;
  
  return patlite_set(buf);
}
int patlite_buzzer(int type, int tonea, int toneb) {
  uint8_t buf[8] = {0x00, 0x00, type, (tonea<<4) + toneb, 0x88, 0x88, 0x80, 0x00};

  if (tonea > 15 || toneb > 15)
    return 1;
  
  return patlite_set(buf);
}

int patlite_set(uint8_t *buf) {
  int r;

  //Do we have valid handle?
  if (!patlite_handle) {
    r = patlite_init();
    if (r)
      return r;
  }

  int rs=0;
  r = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT, buf, 8, &rs, 1000);

  if (r) {
    printf("Patlite set failed, return %d\n", r);
    libusb_close(patlite_handle);
    patlite_handle=0;
    return 2;
  }

  return 0;
}

int patlite_init() {
  int r;
  r = libusb_init(NULL);
  if (r < 0) {
    printf("libusb init failed, err %d\n", r);
    return r;
  }

  patlite_handle = libusb_open_device_with_vid_pid(NULL, PATLITE_VID, PATLITE_PID);
  if (patlite_handle == NULL) {
    printf("patlite device not found\n");
    return -1;
  }

  libusb_set_auto_detach_kernel_driver(patlite_handle, 1);

  r = libusb_claim_interface(patlite_handle, 0);
  if (r != LIBUSB_SUCCESS) {
    printf("libusb claim failed\n");
  }

  uint8_t patlite_buf[8] = {0x00, 0x00, 0x08, 0xff, 0x00, 0x00, 0x00, 0x00};

  int rs=0;
  r = libusb_interrupt_transfer(patlite_handle, PATLITE_ENDPOINT, patlite_buf, sizeof(patlite_buf), &rs, 1000);

  if (r != 0) {
    printf("Patlite reset failed, return %d, transferred %d\n", r, rs);
    libusb_close(patlite_handle);
    patlite_handle=0;
    return -1;
  }

  printf("Patlite USB device opened and claimed\n");

  return 0;
}