//-----------------------------------------------------------------------------------------
// Copyright (C) 2025, Jeremy Lorelli
//-----------------------------------------------------------------------------------------
// Purpose: Generic GPIO device support for Linux
//-----------------------------------------------------------------------------------------
// This file is part of 'devGpioGeneric'. It is subject to the license terms in the
// LICENSE file found in the top-level directory of this distribution.
// No part of 'devGpioGeneric', including this file, may be copied, modified, propagated,
// or otherwise distributed except according to the terms contained in the LICENSE file.
//
// SPDX-License-Identifier: BSD-3-Clause
//-----------------------------------------------------------------------------------------

#define USE_TYPED_DSET

#include <iocsh.h>
#include <boRecord.h>
#include <biRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsExport.h>
#include <dbCommon.h>
#include <recGbl.h>
#include <alarm.h>
#include <drvSup.h>
#include <epicsThread.h>
#include <dbScan.h>
#include <epicsAtomic.h>
#include <iocInit.h>
#include <initHooks.h>

#include <linux/gpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <poll.h>

#include <map>
#include <string>
#include <vector>

#define MAX_PINS_PER_CHIP 32

enum gpio_polarity {
  GPIO_ACTIVE_HIGH = 0,
  GPIO_ACTIVE_LOW,
};

enum gpio_type {
  GPIO_TYPE_INPUT = 0,
  GPIO_TYPE_OUTPUT,
};

enum gpio_edge {
  GPIO_EDGE_RISING = 0,
  GPIO_EDGE_FALLING,
};

enum gpio_bias {
  GPIO_BIAS_NONE = 0,
  GPIO_BIAS_PULL_DOWN,
  GPIO_BIAS_PULL_UP
};

enum gpio_drive {
  GPIO_DRIVE_PUSH_PULL = 0,
  GPIO_DRIVE_OPEN_SOURCE,
  GPIO_DRIVE_OPEN_DRAIN,
};

struct gpio_pin {
  int fd = -1; /* fd for I/O */
  int num {}; /* index into offsets[] array */
  int line {}; /* actual line number */
  enum {
    VAL_NONE = 0,
    VAL_ON = 0x1,      /* currently 'high' */
    VAL_RISING = 0x2,  /* rising edge detected */
    VAL_FALLING = 0x4, /* falling edge detected */
  };
  int value = VAL_NONE;  /* value, set by watch events */
  enum gpio_polarity polarity = GPIO_ACTIVE_HIGH;
  enum gpio_type type = GPIO_TYPE_INPUT;
  enum gpio_edge edge = GPIO_EDGE_RISING;
  enum gpio_bias bias = GPIO_BIAS_NONE;
  enum gpio_drive drive = GPIO_DRIVE_PUSH_PULL;
  uint64_t ts;         /* hardware/kernel timestamp */
};

struct gpio_chip {
  int fd;
  int num_lines;
  bool failed; /* set once we've tried to init and failed */
  IOSCANPVT scan; /* trigger record processing when events happen */
  epicsThreadId thread;
  gpio_pin pins[MAX_PINS_PER_CHIP];
};

struct gpio_dpvt {
  struct gpio_chip* chip;
  int pin;
};

enum gpio_cfg_param {
  GPIO_CFG_EDGE,
  GPIO_CFG_POLARITY,
  GPIO_CFG_TYPE,
  GPIO_CFG_DRIVE,
  GPIO_CFG_BIAS,
};

struct gpio_cfg_dpvt {
  struct gpio_chip* chip;
  int pin;
  enum gpio_cfg_param param;
};

static std::map<std::string, gpio_chip*> s_chips;

static void gpio_thread_proc(void* param);

/**
 * Create a new gpio_chip object for the chip at the specified location in /dev
 * Returns nullptr on error
 */
static gpio_chip*
gpio_init_chip(const std::string& chip)
{
  auto c = chip;

  int fd = open(c.c_str(), O_RDWR);
  if (fd < 0) {
    printf("failed to open %s: %s\n", c.c_str(), strerror(errno));
    return nullptr;
  }

  auto* pchip = new gpio_chip();
  pchip->fd = fd;

  struct gpiochip_info info {};
  if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) < 0) {
    printf("failed to get gpio chip info for %s: %s\n", c.c_str(), strerror(errno));
    delete pchip;
    close(fd);
    return nullptr;
  }

  pchip->num_lines = info.lines;

  /* Init ioscan for the inputs */
  scanIoInit(&pchip->scan);

  return pchip;
}

static int
gpio_find_or_add_pin(gpio_chip* chip, int line)
{
  assert(line < MAX_PINS_PER_CHIP && line >= 0);

  /* try to find existing */
  auto& pin = chip->pins[line];
  if (pin.fd >= 0)
    return line;

  pin.type = GPIO_TYPE_INPUT;
  pin.polarity = GPIO_ACTIVE_HIGH;
  pin.line = line;
  pin.edge = GPIO_EDGE_RISING;
  pin.fd = -1;

  return line;
}

/**
 * Find or init a GPIO chip. This is the chip 'name', not including the /dev/ prefix
 */
static gpio_chip*
find_or_init_chip(const std::string& chip)
{
  auto it = s_chips.find(chip);
  if (it == s_chips.end()) {
    /* init a new chip */
    auto* pchip = gpio_init_chip(chip);
    if (!pchip)
      return nullptr;
    s_chips.insert({chip, pchip});
    return pchip;
  }
  return it->second;
}

/**
 * Create a generic dpvt type for gpio device types
 * Returns nullptr on failure
 */
static gpio_dpvt*
dpvt_create(const char* instio)
{
  char buf[512];
  strncpy(buf, instio, sizeof(buf)-1);
  buf[sizeof(buf)-1] = 0;

  /* instio string in the following format:
   * @/dev/gpiochip13,1 */

  const char* dev = strtok(buf, ",");
  if (!dev) {
    printf("Instio string missing dev specifier\n");
    return nullptr;
  }
  
  const char* num = strtok(nullptr, ",");
  if (!num) {
    printf("Instio string missing num specifier\n");
    return nullptr;
  }

  epicsUInt32 line;
  if (epicsParseUInt32(num, &line, 10, nullptr) != 0) {
    printf("Instio string has invalid num specifier. Must be an integer\n");
    return nullptr;
  }
  
  auto* pchip = find_or_init_chip(dev);
  if (!pchip) {
    printf("No such gpio chip %s\n", dev);
    return nullptr;
  }
  
  auto* dpvt = new gpio_dpvt();
  dpvt->chip = pchip;

  /* add it to the chip */
  dpvt->pin = gpio_find_or_add_pin(pchip, line);

  return dpvt;
}

template<typename RecT>
static gpio_dpvt*
dpvt_get(RecT* prec)
{
  return static_cast<gpio_dpvt*>( prec->dpvt );
}

/**
 * Configure all pins. This doesn't appropriately set flags yet, that is done
 * separately by gpio_reconfig_pin
 */
static bool
gpio_config_pin(gpio_chip* chip, int pin)
{
  auto& p = chip->pins[pin];
  if (p.fd >= 0)
    return true;

  struct gpio_v2_line_request req = {};
  strcpy(req.consumer, "epics");

  /* 1 line per fd */
  req.offsets[0] = p.line;
  req.num_lines = 1;

  /* might need tuning */
  req.event_buffer_size = 128;

  req.config.num_attrs = 0;
  
  if (ioctl(chip->fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
    perror("ioctl(GPIO_V2_GET_LINE_IOCTL)");
    return false;
  }
  p.fd = req.fd;
  return true;
}

/**
 * Reconfigure a gpio pin, returns true on success
 */
static bool
gpio_reconfig_pin(gpio_chip* chip, int pin)
{
  auto& p = chip->pins[pin];
  struct gpio_v2_line_config config = {};
  const auto watch_flags = GPIO_V2_LINE_FLAG_EDGE_FALLING | GPIO_V2_LINE_FLAG_EDGE_RISING;

  /* set polarity flags */
  switch(p.polarity) {
  case GPIO_ACTIVE_LOW:
    config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
    break;
  default:
    break;
  }

  /* set type flags */
  switch (p.type) {
  case GPIO_TYPE_INPUT:
    config.flags |= GPIO_V2_LINE_FLAG_INPUT;
    break;
  case GPIO_TYPE_OUTPUT:
    config.flags |= GPIO_V2_LINE_FLAG_OUTPUT;
    break;
  default:
    assert(0);
  }
  
  /* for inputs, configure the edge */
  if (p.type == GPIO_TYPE_INPUT) {
    config.flags |= watch_flags;

    /* configure pull up/down */
    switch(p.bias) {
    case GPIO_BIAS_PULL_DOWN:
      config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
      break;
    case GPIO_BIAS_PULL_UP:
      config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
      break;
    case GPIO_BIAS_NONE:
      /* no flag for this one */
      break;
    default:
      assert(0);
    }
  }
  else {
    /* configure output drive */
    switch(p.drive) {
    case GPIO_DRIVE_OPEN_DRAIN:
      config.flags |= GPIO_V2_LINE_FLAG_OPEN_DRAIN;
      break;
    case GPIO_DRIVE_OPEN_SOURCE:
      config.flags |= GPIO_V2_LINE_FLAG_OPEN_SOURCE;
      break;
    case GPIO_DRIVE_PUSH_PULL:
      /* no flag for this one */
      break;
    default:
      assert(0);
    }
  }
  
  config.num_attrs = 0;

  if (ioctl(p.fd, GPIO_V2_LINE_SET_CONFIG_IOCTL, &config) < 0) {
    perror("ioctl(GPIO_V2_LINE_SET_CONFIG_IOCTL)");
    return false;
  }
  return true;
}

/**************************** devGpioBi *****************************/

static long devGpioBi_InitRecord(dbCommon* precord);
static long devGpioBi_Read(biRecord* precord);
static long devGpioBi_Init(int after);
static long devGpioBi_GetIointInfo(int op, dbCommon* precord, IOSCANPVT* pvt);

bidset devGpioBi = {
  .common = {
    .number = 5,
    .init = devGpioBi_Init,
    .init_record = devGpioBi_InitRecord,
    .get_ioint_info = devGpioBi_GetIointInfo,
  },
  .read_bi = devGpioBi_Read,
};

epicsExportAddress(dset, devGpioBi);

static long
devGpioBi_Init(int after)
{
  if (after == 0)
    return 0;

  /* Kick off the watcher threads for all chips */
  for (auto& entry : s_chips) {
    auto* pchip = entry.second;
    pchip->thread = epicsThreadCreate("GPIO", epicsThreadPriorityHigh, 
      epicsThreadStackMedium, gpio_thread_proc, pchip); 
  }
  return 0;
}

static long
devGpioBi_InitRecord(dbCommon* precord)
{
  auto* pbi = reinterpret_cast<biRecord*>(precord);
  precord->dpvt = dpvt_create(pbi->inp.value.instio.string);

  if (!precord->dpvt)
    return S_dev_badInpType;

  return 0;
}

static long
devGpioBi_ReadSample(biRecord* precord, gpio_pin& p)
{
  struct gpio_v2_line_values req = {};
  req.bits = 1<<p.num;
  req.mask = 1<<p.num;

  if (ioctl(p.fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &req) < 0) {
    recGblSetSevrMsg(precord, COMM_ALARM, MAJOR_ALARM, "%s", strerror(errno));
    perror("ioctl(GPIO_V2_LINE_GET_VALUES_IOCTL)");
    return -1;
  }

  precord->rval = req.bits >> p.num;  
  return 0;
}

static long
devGpioBi_Read(biRecord* precord)
{
  auto* dpvt = dpvt_get(precord);
  auto& p = dpvt->chip->pins[dpvt->pin];
  if (p.type != GPIO_TYPE_INPUT)
    return 0; /* silently eat the error because we could have a scan rate */

  /* if configured with a fixed scan rate, directly sample hardware w/ioctl */
  if (precord->scan != menuScanI_O_Intr) {
    return devGpioBi_ReadSample(precord, p);
  }
  else {
    precord->rval = p.value & gpio_pin::VAL_ON;
  }

  return 0;
}

static long
devGpioBi_GetIointInfo(int op, dbCommon* precord, IOSCANPVT* pvt)
{
  auto* dpvt = dpvt_get(precord);
  *pvt = dpvt->chip->scan;
  return 0;
}

/**************************** devGpioBo *****************************/

static long devGpioBo_Init(int after);
static long devGpioBo_InitRecord(dbCommon* precord);
static long devGpioBo_Write(boRecord* precord);

bodset devGpioBo = {
  .common = {
    .number = 5,
    .init = devGpioBo_Init,
    .init_record = devGpioBo_InitRecord,
  },
  .write_bo = devGpioBo_Write,
};

epicsExportAddress(dset, devGpioBo);

static long
devGpioBo_Init(int after)
{
  if (!after)
    return 0;
  return 0;
}

static long
devGpioBo_InitRecord(dbCommon* precord)
{
  auto* pbo = reinterpret_cast<boRecord*>(precord);
  auto* dpvt = dpvt_create(pbo->out.value.instio.string);
  precord->dpvt = dpvt;

  if (!precord->dpvt)
    return S_dev_badOutType;
  
  if (!gpio_config_pin(dpvt->chip, dpvt->pin))
    return S_dev_badSignal;

  return 0;
}

static long
devGpioBo_Write(boRecord* precord)
{
  auto* dpvt = dpvt_get(precord);
  auto& p = dpvt->chip->pins[dpvt->pin];
  if (p.type != GPIO_TYPE_OUTPUT)
    return -1;

  struct gpio_v2_line_values req = {};
  req.mask = 1 << p.num;
  req.bits = precord->val << p.num;

  if (ioctl(p.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &req) < 0) {
    recGblSetSevrMsg(precord, COMM_ALARM, MAJOR_ALARM, "%s", strerror(errno));
    perror("ioctl(GPIO_V2_LINE_SET_VALUES_IOCTL)");
    return -1;
  }

  return 0;
}

/**************************** devGpioCfgMbbo *****************************/

static long devGpioCfgMbbo_InitRecord(dbCommon* precord);
static long devGpioCfgMbbo_WriteRecord(mbboRecord* precord);

mbbodset devGpioCfgMbbo = {
  .common = {
    .number = 5,
    .init_record = devGpioCfgMbbo_InitRecord,
  },
  .write_mbbo = devGpioCfgMbbo_WriteRecord
};

epicsExportAddress(dset, devGpioCfgMbbo);

static long
devGpioCfgMbbo_InitRecord(dbCommon* precord)
{
  auto* pmbbo = reinterpret_cast<mbboRecord*>(precord);
  
  char str[512];
  strncpy(str, pmbbo->out.value.instio.string, sizeof(str)-1);
  str[sizeof(str)-1] = 0;
  
  const char* dev = strtok(str, ",");
  if (!dev) {
    printf("instio string missing 'dev' specifier\n");
    return S_dev_badInpType;
  }
  
  const char* pin = strtok(NULL, ",");
  if (!pin) {
    printf("instio string missing 'pin' specifier\n");
    return S_dev_badInpType;
  }
  
  epicsUInt32 line;
  if (epicsParseUInt32(pin, &line, 10, NULL) != 0) {
    printf("instio string missing 'pin' specifier\n");
    return S_dev_badInpType;
  }
  
  const char* param = strtok(NULL, ",");
  if (!param) {
    printf("instio string missing 'param' specifier\n");
    return S_dev_badInpType;
  }
  
  enum gpio_cfg_param p;
  if (!strcmp(param, "edge"))
    p = GPIO_CFG_EDGE;
  else if (!strcmp(param, "polarity"))
    p = GPIO_CFG_POLARITY;
  else if (!strcmp(param, "type"))
    p = GPIO_CFG_TYPE;
  else if (!strcmp(param, "bias"))
    p = GPIO_CFG_BIAS;
  else if (!strcmp(param, "drive"))
    p = GPIO_CFG_DRIVE;
  else {
    printf("instio string has unknown cfg type '%s'\n", param);
    return S_dev_badInpType;
  }

  auto* dpvt = new gpio_cfg_dpvt;
  dpvt->chip = find_or_init_chip(dev);
  dpvt->pin = gpio_find_or_add_pin(dpvt->chip, line);
  dpvt->param = p;

  precord->dpvt = dpvt;

  return 0;
}

static long
devGpioCfgMbbo_WriteRecord(mbboRecord* precord)
{
  auto* dpvt = static_cast<gpio_cfg_dpvt*>(precord->dpvt);

  auto& p = dpvt->chip->pins[dpvt->pin];
  
  switch (dpvt->param) {
  case GPIO_CFG_EDGE:
    p.edge = static_cast<gpio_edge>(precord->val);
    break;
  case GPIO_CFG_POLARITY:
    p.polarity = static_cast<gpio_polarity>(precord->val);
    break;
  case GPIO_CFG_TYPE:
    p.type = static_cast<gpio_type>(precord->val);
    break;
  case GPIO_CFG_BIAS:
    p.bias = static_cast<gpio_bias>(precord->val);
    break;
  case GPIO_CFG_DRIVE:
    p.drive = static_cast<gpio_drive>(precord->val);
    break;
  default:
    assert(0);
  }

  if (!gpio_reconfig_pin(dpvt->chip, dpvt->pin)) {
    recGblSetSevr(precord, COMM_ALARM, MAJOR_ALARM);
    return 1;
  }

  return 0;
}

/**************************** Input Event Thread *****************************/

/* NOTE: This code assumes that we have one fd per pin. */
static void
gpio_thread_read(gpio_chip* chip, int pin, int fd)
{
  gpio_v2_line_event events[64];
  auto& p = chip->pins[pin];

  ssize_t r = read(fd, events, sizeof(events));

  uint32_t lowest = UINT32_MAX;
  for (ssize_t i = 0; i < r / (ssize_t)sizeof(events[0]); ++i) {
    int val = 0;
    switch (events[i].id) {
    case GPIO_V2_LINE_EVENT_RISING_EDGE:
      val = 1;
      p.value |= gpio_pin::VAL_RISING; break;
    case GPIO_V2_LINE_EVENT_FALLING_EDGE:
      p.value |= gpio_pin::VAL_FALLING; break;
    }

    /* I'm not sure if we can always assume gpio events will be ordered.
     * So..instead of sorting the events buffer, we'll just track the lowest
     * sequence number and assign value based on that */
    if (events[i].line_seqno <= lowest) {
      if (val)
        p.value |= gpio_pin::VAL_ON;
      else
        p.value &= ~gpio_pin::VAL_ON;
      lowest = events[i].line_seqno;
      p.ts = events[i].timestamp_ns;
    }
  }
}

static void
gpio_thread_proc(void* param)
{
  gpio_chip* chip = static_cast<gpio_chip*>(param);

  struct pollfd fds[MAX_PINS_PER_CHIP] = {};
  int num_open = 0;

  for (int i = 0; i < MAX_PINS_PER_CHIP; ++i) {
    if (chip->pins[i].fd < 0)
      continue;
    fds[i].fd = chip->pins[i].fd;
    fds[i].events = POLLIN;
    ++num_open;
  }

  while (num_open > 0) {
    bool any_read = false;
    int r = poll(fds, MAX_PINS_PER_CHIP, -1);
    if (r < 0) {
      perror("poll");
      continue;
    }

    for (int i = 0; i < MAX_PINS_PER_CHIP; ++i) {
      if (fds[i].revents == 0)
        continue;

      /* data ready for read */
      if (fds[i].revents & POLLIN) {
        gpio_thread_read(chip, i, fds[i].fd);
        any_read = true;
      }

      if (fds[i].revents & (POLLERR|POLLHUP))
        --num_open;
    }

    /* Kick off db scan now that we've updated values */
    if (any_read)
      scanIoRequest(chip->scan);
  }
}
