
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

#include <linux/gpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <map>
#include <string>
#include <vector>

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
  int fd; /* fd for I/O */
  int num; /* index into offsets[] array */
  int line; /* actual line number */
  enum gpio_polarity polarity;
  enum gpio_type type;
  enum gpio_edge edge;
  enum gpio_bias bias;
  enum gpio_drive drive;
};

struct gpio_chip {
  int fd;
  int num_lines;
  bool failed; /* set once we've tried to init and failed */
  std::vector<gpio_pin> pins;
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
  return pchip;
}

static int
gpio_find_or_add_pin(gpio_chip* chip, int line, enum gpio_type type)
{
  /* try to find existing */
  for (int i = 0; i < chip->pins.size(); ++i) {
    if (chip->pins[i].line == line)
      return i;
  }

  struct gpio_pin pin = {};
  pin.type = type;
  pin.polarity = GPIO_ACTIVE_HIGH;
  pin.line = line;
  pin.edge = GPIO_EDGE_RISING;
  pin.fd = -1;
  
  chip->pins.push_back(pin);
  return chip->pins.size()-1;
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
dpvt_create(const char* instio, enum gpio_type type)
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
  dpvt->pin = gpio_find_or_add_pin(pchip, line, type);

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
  req.num_lines = chip->pins.size();
  strcpy(req.consumer, "epics");

  /* 1 line per fd */
  req.offsets[0] = p.line;
  req.num_lines = 1;

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
    /* configure the edge we're looking for */
    switch(p.edge) {
    case GPIO_EDGE_FALLING:
      config.flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING;
      break;
    case GPIO_EDGE_RISING:
      config.flags |= GPIO_V2_LINE_FLAG_EDGE_RISING;
      break;
    default:
      assert(0);
    }
    
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

bidset devGpioBi = {
  .common = {
    .number = 5,
    .init_record = devGpioBi_InitRecord,
  },
  .read_bi = devGpioBi_Read,
};

epicsExportAddress(dset, devGpioBi);

static long
devGpioBi_InitRecord(dbCommon* precord)
{
  auto* pbi = reinterpret_cast<biRecord*>(precord);
  precord->dpvt = dpvt_create(pbi->inp.value.instio.string, GPIO_TYPE_INPUT);

  if (!precord->dpvt)
    return S_dev_badInpType;

  return 0;
}

static long
devGpioBi_Read(biRecord* precord)
{
  auto* dpvt = dpvt_get(precord);
  auto& p = dpvt->chip->pins[dpvt->pin];
  if (p.type != GPIO_TYPE_INPUT)
    return 0; /* silently eat the error because we could have a scan rate */

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
  auto* dpvt = dpvt_create(pbo->out.value.instio.string, GPIO_TYPE_OUTPUT);
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
  dpvt->pin = gpio_find_or_add_pin(dpvt->chip, line, GPIO_TYPE_INPUT);
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
