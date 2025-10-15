# devGpioGeneric

EPICS device support for generic GPIO devices on Linux using the [chardev interface](https://docs.kernel.org/userspace-api/gpio/chardev.html).

## Supported Devices

* Raspberry Pi 5

Other devices supporting the Linux GPIO driver will work, but need DB templates written for them.

## Records

Each pin has the following associated records, where `$(P)` is provided by the IOC and `$(N)` is substituted with the pin number.
| Record Name | Type | Description |
|---|---|---|
| `$(P)GPIO$(N)_OUT` | bo | Output value, used when the pin is configured as an output |
| `$(P)GPIO$(N)_IN` | bi | Input value, used when the pin is configured as an input |
| `$(P)GPIO$(N)_POLARITY` | mbbo | Polarity for outputs. Default is "active high" |
| `$(P)GPIO$(N)_TYPE` | mbbo | Pin type. Default is Input |
| `$(P)GPIO$(N)_DRIVE` | mbbo | Drive type for outputs. Default is "Push/Pull" | 
| `$(P)GPIO$(N)_BIAS` | mbbo | Bias for inputs. Default is "None" |
| `$(P)GPIO$(N)_RESET` | bo | Resets the value when the pin is configured as a latched input |
| `$(P)GPIO$(N)_DEBOUNCE` | longin | Debounce sampling period in microseconds. Only relevant for input pins |

## Outputs

When `$(P)GPIO$(N)_TYPE` is set to `Output`, the pin is configured as an output. The pin can be driven high or low by writing to 
`$(P)GPIO$(N)_OUT`.

Wrties to `$(P)GPIO$(N)_OUT` will be rejected when the pin is not configured as an output.

## Inputs

When `$(P)GPIO$(N)_TYPE` is set to `Input`, the pin is configured as an input. The value of the pin is available through the record `$(P)GPIO$(N)_IN`.

Reads on `$(P)GPIO$(N)_IN` are no-ops when the pin is configured as an output.

Inputs have two operating modes: an event driven mode and a direct sampling mode.

### Direct Sampling Mode

Setting `$(P)GPIO$(N)_IN.SCAN` to a fixed scan rate enables the direct sampling mode, which directly samples the pin's value every time the record is processed.
Since scan rates are slow, it's recommended to use the event driven mode to avoid missing edge events on the pin.

### Event Mode

If `$(P)GPIO$(N)_IN.SCAN` is set to `I/O Intr` (the default), the input operates in event mode. In event mode, edges are detected by the kernel driver, usually via interrupts,
and are forwarded to the device support module where the values are cached. Edge events trigger record processing where the cached values are read by the device support.

## Latched Inputs

When `$(P)GPIO$(N)_TYPE` is set to `InputLatched`, the pin operates in a special latched input mode.

In latched mode, the input stays high when a rising edge (or falling edge, depending on the polarity setting) is detected.
The value can be reset by writing '1' to `$(P)GPIO$(N)_RESET`

Unlike the `Input` type, the pin always operates in the event mode regardless of `SCAN` setting.
