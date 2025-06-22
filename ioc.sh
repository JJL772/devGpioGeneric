#!/usr/bin/env bash

cd iocBoot/ioc-tst-gpio

$DEBUGGER ../../bin/$EPICS_HOST_ARCH/devGpioGenericTest st.cmd
