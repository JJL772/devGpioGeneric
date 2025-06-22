#!../../bin/linux-aarch64/devGpioGenericTest

#- SPDX-FileCopyrightText: 2003 Argonne National Laboratory
#-
#- SPDX-License-Identifier: EPICS

#- You may have to change devGpioGenericTest to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/devGpioGenericTest.dbd"
devGpioGenericTest_registerRecordDeviceDriver pdbbase

# Load record instances
dbLoadRecords("db/raspberrypi5.db","P=TEST:")

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncxxx,"user=jeremy"
