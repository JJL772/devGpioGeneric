# SPDX-FileCopyrightText: 2003 Argonne National Laboratory
#
# SPDX-License-Identifier: EPICS

TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = devGpioGenericTest
# devGpioGenericTest.dbd will be created and installed
DBD += devGpioGenericTest.dbd

# devGpioGenericTest.dbd will be made up from these files:
devGpioGenericTest_DBD += base.dbd

# Include dbd files from all support applications:
#devGpioGenericTest_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#devGpioGenericTest_LIBS += xxx

# devGpioGenericTest_registerRecordDeviceDriver.cpp derives from devGpioGenericTest.dbd
devGpioGenericTest_SRCS += devGpioGenericTest_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
devGpioGenericTest_SRCS_DEFAULT += devGpioGenericTestMain.cpp
devGpioGenericTest_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#devGpioGenericTest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
devGpioGenericTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

