TOP=../..

include $(TOP)/configure/CONFIG


# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC = motorSimTest

# motorSimTest.dbd will be installed into <top>/dbd
DBD += motorSimTest.dbd

# motorSimTest.dbd will be created from these files
motorSimTest_DBD += base.dbd
motorSimTest_DBD += asyn.dbd
motorSimTest_DBD += motorSupport.dbd
motorSimTest_DBD += motorSimSupport.dbd
motorSimTest_DBD += utilityLinux.dbd
motorSimTest_DBD += asSupport.dbd

# motorSimTest_registerRecordDeviceDriver.cpp will be created
# motorSimTest.dbd
motorSimTest_SRCS += motorSimTest_registerRecordDeviceDriver.cpp

# These two lines are needed for non-vxWorks builds, such as Linux
motorSimTest_SRCS_DEFAULT += motorSimTestMain.cpp
motorSimTest_SRCS_vxWorks += -nil-

# Add locally compiled object code
#motorSimTest_SRCS +=

# The following adds object code from base/src/vxWorks
motorSimTest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# This line says that this IOC Application depends on the
# xxx Support Module
motorSimTest_LIBS += motorSimSupport
motorSimTest_LIBS += motor
motorSimTest_LIBS += asyn
motorSimTest_LIBS += utility
motorSimTest_LIBS += autosave


# We need to link this IOC Application against the EPICS Base libraries
motorSimTest_LIBS += $(EPICS_BASE_IOC_LIBS)

DATA+=motorSimTestGUI
DATA+=motorSimTest.req

include $(TOP)/configure/RULES
