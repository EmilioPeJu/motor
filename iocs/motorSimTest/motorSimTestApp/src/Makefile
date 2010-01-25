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
motorSimTest_DBD += devIocStats.dbd

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
motorSimTest_LIBS += devIocStats


# We need to link this IOC Application against the EPICS Base libraries
motorSimTest_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

# NOTE: To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
#
# The difference between a file with an "st" extension
# and one with an "stt" extension is:
# The "st" extension files get passed through the C preprocessor
# whereas the "stt" files do not.

#ifneq ($(SNCSEQ),)
    # This builds sncExample as a component of motorSimTest
#    motorSimTest_SNCFLAGS += +r
#    motorSimTest_DBD      += sncExample.dbd
#    motorSimTest_SRCS     += sncExample.stt
#    motorSimTest_LIBS     += seq pv

    # The following builds sncExample as a standalone application
#    PROD_HOST           += sncExample
#    sncExample_SNCFLAGS += +m
#    sncExample_SRCS     += sncExample.stt
#    sncExample_LIBS     += seq pv
    # Here, we need to specify the dependency on EPICS Base libraries
    # again because this is a standalone application and
    # not part of motorSimTest
#    sncExample_LIBS     += $(EPICS_BASE_HOST_LIBS)
#endif

DATA+=motorSimTestGUI
DATA+=motorSimTest.req

include $(TOP)/configure/RULES
