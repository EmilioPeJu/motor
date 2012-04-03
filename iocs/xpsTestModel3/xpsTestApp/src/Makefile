TOP=../..

include $(TOP)/configure/CONFIG


# ------------------------
# Build an IOC Application
# ------------------------

PROD_IOC = xpsTest

# xpsTest.dbd will be installed into <top>/dbd
DBD += xpsTest.dbd

DATA+=xpsTestGUI

# xpsTest.dbd will be created from these files
xpsTest_DBD += base.dbd
xpsTest_DBD += motorSupport.dbd
xpsTest_DBD += asyn.dbd
xpsTest_DBD += devNewport.dbd
xpsTest_DBD += asSupport.dbd
xpsTest_DBD += utilityLinux.dbd
xpsTest_DBD += drvAsynIPPort.dbd

# xpsTest_registerRecordDeviceDriver.cpp will be created
# xpsTest.dbd
xpsTest_SRCS += xpsTest_registerRecordDeviceDriver.cpp

# These two lines are needed for non-vxWorks builds, such as Linux
xpsTest_SRCS_DEFAULT += xpsTestMain.cpp
xpsTest_SRCS_vxWorks += -nil-

# Add locally compiled object code
#xpsTest_SRCS +=

# The following adds object code from base/src/vxWorks
xpsTest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# This line says that this IOC Application depends on the
# xxx Support Module
xpsTest_LIBS += asyn
xpsTest_LIBS += motor
xpsTest_LIBS += Newport
xpsTest_LIBS += autosave
xpsTest_LIBS += utility

# We need to link this IOC Application against the EPICS Base libraries
xpsTest_LIBS += $(EPICS_BASE_IOC_LIBS)

# ---------------------------------------------------

# NOTE: To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
#
# The difference between a file with an "st" extension
# and one with an "stt" extension is:
# The "st" extension files get passed through the C preprocessor
# whereas the "stt" files do not.

#ifneq ($(SNCSEQ),)
    # This builds sncExample as a component of xpsTest
#    xpsTest_SNCFLAGS += +r
#    xpsTest_DBD      += sncExample.dbd
#    xpsTest_SRCS     += sncExample.stt
#    xpsTest_LIBS     += seq pv

    # The following builds sncExample as a standalone application
#    PROD_HOST           += sncExample
#    sncExample_SNCFLAGS += +m
#    sncExample_SRCS     += sncExample.stt
#    sncExample_LIBS     += seq pv
    # Here, we need to specify the dependency on EPICS Base libraries
    # again because this is a standalone application and
    # not part of xpsTest
#    sncExample_LIBS     += $(EPICS_BASE_HOST_LIBS)
#endif

DATA += xpsTest.req

include $(TOP)/configure/RULES

