# Makefile
TOP = ../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

DBD += devDeltaTau.dbd

LIBRARY_HOST = DeltaTau

SRCS += devPmac.cc drvPmac.cc
SRCS += PmacRegister.cc


SHRLIB_SEARCH_DIRS += /home/tools/python2.4/lib/python2.4/config

DeltaTau_LIBS += $(EPICS_BASE_IOC_LIBS)
DeltaTau_LIBS += motor
DeltaTau_LIBS += python2.4
                                                                                
USR_INCLUDES += -I/home/tools/python2.4/include/python2.4

# Simulation scripts
SCRIPTS += pmacsimulation.py 
SCRIPTS += dpramsimulation.py


# Build an IOC application
PROD_HOST = pmacsim

DBD += pmacsim.dbd

pmacsim_DBD += base.dbd
pmacsim_DBD += motorSupport.dbd
#pmacsim_DBD += devSoftMotor.dbd
pmacsim_DBD += devDeltaTau.dbd


pmacsim_SRCS += pmacsim_registerRecordDeviceDriver.cpp
pmacsim_SRCS_DEFAULT += pmacsimMain.cpp
pmacsim_SRCS_vxWorks += -nil-

pmacsim_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

pmacsim_LIBS += $(EPICS_BASE_IOC_LIBS)
pmacsim_LIBS += motor
pmacsim_LIBS += python2.4

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

vpath %.py $(USR_VPATH) $(ALL_SRC_DIRS)
