#Makefile at top of application tree
# "#!" marks lines that can be uncommented.
TOP = .
include $(TOP)/configure/CONFIG
DIRS := $(DIRS) $(filter-out $(DIRS), configure)
DIRS := $(DIRS) $(filter-out $(DIRS), motorApp)
#!DIRS := $(DIRS) $(filter-out $(DIRS), motorExApp)
#!DIRS := $(DIRS) $(filter-out $(DIRS), iocBoot)
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard etc))
# Comment out the following line to disable building of example iocs
DIRS := $(DIRS) $(filter-out $(DIRS), $(wildcard iocs))
include $(TOP)/configure/RULES_TOP
