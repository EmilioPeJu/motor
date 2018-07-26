TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
# In a Diamond IOC Application, build db files from
# template files like this
#
DB += motorSimTest.db

DB += limit_alarm.template

#USES_TEMPLATE += $(MOTOR)/db/motor.template
#USES_TEMPLATE += $(MOTOR)/db/basic_motor.template

USES_TEMPLATE += $(MOTOR)/db/basic_asyn_motor.template
USER_TEMPLATE += ${USER}/db/ao.template

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
