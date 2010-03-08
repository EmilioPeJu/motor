TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) dbd files
# <name>.dbd will be created from <name>Include.dbd
#DBD += motor.dbd

#----------------------------------------------------
# Create and install (or just install)
# databases, templates, substitutions like this
DB += motor.template
DB += basic_motor.template
DB += basic_asyn_motor.template
DB += XPS_extra.db
DB += TransPos.template
DB += motorUtil.template
DB += motorReadback.template
DB += motorTranslate.template
DB += trajectoryScan.db
DB += softMotorForPiezo.template

#----------------------------------------------------
# Declare template files which do not show up in DB
#USES_TEMPLATE += motor.template
DATA += basic_motor_settings.req
DATA += motor_positions.req
DATA += motor_settings.req
DATA += pseudoMotor_settings.req

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
