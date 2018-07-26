TOP = ..
include $(TOP)/configure/CONFIG

# MotorSrc is required for all motor record configurations.  It MUST appear
# 1st in this list of directories.

# "#!" marks lines that can be uncommented.

DIRS += MotorSrc

# Select individual device/driver modules by removing the "#!" comment.
#DIRS += DeltaTauSrc
DIRS += OmsSrc
DIRS += SoftMotorSrc

# All the following modules require ASYN.
ifdef ASYN
DIRS += OmsAsynSrc
OmsAsynSrc_DEPEND_DIRS   = MotorSrc

DIRS += MotorSimSrc
DIRS += NewportSrc
#DIRS += ImsSrc
#DIRS += AcsSrc
#DIRS += MclennanSrc
DIRS += PiSrc
DIRS += PIGCS2Src
PIGCS2Src_DEPEND_DIRS = MotorSrc

DIRS += MicroMoSrc
#DIRS += MicosSrc
#DIRS += FaulhaberSrc
#DIRS += PC6KSrc
DIRS += NewFocusSrc
DIRS += AcsTech80Src
#DIRS += OrielSrc
#DIRS += ThorLabsSrc
#DIRS += SmartMotorSrc
DIRS += PiJenaSrc
DIRS += KohzuSrc
DIRS += AerotechSrc
#DIRS += HytecSrc
#HytecSrc_DEPEND_DIRS = MotorSrc

#DIRS += ACRSrc
#ACRSrc_DEPEND_DIRS = MotorSrc

DIRS += SmarActMCSSrc
SmarActMCSSrc_DEPEND_DIRS = MotorSrc

DIRS += NPointSrc
NPointSrc_DEPEND_DIRS = MotorSrc

DIRS += MicronixSrc
MicronixSrc_DEPEND_DIRS = MotorSrc

DIRS += PhytronSrc
PhytronSrc_DEPEND_DIRS = MotorSrc

DIRS += AMCISrc
AMCISrc_DEPEND_DIRS = MotorSrc

endif

# Install the edl files
DIRS += opi

ifdef MX
DIRS += MXmotorSrc
endif

DIRS += Db
include $(TOP)/configure/RULES_DIRS
