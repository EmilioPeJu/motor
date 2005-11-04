#!../../bin/linux-x86/pmacsim

## You may have to change iocapp to something else
## everywhere it appears in this file

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase("dbd/pmacsim.dbd",0,0)
pmacsim_registerRecordDeviceDriver(pdbbase)

## Load record instances
dbLoadRecords("db/pmacsimmotor.template","device=test,c=0,s=1,PREC=5,VMAX=2000,VELO=2000,ACCL=1")
dbLoadRecords("db/pmacsimmotor.template","device=test,c=0,s=2,PREC=5,VMAX=2000,VELO=2000,ACCL=1")
dbLoadRecords("db/pmacsimmotor.template","device=test,c=0,s=3,PREC=5,VMAX=2000,VELO=2000,ACCL=1")
dbLoadRecords("db/pmacsimmotor.template","device=test,c=0,s=4,PREC=5,VMAX=2000,VELO=2000,ACCL=1")
dbLoadRecords("db/pmacsimmotor.template","device=test,c=0,s=5,PREC=5,VMAX=2000,VELO=2000,ACCL=1")
dbLoadRecords("db/pmacsimmotor.template","device=test,c=0,s=6,PREC=5,VMAX=2000,VELO=2000,ACCL=1")


# Configure PMAC
var simulation_mode 1
PmacSetup(0,0,0,0,0,0,0)

cd ${TOP}/iocBoot/${IOC}
iocInit()

## Start any sequence programs
#seq sncxxx,"user=mnHost"
