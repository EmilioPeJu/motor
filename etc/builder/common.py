from iocbuilder import AutoSubstitution, Device, ModuleBase
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import Asyn, AsynPort

__all__ = ['basic_asyn_motor', 'MotorLib', 'MotorSimLib', 'MotorRecord']

class MotorLib(Device):
    Dependencies = (Asyn,)
    LibFileList = ['motor', 'softMotor']
    DbdFileList = ['motorSupport', 'devSoftMotor']
    AutoInstantiate = True

class MotorSimLib(Device):
    Dependencies = (Asyn, MotorLib)
    LibFileList = [ "motorSimSupport" ]
    DbdFileList = [ "motorSimSupport" ]
    AutoInstantiate = True    

class MotorRecord(ModuleBase):
    pass

class basic_asyn_motor(AutoSubstitution, MotorRecord):    
    TemplateFile = 'basic_asyn_motor.template' 
basic_asyn_motor.ArgInfo.descriptions["PORT"] = Ident(
	"Asyn port for motor record", AsynPort)

class motorUtil(AutoSubstitution, Device):
    # Substitution attributes
    TemplateFile = 'motorUtil.template'

    def PostIocInitialise(self):
        print 'motorUtilInit("%(P)s")' % self.args
