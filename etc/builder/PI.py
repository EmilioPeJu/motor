from iocbuilder import Device, AutoSubstitution
from iocbuilder.modules.asyn import Asyn, AsynPort
from common import MotorLib
from iocbuilder.arginfo import *

class PiAsyn(Device):
    AutoInstantiate = True
    Dependencies = (Asyn, MotorLib)
    LibFileList=["PI_GCS2Support"]
    DbdFileList=["PI_GCS2Support"]
        
class piAsynController(AsynPort):
    Dependencies=(PiAsyn,)
    
    def __init__(self, name, port, numAxes, priority=0, stackSize=0, movingPollingRate=10, idlePollingRate=250):
        self.__super.__init__(name)
        self.__dict__.update(locals())
    
    def Initialise(self):
        print "# PI_GCS2_CreateController(const char *portName, const char* asynPort, int numAxes, int priority, int stackSize, int movingPollingRate, int idlePollingRate)"
        print 'PI_GCS2_CreateController("%(name)s", "%(port)s", %(numAxes)d, %(priority)d, %(stackSize)d, %(movingPollingRate)d, %(idlePollingRate)d)' % self.__dict__

    ArgInfo = makeArgInfo(__init__,
        name = Simple("Device Name"),
        port = Ident("Asyn Serial port", AsynPort),
        numAxes = Simple("Number of axes", int),
        priority = Simple("Priority", int),
        stackSize = Simple("Stack Size", int),
        movingPollingRate = Simple("Poll interval while motor is moving in ms", int),
        idlePollingRate = Simple("Poll interval while motor is stationary in ms", int))

class PI_Support_template(AutoSubstitution):
    TemplateFile="PI_Support.db"
    
class PI_SupportCtrl_template(AutoSubstitution):
    TemplateFile="PI_SupportCtrl.db"    
    
