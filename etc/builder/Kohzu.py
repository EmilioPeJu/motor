from iocbuilder import Device, AutoSubstitution
from iocbuilder.modules.asyn import Asyn, AsynPort
from common import MotorLib
from iocbuilder.arginfo import *

class KohzuSc800(Device):
    Dependencies = (MotorLib,)
    LibFileList=["KohzuMotor"]
    DbdFileList=["devKohzuMotor"]
    NUMDEVICES=0
    SetupPlanted=False
        
    # Constructor, just store parameters
    def __init__(self, ASYNPORT):
        Device.__init__(self)
        self.ASYNPORT = ASYNPORT
        self.DEVICENUM = self.NUMDEVICES
        self.NUMDEVICES += 1

    # Once per instantiation
    def Initialise(self):
        if not self.SetupPlanted:
            print 'SC800Setup(%(NUMDEVICES)s, 10)' % self.__dict__
            self.SetupPlanted = True
        print 'SC800Config(%(DEVICENUM)s, "%(ASYNPORT)s", 0)' % self.__dict__

    # Arguments
    ArgInfo = makeArgInfo(__init__,
        ASYNPORT = Simple("Asyn port name", str))

