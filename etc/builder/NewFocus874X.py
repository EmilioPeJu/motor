# MODULES
from iocbuilder import Device
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import AsynPort
from common import MotorLib

__all__ = ['NewFocus']

class NewFocus(AsynPort, Device):
    """ New Focus 874X controller """
    deviceCount = 0

    # Dependencies
    Dependencies = (MotorLib,)
    DbdFileList = ['devNewFocus']
    LibFileList = ['NewFocus']

    def __init__(self, name, IP, AXES, MOVINGPOLL=200, IDLEPOLL=1000):
        NewFocus.deviceCount += 1
        self.__dict__.update(locals())
        self.__super.__init__(name)

    def InitialiseOnce(self):
        print("# New Focus 874X controllers")
        print("# \tTotal devices: {0}".format(NewFocus.deviceCount))
        print('#')
        print("# nf874xCreateController(AsynPortName, ControllerIPPort, NumAxes, MovingPoll, IdlePoll)")

    def Initialise(self):
        print('nf874xCreateController(%(name)s, %(IP)s, %(AXES)d, %(MOVINGPOLL)d, %(IDLEPOLL)d)' % self.__dict__)

    # Arguments
    ArgInfo = makeArgInfo(__init__,
        name = Simple("Asyn port for motor record", str),
        IP = Simple("IP address of controller", str),
        AXES = Simple("Number of axes", int),
        MOVINGPOLL = Simple("Polling period during move (ms)", int),
        IDLEPOLL = Simple("Polling period when idle (ms)", int))
