from common import MotorLib, MotorRecord, basic_asyn_motor
from iocbuilder import AutoSubstitution, Device, ModuleBase
from iocbuilder.arginfo import *
from iocbuilder.modules.asyn import AsynPort, Asyn

__all__ = ['MotorSim']

# noinspection PyPep8Naming
class MotorSim(AsynPort, Device):
    """Simulated controller Device - instantiate one per controller '''"""
    deviceCount = 0
    Dependencies = (MotorLib,)
    DbdFileList = ['motorSim']
    LibFileList = ['motorSimSupport']

    # Constructor, just stores the parameters
    def __init__(self, name, AXES=8, ACTIVEPOLL=50, IDLEPOLL=500, **args):
        self.card_no = MotorSim.deviceCount
        MotorSim.deviceCount += 1
        # put the args into the self.* dictionary
        self.__dict__.update(locals())
        self.__dict__.update(**args)
        self.__super.__init__(name)

    # startup script - once per class
    @staticmethod
    def InitialiseOnce():
        print(
            '# Create simulator: ( start card , start axis , hard low limit, '
            'hard high limit, home posn, # cards, # axes to setup)')
        print('motorSimCreate( 0, 1, -15000000, 15000000, 0, {}, 32 )'.format(
            MotorSim.deviceCount))

    # startup script - once per instatiation
    def Initialise(self):
        print('drvAsynMotorConfigure("{}", "motorSim", {}, {})'.format(
            self.name, self.card_no, self.AXES))

    # Arguments
    ArgInfo = makeArgInfo(__init__,
                          AXES=Simple("Number of Axes", int),
                          ACTIVEPOLL=Simple(
                              "polling period when motors are moving", int),
                          IDLEPOLL=Simple(
                              "polling period when all motors are idle",
                              int),
                          name=Simple("Asyn port for motor record", str))
