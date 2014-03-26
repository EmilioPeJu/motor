from iocbuilder import AutoSubstitution, Device, ModuleBase
from iocbuilder.modules.asyn import Asyn
from iocbuilder.arginfo import *
from common import MotorLib, MotorRecord

__all__ = ['Xps', 'xps_asyn_motor']

class Xps(Asyn, Device):
    ''' XPS controller Device - instatiate one per controller '''
    deviceCount = 0
    Dependencies = [MotorLib]
    DbdFileList = ['devNewport']
    LibFileList = ['Newport']
    
    # Constructor, just stores the parameters
    def __init__(self, name=None, CARD=None, IP=None, PORT=5001, AXES=8, ACTIVEPOLL=50, IDLEPOLL=500, **args):
        Xps.deviceCount += 1
        # put the args into the self.* dictionary
        self.__dict__.update(locals())
        self.__dict__.update(**args)
        self.__super.__init__(**args)

    # startup script - once per class
    def InitialiseOnce(self):
        print 'XPSSetup(%d)' % Xps.deviceCount
        
    # startup script - once per instatiation
    def Initialise(self):
        print 'XPSConfig(%(CARD)d, %(IP)s, %(PORT)d, %(AXES)d, %(ACTIVEPOLL)d, %(IDLEPOLL)d)' % self.__dict__
        print 'drvAsynMotorConfigure( %(name)s, "motorXPS", %(CARD)d,%(AXES)d)' % self.__dict__
      
    # Arguments
    ArgInfo = makeArgInfo(__init__, 
        CARD = Simple("XPS Controller number", int),
        IP = Simple("IP Address of XPS controller ", str),
        PORT = Simple("Telnet PORT for XPS controller", int),
        AXES = Simple("Number of Axes", int),
        ACTIVEPOLL = Simple("polling period when motors are moving", int),
        IDLEPOLL = Simple("polling period when all motors are idle", int),
        name = Simple("Asyn port for motor record", str)  )
        

class xps_asyn_motorT(AutoSubstitution, MotorRecord):    
    Dependencies = [Xps]
    TemplateFile = 'basic_asyn_motor.template' 
xps_asyn_motorT.ArgInfo.descriptions["PORT"] = Ident(
    "Asyn port for motor record", Xps)


class xps_asyn_motor(xps_asyn_motorT, Device):  
    ''' XPS axis - instatiate one per axis '''
    
    # Constructor, just stores the parameters
    def __init__(self, CARD=None, POSITIONER=None, STEPS=1000, **args):
        # put the args into the self.* dictionary
        self.__dict__.update(locals())
        self.__dict__.update(**args)
        self.__super.__init__(**args)
        
    # startup script - once per instatiation
    def Initialise(self):
        print 'XPSConfigAxis(%(CARD)d, %(ADDR)s, %(POSITIONER)s, %(STEPS)d)' % self.__dict__
      
    # Arguments
    ArgInfo = xps_asyn_motorT.ArgInfo + makeArgInfo(__init__,
        CARD = Simple("XPS Controller number", int),
        POSITIONER = Simple("XPS Positioner name of the form G1.P1", str),
        STEPS = Simple("Steps per user unit", int) )    
    
