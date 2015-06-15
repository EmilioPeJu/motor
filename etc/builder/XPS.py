from iocbuilder import AutoSubstitution, Device, ModuleBase
from iocbuilder.modules.asyn import AsynPort, Asyn
from iocbuilder.modules.seq import Seq
from iocbuilder.arginfo import *
from common import MotorLib, MotorRecord, basic_asyn_motor

__all__ = ['Xps', 'xps_asyn_motor']

class Xps(AsynPort, Device):
    ''' XPS controller Device - instatiate one per controller '''
    deviceCount = 0
    Dependencies = (MotorLib,Seq)
    DbdFileList = ['devNewport']
    LibFileList = ['Newport']
    
    # Constructor, just stores the parameters
    def __init__(self, name, CARD, IP, PORT=5001, AXES=8, ACTIVEPOLL=50, IDLEPOLL=500, **args):
        Xps.deviceCount += 1
        # put the args into the self.* dictionary
        self.__dict__.update(locals())
        self.__dict__.update(**args)
        self.__super.__init__(name)

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


class xps_asyn_motor(basic_asyn_motor, Device):  
    ''' XPS axis - instatiate one per axis '''
    
    # Constructor, just stores the parameters
    def __init__(self, PORT, POSITIONER, STEPS=1000, **args):
        # put the args into the self.* dictionary
        self.__dict__.update(locals())
        self.__dict__.update(**args)
        # I have hijacked the PORT prameter from basic_asyn_motor.ArgInfo so that I
        # can extract the CARD number from it (see below)
        # hence I need to explicitly pass it to the superclass basic_asyn_motor init
        self.__super.__init__(PORT=PORT, **args)
        self.CARD = PORT.CARD
        
    # startup script - once per instatiation
    def Initialise(self):
        print 'XPSConfigAxis(%(CARD)d, %(ADDR)s, %(POSITIONER)s, %(STEPS)d)' % self.__dict__
        
    def PostIocInitialise(self):        
        #Fix to stop motor record sending a stop command at the end of a
        #move (this can happen if motor record detects that motor is going backward,
        #for example due to a PID loop correction). For the XPS motors in a group,
        #this can stop the whole group which is not the behaviour we want if
        #we are moving multiple axes.
        print 'dbpf("%(P)s%(M)s.NTM", "0")' % self.__dict__
        #Send a motor record stop/go to do a reset. This is sometimes needed at startup.
        print 'dbpf("%(P)s%(M)s.SPMG", "0")' % self.__dict__
        print 'dbpf("%(P)s%(M)s.SPMG", "3")' % self.__dict__
      
    # Arguments - this defines the arguments that will be passed to init, the makeArgInfo
    # part defines which parameters will appear in XEB for this builder class
    ArgInfo = basic_asyn_motor.ArgInfo + makeArgInfo(__init__,
        POSITIONER = Simple("XPS Positioner name of the form G1.P1", str),
        STEPS = Simple("Steps per user unit", int),
        PORT = Ident("Xps object", Xps) )    
    
