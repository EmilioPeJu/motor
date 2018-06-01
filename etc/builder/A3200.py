from iocbuilder import Device, AutoSubstitution
from iocbuilder.modules.asyn import Asyn, AsynPort
from common import MotorLib, basic_asyn_motor
from iocbuilder.arginfo import *

__all__ = ['Aerotech_A3200_controller']

class A3200Asyn(Device):
    AutoInstantiate = True
    Dependencies = (Asyn, MotorLib)
    LibFileList=["Aerotech"]
    DbdFileList=["devAerotech"]

# Extra axis records
class A3200_axis_extra(AutoSubstitution):
    AutoInstantiate = True
    TemplateFile = "A3200_axis_extra.db"
        
class Aerotech_A3200_controller(AsynPort):
    """Aerotech A3200 controller. Connect to an AsynIP port to control the A3200 over Ethernet. ASCII command interface must be enabled on the A3200 and firewall configured."""
    Dependencies=(A3200Asyn,)
    
    # Count how many instances of this class we will have created
    # because we have to print it in the startup script
    instance_counter = 0
    
    def __init__(self, name, P, M, controllerNum, port, address, numAxes, movingPollingRate=100, idlePollingRate=500):
        self.name = name
        self.prefix = P
        self.suffix = M
        self.numAxes = numAxes
        self.__super.__init__(name)
        self.__dict__.update(locals())
        
        # Increment the counter of instances
        Aerotech_A3200_controller.instance_counter += 1

        # Instantiate extra records for Axes
        for axisNum in range(self.numAxes):
            A3200_axis_extra(P=self.prefix, M=self.suffix, N=axisNum, PORT=self.name, ADDR=axisNum)
    
    def Initialise(self):
        # Print the controller init line in the startup script

        print "# Aerotech A3200 asyn motor driver configure parameters."
        print "#     (1) Controller number being configured"
        print "#     (2) ASYN port name"
        print "#     (3) ASYN address (GPIB only)"
        print "#     (4) Number of axes to control"
        print "#     (5) Time to poll (msec) when an axis is in motion"
        print "#     (6) Time to poll (msec) when an axis is idle. 0 for no polling"
        print 'A3200AsynConfig(%(controllerNum)d, "%(port)s", %(address)d, %(numAxes)d, %(movingPollingRate)d, %(idlePollingRate)d)' % self.__dict__
        
        print ""
        print "# Asyn-based Motor Record support"
        print "#   (1) Asyn port"
        print "#   (2) Driver name"
        print "#   (3) Controller index"
        print "#   (4) Max. number of axes"
        print "drvAsynMotorConfigure(\"%(name)s\",\"motorA3200\", %(address)d, %(numAxes)d)" % self.__dict__
        
        print ""
        print "# Aerotech A3200 asyn motor driver custom parameters"
        print 'A3200Interpose("%(name)s")' % self.__dict__

    def InitialiseOnce(self):
        # Print the class init line in teh startup script
        number_of_controllers = Aerotech_A3200_controller.instance_counter
        
        print "# Aerotech A3200"
        print "# Number of controllers in system:"
        print "A3200AsynSetup(%d)" % number_of_controllers
        print ""

    ArgInfo = makeArgInfo(__init__,
        name = Simple("Device Name"),
        P = Simple("Device Prefix"),
        M = Simple("Device Suffix"),
        controllerNum = Simple("Controller number in system, zero-based", int),
        port = Ident("Asyn port to connect to", AsynPort),
        address = Simple("ASYN address (GPIB only", int),
        numAxes = Simple("Number of axes to control", int),
        movingPollingRate = Simple("Time to poll (msec) when an axis is in motion", int),
        idlePollingRate = Simple("Time to poll (msec) when an axis is idle. 0 for no polling", int))
