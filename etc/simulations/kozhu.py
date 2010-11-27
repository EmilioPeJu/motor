#!/bin/env python2.6

# A simulation of the Kohzu series of motor controllers.
# Jonathan Thompson, October 2010
#
# When used as a library for the creation of complex simulations,
# the exported class is:
#
# Controller(port, modelNumber)
#    Creates an instance of a controller of the specified model number.  The
#    model number should be a string, one of ['200', '210', '400', '410', '800'].
#

import thread, time

from pkg_resources import require
require('dls_simulationlib')
import dls_simulationlib.simsocket
from dls_simulationlib import simui

def isNumber(val):
    if len(val) == 0:
        return False
    elif val[0] == '-':
        return val[1:].isdigit()
    else:
        return val.isdigit()

class Axis(object):
    # This class represents an axis.
    def __init__(self, number, lowerLimit=0, upperLimit=1000):
        self.number = number
        self.wasMoving = False
        self.moving = False
        self.currentPosition = 0
        self.commandPosition = 0
        self.velocity = 1000
        self.acceleration = 100
        self.jogForward = False
        self.jogBackward = False
        self.onLowerLimit = False
        self.onUpperLimit = False
        self.lowerLimit = lowerLimit
        self.upperLimit = upperLimit
        self.homing = False
        self.servoOn = True
        self.searchSequence = []
        self.search = None
        self.homed = False

    def setLimits(self, lowerLimit, upperLimit):
        self.lowerLimit = lowerLimit
        self.upperLimit = upperLimit

    def getSysInfo(self, sysNum):
        result = 0
        if sysNum == 21:
            if self.servoOn:
                result = 0
            else:
                result = 1
        else:
            print 'Unsupported sysNum: %s' % sysNum
        return result

    def getPosition(self):
        return self.currentPosition

    def getDrivingOperation(self):
        if self.moving:
            return 1
        else:
            return 0

    def getNorgSignal(self):
        return 0

    def getOrgSignal(self):
        if self.homed:
            return 1
        else:
            return 0

    def getCwLimitSignal(self):
        if self.onUpperLimit:
            return 1
        else:
            return 0

    def getCcwLimitSignal(self):
        if self.onLowerLimit:
            return 1
        else:
            return 0

    def getError(self):
        return 0

    # Returns the current position
    def getCurrentPosition(self):
        return self.currentPosition

    # Stops the motion
    def stopMotion(self):
        self.moving = False

    # Sets the velocity
    def setVelocity(self, vel):
        self.velocity = vel

    # Returns the velocity
    def getVelocity(self):
        return self.velocity

    # Sets the acceleration
    def setAcceleration(self, acc):
        self.acceleration = acc

    # Returns the acceleration
    def getAcceleration(self):
        return self.acceleration

    # Jog the motor forward
    def jogForward(self, vel=None):
        if vel is not None:
            self.velocity = vel
        self.jogForward = True
        self.jogBackward = False
        self.moving = True

    # Jog the motor backward
    def jogBackward(self, vel=None):
        if vel is not None:
            self.velocity = vel
        self.jogForward = False
        self.jogBackward = True
        self.moving = True

    # Move the motor
    def move(self):
        # Does this motor need to move?
        if self.moving:
            # Which direction?
            movingForward = self.jogForward or \
                (self.commandPosition >= self.currentPosition and \
                not self.jogBackward)
            # How much?
            delta = self.velocity * Controller.servoPeriod
            if not movingForward:
                delta = -delta
            # Where do we end up?
            self.currentPosition = self.currentPosition + delta
            if not self.jogForward and not self.jogBackward:
                if movingForward and \
                        self.currentPosition > self.commandPosition:
                    self.currentPosition = self.commandPosition
                    self.moving = False
                elif not movingForward and \
                        self.currentPosition < self.commandPosition:
                    self.currentPosition = self.commandPosition
                    self.moving = False
            print '   moved: pos=%s cmd=%s llim=%s ulim=%s' %\
                (self.currentPosition, self.commandPosition,
                self.lowerLimit, self.upperLimit)
            # Check the limits
            self.onLowerLimit = False
            self.onUpperLimit = False
            if self.currentPosition >= self.upperLimit:
                self.currentPosition = self.upperLimit
                self.onUpperLimit = True
                self.moving = False
            elif self.currentPosition <= self.lowerLimit:
                self.currentPosition = self.lowerLimit
                self.onLowerLimit = True
                self.moving = False
            # Has a search completed?
            if self.search is None:
                pass
            elif self.search == 'lowLimitOff' and not self.onLowerLimit:
                self.jogForward = False
                self.jogBackward = False
                self.moving = False
                self.search = None
                self.handleSearchSequence()
            elif self.search == 'lowLimitOn' and self.onLowerLimit:
                self.jogForward = False
                self.jogBackward = False
                self.moving = False
                self.search = None
                self.handleSearchSequence()

    def handleSearchSequence(self):
        # Determine next search function
        if not self.moving:
            if len(self.searchSequence) > 0:
                self.search = self.searchSequence[0]
                self.searchSequence = self.searchSequence[1:]
                print '    Search: %s' % self.search
                if self.search == 'lowLimitOff':
                    self.jogForward = True
                    self.moving = True
                elif self.search == 'lowLimitOn':
                    self.jogBackward = True
                    self.moving = True
                else:
                    self.search = None
                    self.handleSearchSequence()
            if self.search is None and self.homing:
                self.homed = True
                self.homing = False
                print 'Home complete'

    # Search for the negative limit
    def searchForNegativeLimit(self):
        self.homing = True
        self.homed = False
        self.searchSequence = ['lowLimitOff', 'lowLimitOn', 'lowLimitOff']
        self.handleSearchSequence()

    # Sets the current position
    def setCurrentPosition(self, pos):
        self.currentPosition = pos

    # Moves the motor relative
    def moveRelative(self, pos):
        if self.servoOn:
            self.commandPosition += pos
        else:
            if pos > 255:
                pos = 255
            elif pos < -255:
                pos = -255
            self.commandPosition += pos * 85 / 1000
        self.jogForward = False
        self.jogBackward = False
        self.moving = True

    # Moves the motor absolute
    def moveAbsolute(self, pos):
        if self.servoOn:
            self.commandPosition = pos
        else:
            if pos > 255:
                pos = 255
            elif pos < -255:
                pos = -255
            self.commandPosition += pos * 85 / 1000
        print '  move absolute: servoOn=%s, pos=%s, cmd=%s' % \
            (self.servoOn, pos, self.commandPosition)
        self.jogForward = False
        self.jogBackward = False
        self.moving = True

    # Enables closed loop mode
    def enableClosedLoopMode(self):
        self.servoOn = True

    # Disables closed loop mode
    def disableClosedLoopMode(self):
        self.servoOn = False

class Controller(object):
    # This class represents the controller
    servoPeriod = 0.1    # seconds

    def __init__(self, tcpPort, modelNumber):
        self.modelNumber = modelNumber
        numAxes = 4
        if modelNumber in ['200', '210']:
            numAxes = 2
        if modelNumber in ['400', '410']:
            numAxes = 4
        if modelNumber in ['800']:
            numAxes = 8
        self.axes = {}
        self.tcpServer = TcpUserServer(self, tcpPort)
        self.moveLoopTaskId = thread.start_new_thread(self.moveAllMotors, ())
        for i in range(numAxes):
            self.axes[i+1] = Axis(i)

    def setAxisLimits(self, axisNum, lowerLimit, upperLimit):
        self.axes[axisNum].setLimits(lowerLimit, upperLimit)

    # Move all the motors
    def moveAllMotors(self):
        while True:
            time.sleep(Controller.servoPeriod)
            for number,axis in self.axes.iteritems():
                axis.move()

    # Returns the driver with the given name.  The 'a' prefix
    # on the name is optional.
    def getAxis(self, number):
        result = None
        if number in self.axes:
            result = self.axes[number]
        return result

    # Handle a command
    def receive(self, command):
        reply = ''
        error = 'Command not supported'
        (keyword, params) = self.parseCmd(command.lower())
        #print '%s: %s' % (keyword, params)
        if keyword is None:
            error = None    # Nothing on the line
        elif keyword == 'IDN' and len(params) == 0:
            (error, reply) = self.handleIdentificationCmd(params)
        elif keyword == 'STR' and len(params) == 2:
            (error, reply) = self.handleStatusReadCmd(params)
        elif keyword == 'RDP' and len(params) == 2:
            (error, reply) = self.handlePositionReadCmd(params)
        elif keyword == 'RSY' and len(params) == 2:
            (error, reply) = self.handleSystemReadCmd(params)
        elif keyword == 'ASI' and len(params) == 14:
            (error, reply) = self.handleMotorInitialSettingsCmd(params)
        elif keyword == 'APS' and len(params) == 8:
            (error, reply) = self.handleAbsolutePositionDriveCmd(params)
        elif keyword == 'ORG' and len(params) == 6:
            (error, reply) = self.handleOriginDriveCmd(params)
        elif keyword == 'STP' and len(params) == 2:
            (error, reply) = self.handleStopCmd(params)
        if error is not None:
            print '%s: %s' % (error, repr(command))
        return reply

    def handleStopCmd(self, params):
        error = None
        reply = ''
        if isNumber(params[0]):
            axisNum = int(params[0])
            if axisNum in self.axes:
                axis = self.axes[axisNum]
                axis.stopMotion()
                reply = 'C\tSTP%s\r\n' % axisNum
            else:
                error = 'Illegal axis'
        else:
            error = 'Parameter not a number'
        return (error, reply)

    def handleOriginDriveCmd(self, params):
        error = None
        reply = ''
        if isNumber(params[0]):
            axisNum = int(params[0])
            if axisNum in self.axes:
                axis = self.axes[axisNum]
                axis.searchForNegativeLimit()
                reply = 'C\tORG%s\r\n' % axisNum
            else:
                error = 'Illegal axis'
        else:
            error = 'Parameter not a number'
        return (error, reply)

    def handleAbsolutePositionDriveCmd(self, params):
        error = None
        reply = ''
        if isNumber(params[0]) and isNumber(params[4]):
            axisNum = int(params[0])
            position = int(params[4])
            if axisNum in self.axes:
                axis = self.axes[axisNum]
                axis.moveAbsolute(position)
                reply = 'C\tAPS%s\r\n' % axisNum
            else:
                error = 'Illegal axis'
        else:
            error = 'Parameter not a number'
        return (error, reply)

    def handleMotorInitialSettingsCmd(self, params):
        # Currently ignores all parameters
        error = None
        reply = ''
        if isNumber(params[0]) and isNumber(params[2]):
            axisNum = int(params[0])
            velocity = int(params[2])
            if axisNum in self.axes:
                axis = self.axes[axisNum]
                # EPICS appears to send a 1 for the homing velocity
                # we'll bodge it for now and see what the real
                # hardware does.
                if velocity > 1:
                    axis.setVelocity(velocity)
                reply = 'C\tASI%s\r\n' % axisNum
            else:
                error = 'Illegal axis'
        else:
            error = 'Parameter not a number'
        return (error, reply)

    def handleSystemReadCmd(self, params):
        error = None
        reply = ''
        if isNumber(params[0]) and isNumber(params[1]):
            axisNum = int(params[0])
            sysNum = int(params[1])
            if axisNum in self.axes:
                axis = self.axes[axisNum]
                reply = 'C\tRSY%s\t%s\t%s\r\n' % (axisNum, sysNum, 
                    axis.getSysInfo(sysNum))
            else:
                error = 'Illegal axis'
        else:
            error = 'Parameter not a number'
        return (error, reply)

    def handlePositionReadCmd(self, params):
        error = None
        reply = ''
        if isNumber(params[0]) and isNumber(params[1]):
            axisNum = int(params[0])
            mode = int(params[1])
            if axisNum in self.axes:
                axis = self.axes[axisNum]
                reply = 'C\tRDP%s\t%s\r\n' % (axisNum, axis.getPosition())
            else:
                error = 'Illegal axis'
        else:
            error = 'Parameter not a number'
        return (error, reply)

    def handleIdentificationCmd(self, params):
        error = None
        reply = 'C\tIDN0\t%s\t1000\r\n' % self.modelNumber
        return (error, reply)

    def handleStatusReadCmd(self, params):
        error = None
        reply = ''
        if isNumber(params[0]) and isNumber(params[1]):
            axisNum = int(params[1])
            mode = int(params[0])
            if axisNum in self.axes:
                axis = self.axes[axisNum]
                reply = 'C\tSTR%s\t%s\t%s\t%s\t%s\t%s\t%s\t0\t%s\r\n' % \
                    (axisNum, mode, axis.getDrivingOperation(),
                    axis.getNorgSignal(), axis.getOrgSignal(),
                    axis.getCwLimitSignal(), axis.getCcwLimitSignal(),
                    axis.getError())
            else:
                error = 'Illegal axis'
        else:
            error = 'Parameter not a number'
        return (error, reply)

    # Parses the command into a tuple consisting of a keyword and
    # a parameter list
    def parseCmd(self, cmd):
        keyword = None
        parameters = None
        if len(cmd) >= 5 and cmd[0] == '\x02' and cmd[-1] == '\r':
            keyword = cmd[1:4].upper()
            parameters = cmd[4:-1].upper().split('/')
            if len(parameters) == 1 and len(parameters[0]) == 0:
                parameters = []
        return (keyword, parameters)

class TcpUser(dls_simulationlib.simsocket.dataSocket):
    # This class handles a single TCP user
    def __init__(self, clientSocket, controller):
        dls_simulationlib.simsocket.dataSocket.__init__(self, clientSocket)
        self.tcpInputData = ''
        self.tcpOutputData = ''
        self.controller = controller

    # Override to receive data from the socket
    def receive(self, data):
        print '< %s' % repr(data)
        self.tcpInputData += data
        eolPos = self.tcpInputData.find('\n')
        if eolPos >= 0:
            command = self.tcpInputData[:eolPos]
            self.tcpInputData = self.tcpInputData[eolPos+1:]
            reply = self.controller.receive(command)
            print '> %s' % repr(reply)
            self.transmit(reply)

class TcpUserServer(dls_simulationlib.simsocket.listenSocket):
    # Listens for TCP connections and makes users as appropriate
    def __init__(self, controller, tcpPort):
        dls_simulationlib.simsocket.listenSocket.__init__(self)
        self.controller = controller
        self.tcpPort = tcpPort
        self.createTcpServer(self.tcpPort)

    # Override to create a client handler object
    def createClient(self, clientSocket):
        print 'Connecting user on port %s' % self.tcpPort
        return TcpUser(clientSocket, self.controller)

class Kohzu(object):

    def __init__(self):
        # User interface
        self.ui = simui.UserInterface(self, name='Kohzu Simulation', gui=False)
        self.ui.run()

    def createSim(self):
        # Reduce thread stack size to be kind
        thread.stack_size(128*1024)
        # Kohzu controller
        self.kohzu1 = Controller(9001, '410')
        self.kohzu1.setAxisLimits(1, 0, 100000)
        self.kohzu1.setAxisLimits(2, 0, 100000)
        self.kohzu1.setAxisLimits(3, 0, 100000)
        self.kohzu1.setAxisLimits(4, 0, 100000)

if __name__=="__main__":
    Kohzu()

