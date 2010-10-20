#!/bin/env python2.4

import thread, time

from pkg_resources import require
require('dls_simulationlib')
import dls_simulationlib.simsocket

# Returns true if the string is an integer with a possible '-' sign prefix
def isNumber(val):
    if len(val) == 0:
        return False
    elif val[0] == '-':
        return val[1:].isdigit()
    else:
        return val.isdigit()

class DriverModule(object):
    # This base class represents a driver module in the chain.
    def __init__(self, name):
        self.name = name.upper()
        self.currentPicomotor = None
        self.enabled = False

    # Returns the status of the driver
    def queryDeviceStatus(self):
        return '%s=0x%02x\r\n' % (self.name, self.currentPicomotor.getDeviceStatus())

    # Returns the device diagnostic
    def queryDeviceDiagnostics(self):
        return '%s=0x%02x\r\n' % (self.name, self.currentPicomotor.getDeviceDiagnostic())

    # Returns the current motor position
    def queryMotorPosition(self):
        return '%s=%d\r\n' % (self.name, self.currentPicomotor.getCurrentPosition())

    # Sets the motor position
    def setMotorPosition(self, pos):
        if isNumber(pos):
            self.currentPicomotor.setCurrentPosition(int(pos))
        return ''

    # Stops the motion of the current motor
    def stopMotion(self):
        self.currentPicomotor.stopMotion()
        return ''

    # Enables the driver
    def enableMotorDriver(self):
        self.enabled = True
        return ''

    # Disables the driver
    def disableMotorDriver(self):
        self.enabled = False
        return ''

    # Moves the motor relative
    def setRelativePos(self, pos):
        if isNumber(pos):
            self.currentPicomotor.moveRelative(int(pos))
        return ''

    # Moves the motor absolute
    def setAbsolutePos(self, pos):
        print '  setAbsolutePos: pos=%s' % repr(pos)
        if isNumber(pos):
            self.currentPicomotor.moveAbsolute(int(pos))
        return ''

    # Sets the motor moving
    def go(self):
        self.currentPicomotor.go()
        return ''

    # Jogs the motor forward
    def jogForward(self, vel=None):
        if vel is None:
            self.currentPicomotor.jogForward()
        elif isNumber(vel):
            self.currentPicomotor.jogForward(int(vel))
        return ''

    # Jogs the motor backward
    def jogBackward(self, vel=None):
        if vel is None:
            self.currentPicomotor.jogBackward()
        elif isNumber(vel):
            self.currentPicomotor.jogBackward(int(vel))
        return ''

    # Actually move the motor
    def move(self):
        self.currentPicomotor.move()

    # Search for the negative limit
    def searchForNegativeLimit(self):
        self.currentPicomotor.searchForNegativeLimit()
        return ''

class ClosedLoopDriverModule(DriverModule):
    # A specialisation that represents a closed loop driver module
    def __init__(self, name, lowerLimit, upperLimit):
        DriverModule.__init__(self, name)
        self.currentPicomotor = ClosedLoopPicomotor('0', lowerLimit, upperLimit)

    # Returns the minimum profile velocity
    def queryMinimumProfileVelocity(self, channel=None):
        return ''

    # Set the minimum profile velocity
    def setMinimumProfileVelocity(self, channel, value):
        return ''

    # Returns the driver type
    def queryDriverType(self):
        return '%s=2\r\n' % self.name

    # Returns the current motor channel
    def queryMotorChannel(self):
        return ''

    # Sets the current motor channel
    def setMotorChannel(self, channel):
        return ''

    # Returns the motor velocity
    def queryMotorVelocity(self, channel=None):
        result = '%s M0=%d\r\n' % (self.name,
            self.currentPicomotor.getVelocity())
        return result

    # Set the motor velocity
    def setMotorVelocity(self, channel, value):
        if value.isdigit():
            self.currentPicomotor.setVelocity(int(value))
        return ''

    # Returns the motor acceleration
    def queryMotorAcceleration(self, channel=None):
        result = '%s M0=%d\r\n' % (self.name,
            self.currentPicomotor.getAcceleration())
        return result

    # Set the motor acceleration
    def setMotorAcceleration(self, channel, value):
        if value.isdigit():
            self.currentPicomotor.setAcceleration(int(value))
        return ''

    # Enables closed loop mode
    def enableClosedLoopMode(self):
        self.currentPicomotor.enableClosedLoopMode()
        return ''

    # Disables closed loop mode
    def disableClosedLoopMode(self):
        self.currentPicomotor.disableClosedLoopMode()
        return ''

class OpenLoopDriverModule(DriverModule):
    # A specialisation that represents an open loop driver module
    def __init__(self, name):
        DriverModule.__init__(self, name)
        self.picomotors = {}
        self.picomotors['0'] = OpenLoopPicomotor('0')
        self.picomotors['1'] = OpenLoopPicomotor('1')
        self.picomotors['2'] = OpenLoopPicomotor('2')
        self.currentPicomotor = self.picomotors['0']

    # Returns the minimum profile velocity
    def queryMinimumProfileVelocity(self, channel=None):
        result = ''
        if channel is None:
            for name,motor in self.picomotors.iteritems():
                result += '%s M%s=%d\r\n' % (self.name, name,
                    motor.getMinimumProfileVelocity())
        elif channel in self.picomotors:
            result += '%s M%s=%d\r\n' % (self.name, channel, 
                self.picomotors[channel].getMinimumProfileVelocity())
        return result

    # Set the minimum profile velocity
    def setMinimumProfileVelocity(self, channel, value):
        if value.isdigit():
            if channel in self.picomotors:
                self.currentPicomotor = self.picomotors[channel]
                self.currentPicomotor.setMinimumProfileVelocity(int(value))
        return ''

    # Returns the driver type
    def queryDriverType(self):
        return '%s=1\r\n' % self.name

    # Returns the current motor channel
    def queryMotorChannel(self):
        return '%s=%s\r\n' % (self.name, self.currentPicomotor.name)

    # Sets the current motor channel
    def setMotorChannel(self, channel):
        if channel in self.picomotors:
            self.currentPicomotor = self.picomotors[channel]
        return ''

    # Returns the motor velocity
    def queryMotorVelocity(self, channel=None):
        result = ''
        if channel is None:
            for name,motor in self.picomotors.iteritems():
                result += '%s M%s=%d\r\n' % (self.name, name,
                    motor.getVelocity())
        elif channel in self.picomotors:
            result += '%s M%s=%d\r\n' % (self.name, channel, 
                self.picomotors[channel].getVelocity())
        return result

    # Set the motor velocity
    def setMotorVelocity(self, channel, value):
        if value.isdigit():
            if channel in self.picomotors:
                self.currentPicomotor = self.picomotors[channel]
                self.currentPicomotor.setVelocity(int(value))
        return ''

    # Returns the motor acceleration
    def queryMotorAcceleration(self, channel=None):
        result = ''
        if channel is None:
            for name,motor in self.picomotors.iteritems():
                result += '%s M%s=%d\r\n' % (self.name, name,
                    motor.getAcceleration())
        elif channel in self.picomotors:
            result += '%s M%s=%d\r\n' % (self.name, channel, 
                self.picomotors[channel].getAcceleration())
        return result

    # Set the motor acceleration
    def setMotorAcceleration(self, channel, value):
        if value.isdigit():
            if channel in self.picomotors:
                self.currentPicomotor = self.picomotors[channel]
                self.currentPicomotor.setAcceleration(int(value))
        return ''

    # Enables closed loop mode
    def enableClosedLoopMode(self):
        return ''

    # Disables closed loop mode
    def disableClosedLoopMode(self):
        return ''

class Picomotor(object):
    # This base class represents a single picomotor axis
    def __init__(self, name):
        self.wasMoving = False
        self.moving = False
        self.name = name
        self.currentPosition = 0
        self.commandPosition = 0
        self.velocity = 1000
        self.acceleration = 100
        self.jogForward = False
        self.jogBackward = False

    # Returns the current position
    def getCurrentPosition(self):
        return self.currentPosition

    # Stops the motion
    def stopMotion(self):
        self.moving = False

    # Starts a motion
    def go(self):
        self.moving = True

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

    # Jog the motor backward
    def jogBackward(self, vel=None):
        if vel is not None:
            self.velocity = vel
        self.jogForward = False
        self.jogBackward = True

    # Move the motor
    def move(self):
        # Does this motor need to move?
        self.wasMoving = self.moving
        if self.moving:
            # Which direction?
            movingForward = self.jogForward or \
                (self.commandPosition >= self.currentPosition and not self.jogBackward)
            # How much?
            delta = self.velocity * ControllerModule.servoPeriod
            if not movingForward:
                delta = -delta
            # Where do we end up?
            self.currentPosition = self.currentPosition + delta
            if not self.jogForward and not self.jogBackward:
                if movingForward and self.currentPosition > self.commandPosition:
                    self.currentPosition = self.commandPosition
                    self.moving = False
                elif not movingForward and self.currentPosition < self.commandPosition:
                    self.currentPosition = self.commandPosition
                    self.moving = False

class ClosedLoopPicomotor(Picomotor):
    # A specialisation that represents a closed loop axis
    def __init__(self, name, lowerLimit, upperLimit):
        Picomotor.__init__(self, name)
        self.onLowerLimit = False
        self.onUpperLimit = False
        self.lowerLimit = lowerLimit
        self.upperLimit = upperLimit
        self.homing = False
        self.servoOn = False
        self.searchSequence = []
        self.search = None

    # Move the motor
    def move(self):
        # Base class does most of the work
        Picomotor.move(self)
        if self.wasMoving:
            print '   moved: pos=%s cmd=%s llim=%s ulim=%s' % (self.currentPosition,
                self.commandPosition, self.lowerLimit, self.upperLimit)
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
        if not self.moving and len(self.searchSequence) > 0:
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

    # Search for the negative limit
    def searchForNegativeLimit(self):
        self.searchSequence = ['lowLimitOff', 'lowLimitOn', 'lowLimitOff']
        self.handleSearchSequence()

    # Sets the current position
    def setCurrentPosition(self, pos):
        self.currentPosition = pos

    # Returns the status of the motor
    def getDeviceStatus(self):
        stat = 0
        if not self.moving:
            stat |= 0x01
        stat |= 0x08
        if self.onLowerLimit:
            stat |= 0x20
        if self.onUpperLimit:
            stat |= 0x40
        if self.homing:
            stat |= 0x80
        return stat

    # Returns the device diagnostic
    def getDeviceDiagnostic(self):
        stat = 0
        if self.servoOn:
            stat |= 0x04
        return stat

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
        print '  move absolute: servoOn=%s, pos=%s, cmd=%s' % (self.servoOn, pos, self.commandPosition)
        self.jogForward = False
        self.jogBackward = False

    # Enables closed loop mode
    def enableClosedLoopMode(self):
        self.servoOn = True

    # Disables closed loop mode
    def disableClosedLoopMode(self):
        self.servoOn = False

class OpenLoopPicomotor(Picomotor):
    # A specialisation that represents an open loop axis
    def __init__(self, name):
        Picomotor.__init__(self, name)
        self.minimumProfileVelocity = 8

    # Sets the current position
    def setCurrentPosition(self, pos):
        pass

    # Returns the status of the motor
    def getDeviceStatus(self):
        stat = 0
        if self.moving:
            stat |= 0x01
        stat |= 0x04
        return stat

    # Returns the device diagnostic
    def getDeviceDiagnostic(self):
        return 0

    # Returns the minimum profile velocity
    def getMinimumProfileVelocity(self):
        return self.minimumProfileVelocity

    # Sets the minimum profile velocity
    def setMinimumProfileVelocity(self, value):
        self.minimumProfileVelocity = value

    # Moves the motor relative
    def moveRelative(self, pos):
        self.commandPosition += pos
        self.jogForward = False
        self.jogBackward = False

    # Moves the motor absolute
    def moveAbsolute(self, pos):
        self.commandPosition += pos
        self.jogForward = False
        self.jogBackward = False

    # Search for the negative limit
    def searchForNegativeLimit(self):
        pass

class ControllerModule(object):
    # This class represents the controller module
    servoPeriod = 0.1    # seconds

    def __init__(self, tcpPort):
        self.drivers = {}
        self.joystickOn = True
        self.tcpServer = TcpUserServer(self, tcpPort)
        self.moveLoopTaskId = thread.start_new_thread(self.moveAllMotors, ())

    # Move all the motors
    def moveAllMotors(self):
        while True:
            time.sleep(ControllerModule.servoPeriod)
            for name,driver in self.drivers.iteritems():
                driver.move()

    # Add a driver module to the chain
    def addDriverModule(self, module):
        self.drivers[module.name] = module

    # Returns the driver with the given name.  The 'a' prefix
    # on the name is optional.
    def getDriver(self, name):
        upperName = name.upper()
        if upperName in self.drivers:
            return self.drivers[upperName]
        elif 'A'+upperName in self.drivers:
            return self.drivers['A'+upperName]
        else:
            return None

    # Handle a command
    def receive(self, command):
        reply = ''
        (keyword, params) = self.parseCmd(command.lower())
        if keyword is None:
            pass    # Nothing on the line
        elif keyword == 'ver':
            reply = 'Version 1.5.0\r\n'
        elif keyword == 'sta':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryDeviceStatus()
            elif len(params) == 0:
                names = self.drivers.keys()
                names.sort()
                for name in names:
                    reply += self.drivers[name].queryDeviceStatus()
        elif keyword == 'diag':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryDeviceDiagnostics()
        elif keyword == 'pos':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMotorPosition()
            elif len(params) == 2:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.setMotorPosition(params[1])
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.queryMotorPosition()
        elif keyword == 'sto':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.stopMotion()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.stopMotion()
        elif keyword == 'mon':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.enableMotorDriver()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.enableMotorDriver()
        elif keyword == 'mof':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.disableMotorDriver()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.disableMotorDriver()
        elif keyword == 'jon':
            if len(params) == 0:
                self.joystickOn = True
        elif keyword == 'jof':
            if len(params) == 0:
                self.joystickOn = False
        elif keyword == 'mpv':
            if len(params) == 2:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMinimumProfileVelocity(params[1])
            elif len(params) == 3:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.setMinimumProfileVelocity(params[1], params[2])
            elif len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMinimumProfileVelocity()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.queryMinimumProfileVelocity()
        elif keyword == 'drt':
            if len(params) == 0:
                names = self.drivers.keys()
                names.sort()
                for name in names:
                    reply += self.drivers[name].queryDriverType()
        elif keyword == 'rel':
            if len(params) in [2,3]:
                driver = self.getDriver(params[0])
                if driver is not None:
                    driver.setRelativePos(params[1])
                    if len(params) == 3 and param[2] == 'g':
                        driver.go()
        elif keyword == 'abs':
            print '  abs: params=%s' % params
            if len(params) in [2,3]:
                driver = self.getDriver(params[0])
                if driver is not None:
                    driver.setAbsolutePos(params[1])
                    if len(params) == 3 and param[2] == 'g':
                        driver.go()
        elif keyword == 'chl':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMotorChannel()
            elif len(params) == 2:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.setMotorChannel(params[1])
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.queryMotorChannel()
        elif keyword == 'vel':
            if len(params) == 2:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMotorVelocity(params[1])
            elif len(params) == 3:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.setMotorVelocity(params[1], params[2])
            elif len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMotorVelocity()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.queryMotorVelocity()
        elif keyword == 'acc':
            if len(params) == 2:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMotorAcceleration(params[1])
            elif len(params) == 3:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.setMotorAcceleration(params[1], params[2])
            elif len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.queryMotorAcceleration()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.queryMotorAcceleration()
        elif keyword == 'go':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.go()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.go()
        elif keyword == 'hal':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.stopMotion()
            elif len(params) == 0:
                for name,driver in self.drivers.iteritems():
                    reply += driver.stopMotion()
        elif keyword == 'for':
            if len(params) in [1,2,3]:
                driver = self.getDriver(params[0])
                if driver is not None:
                    vel = None
                    if len(params) in [2,3]:
                        vel = params[1]
                    reply = driver.jogForward(vel)
                    if len(params) == 3 and params[2] == 'g':
                        driver.go()
        elif keyword == 'rev':
            if len(params) in [1,2,3]:
                driver = self.getDriver(params[0])
                if driver is not None:
                    vel = None
                    if len(params) in [2,3]:
                        vel = params[1]
                    reply = driver.jogBackward(vel)
                    if len(params) == 3 and params[2] == 'g':
                        driver.go()
        elif keyword == 'ser':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.enableClosedLoopMode()
        elif keyword == 'nos':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.disableClosedLoopMode()
        elif keyword == 'rli':
            if len(params) == 1:
                driver = self.getDriver(params[0])
                if driver is not None:
                    reply = driver.searchForNegativeLimit()
        else:
            print 'Command not supported: %s' % repr(command)
        return reply

    # Parses the command into a tuple consisting of a keyword and a parameter list
    def parseCmd(self, cmd):
        parts = []
        word = None
        for ch in cmd:
            if ch.isspace():
                if word is not None:
                    parts.append(word)
                    word = None
            elif len(parts) >= 1 and ch == '=':
                if word is not None:
                    parts.append(word)
                    word = None
            else:
                if word is not None:
                    word += ch
                else:
                    word = ch
        if word is not None:
            parts.append(word)
            word = None
        if len(parts) > 0:
            return (parts[0], parts[1:])
        else:
            return (None, None)

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
        eolPos = self.tcpInputData.find('\r')
        if eolPos >= 0:
            command = self.tcpInputData[:eolPos]
            self.tcpInputData = self.tcpInputData[eolPos+1:]
            reply = self.controller.receive(command)
            print '> %s' % repr(reply + '\r\n>')
            self.transmit(reply + '\r\n>')

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

