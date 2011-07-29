#!/dls_sw/tools/bin/python2.4

from pkg_resources import require
require('dls_autotestframework')
from dls_autotestframework import TestCase

require('cothread') 

import re
import cothread

from XPS_C8_drivers import XPS

class motorCaseBase(TestCase):
   """
   Base class for all motor test cases.
   This is where the base PV is hardcoded to be 'mp49t:xps'
   """
   
   def __init__(self, A):
      TestCase.__init__(self, A)
      self.__pv1 = "mp49t:xps"
      self.__motors = [":motor1",":motor2"]
      self.__diag = 1
      self.__timeout = 10000
      self.__xpsHostname = "172.23.243.157"
      self.__xpsPort = 5001
      self.__xpsSocketTimeout = 2.0
      self.__xpsGroupName = "M"
      
   def getPVBase(self):
      return self.__pv1

   def getMotors(self):
      return self.__motors

   def getDiag(self):
      return self.__diag

   def getTimeout(self):
      return self.__timeout

   def getXPSHostname(self):
      return self.__xpsHostname

   def getXPSPort(self):
      return self.__xpsPort

   def getXPSSocketTimeout(self):
      return self.__xpsSocketTimeout

   def getXPSGroupName(self):
      return self.__xpsGroupName


   def doMoveSequence(self, distance, moves, axes=2):
      """
      Do a series of moves on each motor record. The number of moves
      and the distance are arguments to the function. The number of axes
      is also specified (max 6).

      The moves are triggered by writing to VAL.
      The start and end positions are checked (to be within RDBD)
      """

      move = 0.0
      axis_count = 0

      for motor in self.getMotors():

         axis_count = axis_count + 1
         if axis_count <= axes:

            self.diagnostic("Moving motor record " + self.getPVBase() + motor, self.getDiag());

            pv_rbv = self.getPVBase() + motor + ".RBV"
            pv_rdbd = self.getPVBase() + motor + ".RDBD"
            pv_val = self.getPVBase() + motor + ".VAL"

            rdbd = self.getPv(pv_rdbd)
            self.diagnostic(pv_rdbd + ": " + str(rdbd),  self.getDiag())
            
            for i in range(moves):
               
               move += distance
               
               self.diagnostic("Moving to " + str(move), self.getDiag())
               
               #Read RBV
               rbv = self.getPv(pv_rbv)
               self.diagnostic(pv_rbv + ": " + str(self.getPv(pv_rbv)), self.getDiag()+1)
               #Do a move
               self.putPv(pv_val, move, wait=True, timeout=self.getTimeout())
               #Read back RBV
               self.diagnostic(pv_rbv + ": " + str(self.getPv(pv_rbv)), self.getDiag()+1)

               #Verify RBV is within range
               self.verifyPvInRange(pv_rbv, move-rdbd, move+rdbd)

            #Now move back to zero
            move = 0.0
            self.putPv(pv_val, move, wait=True, timeout=self.getTimeout())
            self.verifyPvInRange(pv_rbv, move-rdbd, move+rdbd)


         

class motorCaseReadInit(motorCaseBase):
   """
   Test case to read motor record initial startup.
   It reads DMOV, MOVN, DVAL, OFF and MSTA.
   DVAL and OFF should have been set by autosave, so I want to move
   all to zero to start from a well defined state.
   """
   
   def runTest(self):

      init_dmov = 1
      init_movn = 0
      init_set = 0
      init_msta = 2 #Only check the 'done' bit.

      for motor in self.getMotors():

         self.diagnostic("Reading motor record " + self.getPVBase() + motor + " initial state", self.getDiag())
      
         pv_dmov = self.getPVBase() + motor + ".DMOV"
         pv_movn = self.getPVBase() + motor + ".MOVN"
         pv_set = self.getPVBase() + motor + ".SET"
         pv_val = self.getPVBase() + motor + ".VAL"
         pv_off = self.getPVBase() + motor + ".OFF"
         pv_msta = self.getPVBase() + motor + ".MSTA"

         self.diagnostic(pv_dmov + ": " + str(self.getPv(pv_dmov)), self.getDiag())
         self.diagnostic(pv_movn + ": " + str(self.getPv(pv_movn)), self.getDiag())
         self.diagnostic(pv_set + ": " + str(self.getPv(pv_set)), self.getDiag())
         self.diagnostic(pv_msta + ": " + str(self.getPv(pv_msta)), self.getDiag())
         
         self.verify(init_dmov, self.getPv(pv_dmov))
         self.verify(init_movn, self.getPv(pv_movn))
         self.verify(init_set, self.getPv(pv_set))

         self.diagnostic("Check that MSTA is not zero and at least in position is set...", self.getDiag())
         self.verify(init_msta, (0x2 & int(self.getPv(pv_msta))))

         #Move to zero, and set offsets to zero.
         self.putPv(pv_off, 0.0, wait=True, timeout=self.getTimeout())
         self.putPv(pv_val, 0.0, wait=True, timeout=self.getTimeout())
         

class motorCaseAutosaveRestoreCheck(motorCaseBase):
   """
   Class to check autosave restore. This means bringing down the IOC
   and restarting it, checking the restored positions. Do this twice to
   check the case when positions were not zero (which should not cause
   positions to be written on startup).
   """

   def runTest(self):

      offset = 1.0
      pos = 5.0

      for motor in self.getMotors():

         pv_val = self.getPVBase() + motor + ".VAL"
         pv_off = self.getPVBase() + motor + ".OFF"

         #First to a none zero position
         self.putPv(pv_off, offset, wait=True, timeout=self.getTimeout())
         self.putPv(pv_val, pos, wait=True, timeout=self.getTimeout())

      #Wait 1 minute for autosave to save the new numbers
      cothread.Sleep(60)

      #Now stop the IOC
      ioc = self.entity("ioc")
      self.diagnostic("Stopping IOC...", self.getDiag())
      ioc.stop()
      cothread.Sleep(10)
      #Start it again
      ioc.start()
      cothread.Sleep(10)

      for motor in self.getMotors():

         pv_val = self.getPVBase() + motor + ".VAL"
         pv_off = self.getPVBase() + motor + ".OFF"
         pv_rbv = self.getPVBase() + motor + ".RBV"
         pv_rdbd = self.getPVBase() + motor + ".RDBD"

         rdbd = self.getPv(pv_rdbd)

         #Verify RBV and OFF
         self.verifyPvInRange(pv_rbv, pos-rdbd, pos+rdbd)
         self.verifyPvInRange(pv_val, pos-rdbd, pos+rdbd)
         self.verify(offset, self.getPv(pv_off))

         #Now try from zero
         #First to a none zero position
         self.putPv(pv_off, 0, wait=True, timeout=self.getTimeout())
         self.putPv(pv_val, 0, wait=True, timeout=self.getTimeout())

      #Wait 1 minute for autosave to save the new numbers
      cothread.Sleep(60)

      #Now stop the IOC
      ioc = self.entity("ioc")
      self.diagnostic("Stopping IOC...", self.getDiag())
      ioc.stop()
      cothread.Sleep(10)
      #Start it again
      ioc.start()
      cothread.Sleep(10)
      
      for motor in self.getMotors():

         pv_val = self.getPVBase() + motor + ".VAL"
         pv_off = self.getPVBase() + motor + ".OFF"
         pv_rbv = self.getPVBase() + motor + ".RBV"
         pv_rdbd = self.getPVBase() + motor + ".RDBD"

         rdbd = self.getPv(pv_rdbd)

         #Verify RBV, VAL and OFF
         self.verifyPvInRange(pv_rbv, 0-rdbd, 0+rdbd)
         self.verifyPvInRange(pv_val, 0-rdbd, 0+rdbd)
         self.verify(0, self.getPv(pv_off))
         
         
         
   


class motorCaseMoveSequence1(motorCaseBase):
   """
   Class to do a series of moves on each motor record.
   """

   def runTest(self):

      distances = [0.001, 0.002, 0.003]
      moves = 10

      for distance in distances:
         self.diagnostic("Moving distances of " + str(distance), self.getDiag())
         self.doMoveSequence(distance, moves)


class motorCaseMoveSequence2(motorCaseBase):
   """
   Class to do a series of moves on each motor record.
   """

   def runTest(self):

      distances = [0.01, 0.02]
      moves = 10

      for distance in distances:
         self.diagnostic("Moving distances of " + str(distance), self.getDiag())
         self.doMoveSequence(distance, moves)


class motorCaseMoveSequence3(motorCaseBase):
   """
   Class to do a series of moves on each motor record.
   """

   def runTest(self):

      distances = [0.1, 0.2]
      moves = 10

      for distance in distances:
         self.diagnostic("Moving distances of " + str(distance), self.getDiag())
         self.doMoveSequence(distance, moves)


class motorCaseMoveSequence4(motorCaseBase):
   """
   Class to do a series of moves on each motor record.
   """

   def runTest(self):

      distances = [1.0]
      moves = 5

      for distance in distances:
         self.diagnostic("Moving distances of " + str(distance), self.getDiag())
         self.doMoveSequence(distance, moves)



class motorCaseMoveSequence5(motorCaseBase):
   """
   Class to do a series of moves on each motor record.
   """

   def runTest(self):

      distances = [1.230, 1.567, 2.810]
      moves = 5

      for distance in distances:
         self.diagnostic("Moving distances of " + str(distance), self.getDiag())
         self.doMoveSequence(distance, moves)


class motorCaseMoveCheckStatus(motorCaseBase):
   """
   Class to check MSTA during a move
   """

   def runTest(self):

      for motor in self.getMotors():

         self.diagnostic("motorCaseMoveCheckStatus for motor " + str(motor) + "...", self.getDiag())

         pv_val = self.getPVBase() + motor + ".VAL"
         pv_dmov = self.getPVBase() + motor + ".DMOV"
         pv_msta = self.getPVBase() + motor + ".MSTA"
         pv_dir = self.getPVBase() + motor + ".DIR"
         pv_stop = self.getPVBase() + motor + ".STOP"

         #First move motor to zero
         self.diagnostic("Move to zero.", self.getDiag())
         self.putPv(pv_val, 0.0, wait=True, timeout=self.getTimeout())

         #Check MSTA Done is set
         self.verify(0x2, (int(self.getPv(pv_msta)) & 0x2))

         #Not do a move, with no callback
         self.diagnostic("Move to 10.", self.getDiag())
         self.putPv(pv_val, 10.0, wait=False)

         #Use cothread sleep instead of time.sleep(), which blocks the cothread library (stopping the previous caput working).
         cothread.Sleep(1.0)

         #Read DIR field
         direction = self.getPv(pv_dir)
         if (direction == 0):
            #Check MSTA Done is not set, and moving flag is on and direction positive is on.
            self.verify(0x401, (int(self.getPv(pv_msta)) & 0x401))
         else:
            #Check MSTA Done is not set, and moving flag is on.
            self.verify(0x400, (int(self.getPv(pv_msta)) & 0x400))

         #Now stop it, check status, and move back to zero
         self.diagnostic("Stopping.", self.getDiag())
         self.putPv(pv_stop, 1, wait=True, timeout=self.getTimeout())

         self.diagnostic("Move completed. Do final MSTA checks.", self.getDiag())
         #Check MSTA Done is set
         self.verify(0x2, (int(self.getPv(pv_msta)) & 0x2))
         #Check MSTA moving is not set
         self.verify(0x0, (int(self.getPv(pv_msta)) & 0x400))

         #Move back to zero
         self.diagnostic("Move back to zero.", self.getDiag())
         self.putPv(pv_val, 0.0, wait=True, timeout=self.getTimeout())




class motorCaseCheckOffset(motorCaseBase):
   """
   Class to check the use of offset. It checks
   that soft limits and user coordinates are
   offset correctly when setting an offset.
   """

   def runTest(self):

      for motor in self.getMotors():

         self.diagnostic("motorCaseCheckOffset for motor " + str(motor) + "...", self.getDiag())

         pv_rbv = self.getPVBase() + motor + ".RBV"
         pv_off = self.getPVBase() + motor + ".OFF"
         pv_val = self.getPVBase() + motor + ".VAL"
         pv_hlm = self.getPVBase() + motor + ".HLM"
         pv_llm = self.getPVBase() + motor + ".LLM"
         pv_rdbd = self.getPVBase() + motor + ".RDBD"

         #Read current soft limits, offset and val
         start_off = self.getPv(pv_off)
         start_val = self.getPv(pv_val)
         start_rbv = self.getPv(pv_rbv)
         start_hlm = self.getPv(pv_hlm)
         start_llm = self.getPv(pv_llm)
         rdbd = self.getPv(pv_rdbd)

         self.diagnostic("Increasing offset by 1.", self.getDiag())

         #Increase offset by 1
         self.putPv(pv_off, start_off+1, wait=True, timeout=self.getTimeout())

         self.diagnostic("Now check the offset fields.", self.getDiag())

         #Now check soft limits, val and rbv
         self.verify(start_off+1, self.getPv(pv_off))
         self.verify(start_val+1, self.getPv(pv_val))
         self.verify(start_hlm+1, self.getPv(pv_hlm))
         self.verify(start_llm+1, self.getPv(pv_llm))

         self.verifyPvInRange(pv_val, start_val+1-rdbd, start_val+1+rdbd)
         self.verifyPvInRange(pv_rbv, start_rbv+1-rdbd, start_rbv+1+rdbd)

         self.diagnostic("Reset original offset.", self.getDiag())

         #Set back old offset
         self.putPv(pv_off, start_off, wait=True, timeout=self.getTimeout())

         

class motorCaseSetPosition(motorCaseBase):
   """
   Class to attempt a set position. The position before and
   after are read and verfied, along with the motor record state.
   """

   def runTest(self):

      for motor in self.getMotors():

         pv_drbv = self.getPVBase() + motor + ".DRBV"
         pv_off = self.getPVBase() + motor + ".OFF"
         pv_dval = self.getPVBase() + motor + ".DVAL"
         pv_val = self.getPVBase() + motor + ".VAL"
         pv_set = self.getPVBase() + motor + ".SET"
         pv_rdbd = self.getPVBase() + motor + ".RDBD"
         pv_dmov = self.getPVBase() + motor + ".DMOV"

         #For the first test, set the offset to zero.
         self.putPv(pv_off, 0, wait=True, timeout=self.getTimeout())

         self.diagnostic("Setting position with a zero offset...")
         
         start_position = self.getPv(pv_drbv)
         offset = self.getPv(pv_off)
         rdbd = self.getPv(pv_rdbd)
         self.diagnostic("Motor " + str(motor) + " is at DRBV: " + str(start_position))
         self.diagnostic("Motor " + str(motor) + " has offset OFF: " + str(offset))
         
         #Do a set position to start_position + 1.0
         end_position = start_position + 1.0
         self.putPv(pv_set, 1, wait=True, timeout=self.getTimeout())
         self.putPv(pv_dval, end_position, wait=True, timeout=self.getTimeout())
         self.putPv(pv_off, offset, wait=True, timeout=self.getTimeout())
         self.putPv(pv_set, 0, wait=True, timeout=self.getTimeout())

         self.verifyPvInRange(pv_drbv, end_position-rdbd, end_position+rdbd)
         self.verifyPvInRange(pv_dval, end_position-rdbd, end_position+rdbd)
         self.verifyPvInRange(pv_val, end_position+offset-rdbd, end_position+offset+rdbd)
         self.verify(offset, self.getPv(pv_off))
         self.verify(1, self.getPv(pv_dmov))
         self.verify(0, self.getPv(pv_set))

               
         #Now repeat this with a non-zero offset
         self.putPv(pv_off, 1.1, wait=True, timeout=self.getTimeout())

         self.diagnostic("Setting position with a offset of 1.1...")

         start_position = self.getPv(pv_drbv)
         offset = self.getPv(pv_off)
         rdbd = self.getPv(pv_rdbd)
         self.diagnostic("Motor " + str(motor) + " is at DRBV: " + str(start_position))
         self.diagnostic("Motor " + str(motor) + " has offset OFF: " + str(offset))
         
         #Do a set position to start_position + 1.0
         end_position = start_position + 1.0
         self.putPv(pv_set, 1, wait=True, timeout=self.getTimeout())
         self.putPv(pv_dval, end_position, wait=True, timeout=self.getTimeout())
         self.putPv(pv_off, offset, wait=True, timeout=self.getTimeout())
         self.putPv(pv_set, 0, wait=True, timeout=self.getTimeout())

         self.verifyPvInRange(pv_drbv, end_position-rdbd, end_position+rdbd)
         self.verifyPvInRange(pv_dval, end_position-rdbd, end_position+rdbd)
         self.verifyPvInRange(pv_val, end_position+offset-rdbd, end_position+offset+rdbd)
         self.verify(offset, self.getPv(pv_off))
         self.verify(1, self.getPv(pv_dmov))
         self.verify(0, self.getPv(pv_set))

         #Move to zero, and set offsets to zero.
         self.putPv(pv_off, 0.0, wait=True, timeout=self.getTimeout())
         self.putPv(pv_val, 0.0, wait=True, timeout=self.getTimeout()) 

         
class motorCaseTestAlarm(motorCaseBase):
   """
   Class to check the behaviour of the motor record
   in the event that axes on the XPS are disabled.
   This is achived by directly communicating with the XPS
   using the python library XPS_C8_drivers from Newport.
   """

   def runTest(self):

      #Open socket to XPS.
      xps = XPS()
      socket = xps.TCP_ConnectToServer(self.getXPSHostname(), self.getXPSPort(), self.getXPSSocketTimeout())

      for motor in self.getMotors():

         self.diagnostic("motorCaseTestAlarm for motor " + str(motor) + " before disabling axes...", self.getDiag())

         pv_stat = self.getPVBase() + motor + ".STAT"
         pv_sevr = self.getPVBase() + motor + ".SEVR"

         stat = self.getPv(pv_stat)
         sevr = self.getPv(pv_sevr)

         self.diagnostic("  STAT= " + str(stat), self.getDiag())
         self.diagnostic("  SEVR= " + str(stat), self.getDiag())
         try:
            self.verify(0, stat)
            self.verify(0, sevr)
         except:
            self.diagnostic("motorCaseTestAlarm. Caught exception from verify. Closing XPS socket and re-throwing.")
            xps.TCP_CloseSocket(socket)
            raise

      self.diagnostic("Disabling XPS group " + self.getXPSGroupName() + "...", self.getDiag())

      #Disable group
      xps.GroupMotionDisable(socket, self.getXPSGroupName())
      cothread.Sleep(2.0)

      for motor in self.getMotors():

         self.diagnostic("motorCaseTestAlarm for motor " + str(motor) + " after disabling axes...", self.getDiag())

         stat = self.getPv(pv_stat)
         sevr = self.getPv(pv_sevr)

         self.diagnostic("  STAT= " + str(stat), self.getDiag())
         self.diagnostic("  SEVR= " + str(sevr), self.getDiag())
         try:
            self.verify(7, stat)
            self.verify(2, sevr)
         except:
            self.diagnostic("motorCaseTestAlarm. Caught exception from verify. Enabling axes, closing XPS socket and re-throwing.")
            xps.GroupMotionEnable(socket, self.getXPSGroupName())
            xps.TCP_CloseSocket(socket)
            raise

      #Enable group again and close socket
      xps.GroupMotionEnable(socket, self.getXPSGroupName())
      xps.TCP_CloseSocket(socket)
      cothread.Sleep(2.0)
