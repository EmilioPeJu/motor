#!/dls_sw/tools/bin/python2.4

from pkg_resources import require
require('dls.autotestframework')
from dls.autotestframework import TestCase

import re

class motorCaseBase(TestCase):
   """
   Base class for all motor test cases.
   This is where the base PV is hardcoded to be 'mp49:sim'
   """
   
   def __init__(self, A):
      TestCase.__init__(self, A)
      self.__pv1 = "mp49:sim"
      self.__motors = ["1","2","3","4","5","6"]
      self.__diag = 1
      self.__timeout = 1000
      
   def getPVBase(self):
      return self.__pv1

   def getMotors(self):
      return self.__motors

   def getDiag(self):
      return self.__diag

   def getTimeout(self):
      return self.__timeout


   def doMoveSequence(self, distance, moves):
      """
      Do a series of moves on each motor record. The number of moves
      and the distance are arguments to the function.

      The moves are triggered by writing to VAL.
      The start and end positions are checked (to be within RDBD)
      """

      move = 0.0

      for motor in self.getMotors():

         self.diagnostic("Moving motor record " + self.getPVBase() + motor, self.getDiag());

         pv_rbv = self.getPVBase() + motor + ".RBV"
         pv_rdbd = self.getPVBase() + motor + ".RDBD"
         pv_val = self.getPVBase() + motor + ".VAL"

         rdbd = self.getPv(pv_rdbd)
         self.diagnostic(pv_rdbd + ": " + str(rdbd),  self.getDiag())

         for i in range(moves):

            move += distance

            self.diagnostic("Moving to " + str(move), self.getDiag()+1)
            
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
   It reads DMOV, MOVN, DVAl and OFF.
   DVAL and OFF should have been set correctly by autosave.
   """
   
   def runTest(self):

      init_dmov = 1
      init_movn = 0

      for motor in self.getMotors():

         self.diagnostic("Reading motor record " + self.getPVBase() + motor + " initial state", self.getDiag())
      
         pv_dmov = self.getPVBase() + motor + ".DMOV"
         pv_movn = self.getPVBase() + motor + ".MOVN"

         self.diagnostic(pv_dmov + ": " + str(self.getPv(pv_dmov)), self.getDiag())
         self.diagnostic(pv_movn + ": " + str(self.getPv(pv_movn)), self.getDiag())
         
         self.verify(init_dmov, self.getPv(pv_dmov))
         self.verify(init_movn, self.getPv(pv_movn))


class motorCaseMoveSequence(motorCaseBase):
   """
   Class to do a series of moves on each motor record.
   """

   def runTest(self):

      distances = [0.001, 0.01, 0.1, 1.0, 10.0, 100.0]
      moves = 1

      for distance in distances:
         self.diagnostic("Moving distances of " + str(distance), self.getDiag())
         self.doMoveSequence(distance, moves)



   
         
            
