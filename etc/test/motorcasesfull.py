#!/dls_sw/tools/bin/python2.4

from pkg_resources import require
require('dls.autotestframework')
from dls.autotestframework import TestCase

from motorcases import *


class motorCaseMoveSequence2(motorCaseBase):
   """
   Class to do a series of small moves on each motor record.
   """

   def runTest(self):

      distance = 0.01
      moves = 1000

      self.diagnostic("Moving distance of " + str(distance) + " " + str(moves) + " times.", self.getDiag())
      
      self.doMoveSequence(distance, moves)
