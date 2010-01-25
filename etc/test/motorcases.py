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

   def getPVBase(self):
      return self.__pv1
   

class motorCaseReadInit(motorCaseBase):
   """
   Test case to read motor record initial startup.
   It reads DMOV, MOVN, DVAl and OFF.
   DVAL and OFF should have been set correctly by autosave.
   """
   
   def runTest(self):

      motor = "1";

      init_dmov = 1
      init_movn = 0
      
      pv_dmov = self.getPVBase() + motor + ".DMOV"
      pv_movn = self.getPVBase() + motor + ".MOVN"

      print "init_dmov: ", init_dmov
      print "init_movn: ", init_dmov
      print "self.getPv(pv_dmov): ", self.getPv(pv_dmov)
      print "self.getPv(pv_movn): ", self.getPv(pv_movn)

      self.verify(init_dmov, self.getPv(pv_dmov))
      self.verify(init_movn, self.getPv(pv_movn))

