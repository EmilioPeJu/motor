#!/dls_sw/tools/bin/python2.4

# Test suite to use with pyUnit

from pkg_resources import require
require('dls_autotestframework')
from dls_autotestframework import *

from motorcases_xps import *
import pyclbr

################################################
# Test suite for the motor support module.
    
class motorTestSuite(TestSuite):

   def loadCasePlugins(self):
      classes = pyclbr.readmodule("motorcases_xps")
      for c in classes:
         if not (c.endswith("Base")):
            classobj = eval(c)
            if (issubclass(classobj, TestCase)):
               if not (classobj == TestCase):
                  classinstance = classobj(self)

   def createTests(self):
      # Define the targets for this test suite
      Target("xps", self, [
            BuildEntity('motor'),
            IocEntity('ioc', directory='iocs/xpsTest', bootCmd='bin/linux-x86/stxpsTest.boot'),
            EpicsDbEntity('db', directory='iocs/xpsTest', fileName="db/xpsTest.db")])
      

      motorCaseReadInit(self)
      motorCaseMoveSequence1(self) #do a few moves before the autosave test
      motorCaseAutosaveRestoreCheck(self)
      motorCaseMoveSequence1(self)
      motorCaseMoveSequence2(self)
      motorCaseMoveSequence3(self)
      motorCaseMoveSequence4(self)
      motorCaseMoveSequence5(self)
      motorCaseMoveCheckStatus(self)
      motorCaseCheckOffset(self)
      motorCaseSetPosition(self)

#      self.loadCasePlugins()
      
       


################################################
# Main entry point

if __name__ == "__main__":
      # Create and run the test sequence
      motorTestSuite()
