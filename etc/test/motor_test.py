#!/dls_sw/tools/bin/python2.4

# Test suite to use with pyUnit

from pkg_resources import require
require('dls.autotestframework')
from dls.autotestframework import *

from motorcases import *
import pyclbr

################################################
# Test suite for the motor support module.
    
class motorTestSuite(TestSuite):

   def loadCasePlugins(self):
      classes = pyclbr.readmodule("motorcases")
      for c in classes:
         if not (c.endswith("Base")):
            classobj = eval(c)
            if (issubclass(classobj, TestCase)):
               if not (classobj == TestCase):
                  classinstance = classobj(self)

   def createTests(self):
      # Define the targets for this test suite
      Target("simulation", self,
             iocDirectory="iocs/motorSimTest",
             iocBootCmd="screen -D -m -L bin/linux-x86/stmotorSimTest.boot",
             epicsDbFiles="db/motorSimTest.db"
             #guiCmds=['edm -m "lakeshore340=mp49:ls340sim" -x data/lakeshore340.edl'])
             )

      self.loadCasePlugins()
      
     


################################################
# Main entry point

if __name__ == "__main__":
      # Create and run the test sequence
      motorTestSuite()
