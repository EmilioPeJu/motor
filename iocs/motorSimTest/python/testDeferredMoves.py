#!/bin/env python2.4

"""

 Python script demonstrating the use of
 deferred moves.
 
 Matthew Pearson, May 2009

 The script expects a text file in the same location as the script
 that contains a list of motor PVs and positions to move to. The
 file should be in the format:
 {pv_name1}{single space}{position1}
 {pv_name2}{single space}{position2}
 {pv_name3}{single space}{position3}

 The text file should be called motors.txt

 The deferred move flag should be called defer, eg. mp49:defer
"""


from pkg_resources import require
require("dls.ca2==2.16")

from dls.ca2.catools import *

import sys
import os
import threading
import time

__motors_list = []

def handleEvent(args):
   """
   Callback function for the CA monitor.
   
   This handles the change of state event.
   """
   if args.status == ECA_NORMAL:
      type = args.dbr
      name = ca_name(args.chid)
      if type.value == "1":
         print name.split('.')[0], "has finished at", type.stamp


class CaputThread(threading.Thread):
   """
   Class which provides a caput thread, which uses channel access
   ca_put_callback.
   """
   
   __TIMEOUT = 1000 #seconds
   __thread_run = False
   
   def __init__(self, pvname, pvvalue):
      """
      Constructor
      
      pvname = name of PV (string)
      pvvalue = value to set
      """
      threading.Thread.__init__(self)
      self.setDaemon(1)
      self.__pvname = pvname
      self.__pvvalue = pvvalue
      
   def running(self):
      """
      Returns true if the thread is running.
      """
      return self.isAlive()
      
   def stopThread(self):
      """
      Stop the thread as soon as possible by setting internal flag.
      
      None stopThread()
      """
      if (self.__thread_run):
         self.__thread_run = False
         print "Stopping caput thread."

   def startThread(self):
      """
      Calls self.start() to start the thread.
      
      None startThread()
      
      Resets internal flag used to control thread lifetime.
      """
      self.__thread_run = True
      self.start()
      print "Started caput thread."


   def run(self):
      """
      Main thread. Start this by calling startThread()
      """
      
      print "Moving " + self.__pvname + " to " + str(self.__pvvalue)

      try:
         castat = caput(self.__pvname, self.__pvvalue, self.__TIMEOUT)
      except:
         print "ERROR: Problem with caput on " + self.__pvname
         print castat
         sys.exit(1)

      print "Done moving " + self.__pvname


      

def readMotorsFile(directory):
   """
   Open a file called motors.txt that is in the same location
   as the script.

   The file should be in the format:
   {motorPV} {position}
   {motorPV} {position}
   ...
   """

   filename = directory+"/motors.txt"

   print "Reading file: " + filename

   try:
      file = open(filename, 'r')
      lines = file.readlines()
   except IOError:
      print "ERROR: Could not read file " + filename
      sys.exit(1)
   else:
      file.close()

   for line in lines:
      if (len(line) > 1):
         __motors_list.append(line.strip().strip('\n'))

   print __motors_list


def main():
   print "Script demonstrating deferred moves."

   base_dir = os.path.dirname(__file__)
   readMotorsFile(base_dir)

   castat = 0

   #Generate PV name for deferred flag
   deferpv = ((__motors_list[0].split(':'))[0])+":defer"

   print "Setting deferred move flag..."
   try:
      castat = caput(deferpv, 1)
   except:
      print "ERROR: Problem setting deferred move flag."
      print castat
      sys.exit(1)
      
   #Loop over all the motor PVs.
   for motor in __motors_list:
      pv = motor.split()
         
      #####################Single thread version######################
      castat = caputnowait(pv[0], float(pv[1]))
      keys = camonitor(pv[0]+".DMOV", handleEvent, datatype = dbr_time_string)
      #####################Multi-thread version#######################
      #cathread = CaputThread(pv[0], float(pv[1]))
      #cathread.startThread()

   #Now unset defer flag and do the moves
   print "Executing deferred moves..."
   try:
      castat = caput(deferpv, 0)
   except:
      print "ERROR: Problem setting deferred move flag."
      print castat
      sys.exit(1)
      
   #####################Single thread version######################
   # monitor events are only delivered during ca_pend_event
   while 1:
      ca_pend_event(1.0)

   #####################Multi-thread version#######################
   #Wait for all other threads to finish before exiting main.
   #while (threading.activeCount()>1):
   #   time.sleep(0.1)

   print "Finished"



if __name__ == "__main__":
   main()
   


