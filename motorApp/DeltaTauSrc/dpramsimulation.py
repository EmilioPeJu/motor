"""
Module:
   dpramsimulation.py

Description:
   Module containing function called by embedded 

$Id$

$Revision$
"""

#System imports
import time
import thread
import struct
import re

# Local imports 
from pmacsimulation import *

def testfunc(buf):
    print 'called testfunc'
    while 1:
       time.sleep(0.1)
    print 'leaving testfunc'

def startfunc(buf):
    """
    Function:
       startfunc

    Description:
       Main entry point for drvPmac.c code. This function
       is called once per pmac card and spawns a dpram
       thread each time.

    Arguments:
       buf - Simulated DPRAM buffer passed from C code
    """

    # Launch main function
    thread.start_new_thread(mainfunc,(buf,))
    

def mainfunc(buf):
    """
    Function:
       mainfunc

    Description:
       Main entry point for dpram thread.

    Arguments:
       buf  -  Simulated DPRAM buffer passed from C code
    """

    # Create pmac object
    pmac = pmaccontroller()

    # Launch DPRAM thread
    dpram(buf,pmac)

class dpram:
    """
    Class used to parse and process incoming messages
    from motor record driver code.
    """
    def __init__(self,buf,pmac):
        """
        Initialisation function for DPRAM class
        """        
        # Dual port ram structure (matching drvPmac.h)
        self.dpramstruct="3740xBBH160sxBxxxx258s"

        # List of tokens to be used in regular expression
        self.tokens = ( ('movecmd','(#[0-9]+\s+(?:I[0-9]+=[0-9.]+\s+)*J=[0-9-]+)'),
                        ('type','(TYPE|type)'),
                        ('version','(VERSION|version)'),
                        ('setmvar','(M[0-9]+\s*=\s*[0-9]+)'),
                        ('getmvar','(M[0-9]+)'),
                        ('setivar','(I[0-9]+\s*=\s*[0-9]+)'),
                        ('getivar','(I[0-9]+)'),
                        ('getstat','(#[0-9]+\?)'),
                        ('stop'   ,'(#[0-9]+J\/)'))                       
        
        # Join tokens
        alltokens = [ '%s' % x[1] for x in self.tokens ]
        allreg    = '|'.join(alltokens)

        self.reg       = re.compile(allreg)

        # Control character responses
        self.chr_response = {'CMDERR':0x3,'ACK':0x6,'CR':0xD}

        # Start the main DPRAM thread
        self.dpramthread(buf,pmac)

    def movecmd(self,pmac,command):
        """
        Function to process a move command with optional I-variables
        """
        # Split move command tokens
        cmnds = command.split(' ')

        # Loop over tokens processing in order
        for cmnd in cmnds:
            # There may be empty tokens (due to multiple spaces) Skip these
            if (len(cmnd) > 0):
                # check 1st character of command to see what sort it is.
                if (cmnd[0]=='#'):
                    # Have found axis number, so set it.
                    axis = int(cmnd[1:])
                elif (cmnd[0]=='I'):
                    # this is an i variable, store variable and value.
                    strs = cmnd.split('=')
                    # Note a leading 0 on the motor number has to be stripped.
                    iVar = 'I'+str(int(strs[0][1:]))
                    pmac.setivar(iVar,strs[1])
                elif (cmnd[0]=='J'):
                    # this is an value string "J=VALUE", send value to the axis
                    pmac.setpos(axis,float(cmnd[2:]))
        
        return None
        
    def version(self,pmac,command):
        """
        Return the pmac version
        """
        return pmac.getversion()
    def type(self,pmac,command):
        """
        Return the pmac type
        """
        return pmac.gettype()
    def mset(self,pmac,command):
        """
        Set M-Variable
        """
        # Strip any leading zero's
        mnum = int(command.split('M')[1].split('=')[0])
        val = command.split('=')[1]

        mstring = 'M'+str(mnum)
        pmac.setmvar(mstring,val)

        return None
    def mget(self,pmac,command):
        """
        Get M-Variable
        """
        # Strip any leading zero's
        mnum = int(command.split('M')[1])

        # Reform string
        mstring = 'M'+str(mnum)
        return str(pmac.getmvar(mstring))
    def iset(self,pmac,command):
        """
        Set I-Variable
        """
        # Strip any leading zero's
        inum = int(command.split('I')[1].split('=')[0])
        val = command.split('=')[1]

        istring = 'I'+str(inum)
        pmac.setivar(istring,val)

        return None

    def iget(self,pmac,command):
        """
        Get I-Variable
        """
        # Strip any leading zero's
        inum = int(command.split('I')[1])

        # Reform string
        istring = 'I'+str(inum)
        return str(pmac.getivar(istring))

    def statget(self,pmac,command):
        """
        Get the status for an axis
        """
        # Retrieve axis number from command
        axis = int(command.split('#')[1].split('?')[0])

        return pmac.getstat(axis)
    def stop(self,pmac,command):
        """
        Process stop command
        """
        # Retrieve axis number from command
        axis = int(command.split('#')[1].split('J')[0])

        pmac.stop(axis)
        return None

    def dpramthread(self,buf,pmac):
        """
        Main routine for DPRAM thread
        """
        funcs = ( self.movecmd,
                  self.type,
                  self.version,
                  self.mset,
                  self.mget,
                  self.iset,
                  self.iget,
                  self.statget,
                  self.stop
                 )


        while 1:
            # Unpack the buffer
            asciibuffer=struct.unpack(self.dpramstruct,buf)
            
            # Extract the ascii buffer control bit
            cntrlbit = asciibuffer[0]&1
            
            # Check to see if we have a new command
            if cntrlbit:
                # Extract null terminated command in buffer
                command = asciibuffer[3].split('\x00')[0]

                # Search for regular expression match
                match = self.reg.match(command)
                if match:
                    groups = match.groups()
                    
                    for (n,g) in enumerate(groups):
                        if g is not None:
                            reply = funcs[n](pmac,g)
                            

                    if reply is not None:
                        # Generate acknowledgement
                        d=[0,0,0,'',self.chr_response['CR'],reply]
                        reply = struct.pack(self.dpramstruct,d[0],d[1],d[2],d[3],d[4],d[5])
                    else:
                        d=[0,0,0,'',self.chr_response['ACK'],'']
                        reply = struct.pack(self.dpramstruct,d[0],d[1],d[2],d[3],d[4],d[5])
                        
                    
                    # Copy reply to buffer
                    buf[0:len(reply)]=reply
                else:
                    print 'Invalid message (%s)' % command

                # Send reply to client application
                    
            # Sleep before repeating
            time.sleep(0.1)


