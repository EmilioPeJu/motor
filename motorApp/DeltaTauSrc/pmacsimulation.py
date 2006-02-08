#!/usr/bin/env python2.4

# Package imports
import time
import thread

UPDATE_FREQUENCY = 100

# Maximum number of axis on pmac
MAX_AXIS = 32
MAX_IVAR = 3300
MAX_MVAR = 3300

# I-variables
I_ACTIVATE = 00
I_DEMVEL   = 22    # Counts/msec
I_DEMACC   = 19    # Counts/msec^2

# M-variables
M_DEMANDPOS = 61
M_READBACK  = 62

# Status masks
# Word1

# Word2

# Word3
STAT_INPOSITION = 0x1



class pmaccontroller(object):
    def __init__(self):
        #print 'created pmac controller'
               
        # Finish flag used to halt update thread
        self.finish = 0
               
        # Set pmac simulation version
        self.type = 'SIMULATION'
        self.version = 'V1.0'
        
        # Initialise I-variables
        self.ivarinit()
        
        # Initalise M-variables
        self.mvarinit()
        
        # Create motors
        self.createmotors()
        
        # Start controller update thread
        self.update_thread = thread.start_new(self.controllerupdate,())
      
    def gettype(self):
        return self.type
    def getversion(self):
        return self.version
    
    def ivarinit(self):
        #print 'Initialising I-variables'
        self.ivar = {}
        
        # Initialise I-variables to 0
        for i in range(1,MAX_IVAR+1):
            istring = 'I'+str(i)
            self.ivar[istring]=0 
            
        # Set motor activation variable to 1
        for i in range(1,MAX_AXIS+1):
            istring = 'I'+str(i)+str('%.2d'%I_ACTIVATE)
            self.setivar(istring,1)
            
    def mvarinit(self):
        #print 'Initialising M-variables'
        self.mvar = {}
        
        # Initialise M-variables to 0
        for m in range(1,MAX_MVAR+1):
            mstring = 'M'+str(m)
            self.mvar[mstring]=0 
            
        
        
    def createmotors(self):
        # Instantiate motors
        self.motors = {}
        
        for i in range(1,MAX_AXIS+1):
            self.motors[i]=motor(i)
            
    def setivar(self,inum,newval):
        if self.ivar.has_key(inum):
            self.ivar[inum]=newval
        else :
            print 'setivar: Invalid i-variable (',inum,')'
    def getivar(self,inum):
        if self.ivar.has_key(inum):
            return self.ivar[inum]
        else:
            print 'getivar: Invalid i-variable (',inum,')'
            
    def setmvar(self,mnum,newval):
        """ Set M-variable """
        axis = int(mnum[1:-2])
        if mnum[-2:] == str(M_DEMANDPOS):                        
            self.motors[axis].setdempos(newval)
        else:
            print 'setmvar: Unknown M-variable (',mnum,')'
        
        
    def getmvar(self,mnum):
        axis = int(mnum[1:-2])
        if mnum[-2:] == str(M_READBACK):
            return self.motors[axis].getreadback()
        elif mnum[-2:] == str(M_DEMANDPOS):
            return self.motors[axis].getreadback()
        else:
            print 'getmvar: Unknown M-variable (',mnum,')'

    def getstat(self,axis):
        if (axis >= 1) & (axis <= MAX_AXIS+1):
            return str(self.motors[axis].getstatus())
        else:
            print 'getstat: Unknown axis (',axis,')'

    def setpos(self,axis,newpos):
        if( axis >= 1) & (axis <= MAX_AXIS+1):
            self.motors[axis].setdempos(newpos * 32.0)
        else:
            print 'setpos: Unknown axis (',axis,')'
    
    def controllerupdate(self):        
        """ Update thread function (Runs at 10Hz) """
        while not self.finish:
            time.sleep(1.0/UPDATE_FREQUENCY)            
            self.updatestat()
            self.updatepos()

    def stop(self,axis):
        readback = self.motors[axis].getreadback()
        if( axis >= 1) & (axis <= MAX_AXIS+1):
            self.motors[axis].setdempos(readback)
            self.motors[axis].setmotstate('STOP')
        else:
            print 'stop: Unknown axis(',axis,')'
        

    def updatepos(self):
        """ Update Motor Position """
        for axis in range(1,MAX_AXIS+1):
            if not self.motors[axis].getword3() & STAT_INPOSITION:
                #print 'axis %d needs updating' % axis
                dempos   = self.motors[axis].getdempos()
                readback = self.motors[axis].getreadback()
                idemvel = 'I%d%d' % (axis,int(I_DEMVEL))
                demvel   = 1000.0 * float(self.getivar(idemvel))
                idemacc = 'I%d%d' % (axis,int(I_DEMACC))
                demacc   = 1000000.0 * float(self.getivar(idemacc))
                curvel   = self.motors[axis].getcurvel()
                #print dempos, readback, demvel, demacc, curvel
                
                # If we're in STOP state make sure we're in position
                if self.motors[axis].getmotstate() == 'STOP':
                    self.motors[axis].setstatus(3,STAT_INPOSITION,1)
                
                # Increase velocity if we're accelerating
                if self.motors[axis].getmotstate() == 'ACCEL':
                    curvel = curvel + demacc / UPDATE_FREQUENCY
                    #print 'Accelerating curvel = ',curvel
                    
                    # Cap the current velocity 
                    if curvel >= demvel:
                        curvel = demvel
                        self.motors[axis].setmotstate('CONSTVEL')                    
                    
                
                # Constant velocity phase
                if self.motors[axis].getmotstate() == 'CONSTVEL':
                    pass
                
                # Decrease velocity if we're decelerating
                if self.motors[axis].getmotstate() == 'DECEL':
                    curvel = curvel - demacc / UPDATE_FREQUENCY
                    #print 'Decelerating curvel = ',curvel
                    
                    # Cap the current velocity
                    if curvel <= 0:
                        curvel = 0
                        self.motors[axis].setmotstate('STOP')
                        
                # Save current velocity
                self.motors[axis].setcurvel(curvel)
               
                
                if dempos > readback:             
                    readback = readback + ((curvel * 32.0) / UPDATE_FREQUENCY)
                    self.motors[axis].setreadback(readback)
                elif dempos < readback:                
                    readback = readback - ((curvel * 32.0) / UPDATE_FREQUENCY)
                    self.motors[axis].setreadback(readback)
                elif dempos == readback:
                    self.motors[axis].setstatus(3,STAT_INPOSITION,1)
                    
                # Calculate the acceleration/deceleration distances
                if (demacc != 0) & (demvel != 0):
                    accdist  = demvel * demvel / (2 * demacc)
                    decdist  = curvel * curvel / (2 * demacc)
                else:
                    accdist = 1.0
                    decdist = 1.0
                    #print 'velocity/acceleration zero %d %d' % (demvel,demacc)
                    
                    
                # Check to see if we need to slow down
                # velocities and distances are in real units,
                # readbacks in 1/32 units
                if self.motors[axis].getmotstate() != 'DECEL':
                    if abs(readback - dempos) <= (accdist * 32.0):
                        if abs(readback - dempos) <= (decdist * 32.0):
                            #print readback,dempos,accdist,decdist
                            self.motors[axis].setmotstate('DECEL')
                    
        
    def updatestat(self):
        """ Update Motor Status """
        pass

        
class motor(object):
    def __init__(self, axis=0):
        self.axis      = axis
        self.demandpos = 0.0
        self.readback  = 0.0
        self.demandvel = 1000.0
        self.currentvel= 0.0
        self.demandacc = 100.0
        self.status    = 0
        self.statw1    = 0
        self.statw2    = 0
        self.statw3    = 0
        self.motstate  = 'STOP'        
        
    def setmotstate(self,newstate):
        self.motstate = newstate
        
    def getmotstate(self):
        return self.motstate
        
    def setdempos(self,newpos):
        print 'axis %d setting dempos to %d' % (self.axis,newpos)
        self.demandpos = newpos
        
        # Set the status to moving
        self.setstatus(3,STAT_INPOSITION,0)
        self.setmotstate('ACCEL')
        
    def getdemvel(self):
        return self.demandvel
    def getcurvel(self):
        return self.currentvel
    def setcurvel(self,vel):
        self.currentvel = vel
    def getacc(self):
        return self.demandacc
        
    def getdempos(self):
        return self.demandpos
    def setreadback(self,newrbk):
        self.readback = newrbk
    def getreadback(self):
        #print 'axis %d getting readback' % self.axis
        return self.readback 
    def setstatus(self,word,mask,val):
        if word == 1:
            if val:
                self.statw1 = self.statw1 | mask
            else:
                self.statw1 = self.statw1 & ~mask
        if word == 2:
            if val:
                self.statw2 = self.statw2 | mask
            else:
                self.statw2 = self.statw2 & ~mask
        if word == 3:
            if val:
                self.statw3 = self.statw3 | mask
            else:
                self.statw3 = self.statw3 & ~mask

        self.status = '%.4x%.4x%.4x' % (self.statw1,self.statw2,self.statw3)       
                        
    def getstatus(self):
        return self.status
    def getword1(self):
        return self.statw1
    def getword2(self):
        return self.statw2
    def getword3(self):
        return self.statw3

if __name__ == '__main__':
    
    # Instantiate Pmac controller
    pmac = pmaccontroller()
    
    pmac.setmvar('M561',1000)
    
    readback = pmac.getmvar('M562')
    
    print 'readback = ',readback
    
    print 'Main thread sleep'
    while 1:
       time.sleep(1)
