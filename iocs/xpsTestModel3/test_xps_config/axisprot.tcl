###############################
# Matthew Pearson, July 2010
#
# Script to monitor the state of all axes, and 
# if any go into error state then stop all axes.
# 
# The state is read every 0.1s.
#
# This script should be run at startup 
# (listed in the system.ini file)  
#
###############################

set groups(1) "S1"
set groups(2) "S2"


# Display error and close procedure
proc DisplayErrorAndClose {socketID code APIName} {
    global tcl_argv
    if {$code != -2} {
        set code2 [catch "ErrorStringGet $socketID $code strError"]
        if {$code2 != 0} {
            puts "$APIName ERROR => $code - ErrorStringGet ERROR => $code2"
            set tcl_argv(0) "$APIName ERROR => $code - ErrorStringGet ERROR => $code2"
        } else {
            puts stdout "$APIName ERROR => $code : $strError"
            set tcl_argv(0) "$APIName ERROR => $code : $strError"
        }
    } else {
        puts stdout "$APIName ERROR => $code : TCP timeout"
        set tcl_argv(0) "$APIName ERROR => $code : TCP timeout"
    }
    set code2 [catch "TCP_CloseSocket $socketID"] 
    return
}

proc StopGroup {socketID group} {
    #Issue a stop
    set code [catch "GroupMoveAbort $socketID $group"]
}

proc DisableGroup {socketID group} {    
    #Disable group
    set code [catch "GroupMotionDisable $socketID $group"]
}

proc EnableGroup {socketID group} {
    #Enable group
    set code [catch "GroupMotionEnable $socketID $group"]
}



# Open socket
set TimeOut 60
set code [catch "OpenConnection $TimeOut socketID"]
if {$code == 0} {
      
      puts stdout "axisprot.tcl: Script axisprot running OK."
     
      set firsterror 1
      set errcount 0

      #Run forever
      while { 1 == 1 } {

	  #Wait .1s
	  after 100

	  set stopall 0

	  #Read state of all axes. Set a flag to stop all axes if any are in a bad state.
	  foreach i [array names groups] {
	      set code [catch "GroupStatusGet $socketID $groups($i) value"]
	      if {$code != 0} {
		  DisplayErrorAndClose $socketID $code "GroupStatusGet"
		  return
	      }
 
	      #Doing all this in one if statement doesn't seem to work?!
	      if {$value >= 3 && $value <=9} {
		  #puts stdout "Axis $i is in error state: $value."
		  set stopall 1
	      } 
	      if {$value >= 17 && $value <=34} {
		  #puts stdout "Axis $i is in error state: $value."
		  set stopall 1
	      }
	      if {$value == 40 || $value == 50 || $value == 63} {
		  #puts stdout "Axis $i is in error state: $value."
		  set stopall 1
	      }
	  }

	  #Only print an error once. Reset when error are gone.
	  if {$stopall == 0} {
	      if {$errcount == 1} {
		  puts stdout "axisprot.tcl: All axes are OK. Groovy."
	      }
	      set errcount 0
	      set firsterror 1
	  }
	  if {$stopall == 1} {
	      set errcount 1
	  }
	  if {$firsterror == 1 && $errcount == 1} {
	      puts stdout "axisprot.tcl: Stopping all axes. Axis detected in error state."
	      set firsterror 0
	  }
	  
	  if {$stopall == 1} {
	      #Stop each axis in turn.
	      foreach i [array names groups] {
		  #puts stdout "Stopping axis $i"
		  StopGroup $socketID $groups($i)
	      }
	  }
	  
      }
      #End of while
      
      #### Close TCP socket
      set code [catch "TCP_CloseSocket $socketID"]
} else {
    puts stdout "OpenConnection Error => $code"
}