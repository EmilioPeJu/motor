###############################
# Matthew Pearson, March 2010
#
# Script to monitor the digital input on GPIO2,
# and disable or enable axes based on the input.
#
# If the first pin of the digital input is shorted
# to ground, then the inhibit signal is active. The motors
# are stopped, then disabled after 2 sec. Once the signal
# is not inhibited the motors are re-enabled.
# 
# The input is read every second.
#
# This script should be run at startup 
# (listed in the system.ini file)  
#
###############################

set digitalin "GPIO2.DI"

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
      
      puts stdout "Script running OK for motion stop."
     
      #Run forever
      while { 1 == 1 } {

	  #Wait 1s
	  after 1000

	  #Read digital input of GPIO2.
	  set code [catch "GPIODigitalGet $socketID $digitalin value"]
	  if {$code != 0} {
	      DisplayErrorAndClose $socketID $code "GPIOAnalogGet"
	      return
	  } 

	  if {$value == 1} {
	      #Disable axes
	      #Loop over all the groups
	      foreach i [array names groups] {
		  puts stdout "Stopping group: $groups($i)"
		  StopGroup $socketID $groups($i)
	      }
	      #Wait 2s
	      after 2000
	      #Loop over all the groups
	      foreach i [array names groups] {
		  puts stdout "Disabling group: $groups($i)"
		  DisableGroup $socketID $groups($i)
	      }
	  } else {
	      #Enable groups
	      #Loop over all the groups
	      foreach i [array names groups] {
		  puts stdout "Enabling group: $groups($i)"
		  EnableGroup $socketID $groups($i)
	      }
	  }
	  
      }
      #End of while
      
      #### Close TCP socket
      set code [catch "TCP_CloseSocket $socketID"]
} else {
    puts stdout "OpenConnection Error => $code"
}