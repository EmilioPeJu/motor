program XPS_trajectoryScan("P=13BMC:,R=traj1,M1=m33,M2=m34,M3=m35,M4=m36,M5=m37,M6=m38,M7=m39,M8=m40")

/*  
 *  This was adapted from trajectoryScan.st written by Mark Rivers!
 *
 *  This sequencer program works with trajectoryScan.db.  It implements
 *  coordinated trajectory motion with the Newport XPS-C8 motor controller.
 *  It can be used with the Newport General Purpose Diffractometer or with any
 *  other set of motors connected to that controller.
 *  
 *  This inforces a minimum trajectory element period of 0.1s because the SNL
 *  must be processed during each element period for the gathering to perform
 *  correctly
 *
 *  This has been converted to use the Asyn XPS driver. 
 *
 *  Jon Kelly
 *  24th May 2005
 *
 * Modifications:
*/

%% #include <string.h>
%% #include <stdio.h>
%% #include "XPS_C8_drivers.h"
%% #include "Socket.h"
%% #include <hostLib.h>
%% #include <remLib.h>
%% #include <netDrv.h>


/* This program must be compiled with the recursive option */
option +r;

/* State codes for Build, Read and Execute. Careful, these must match the
 * corresponding MBBI records, but there is no way to check this */
#define BUILD_STATE_DONE            0
#define BUILD_STATE_BUSY            1
#define READ_STATE_DONE             0
#define READ_STATE_BUSY             1
#define EXECUTE_STATE_DONE          0
#define EXECUTE_STATE_MOVE_START    1
#define EXECUTE_STATE_EXECUTING     2
#define EXECUTE_STATE_FLYBACK       3

/* Status codes for Build, Execute and Read */
#define STATUS_UNDEFINED 0
#define STATUS_SUCCESS   1
#define STATUS_FAILURE   2
#define STATUS_ABORT     3
#define STATUS_TIMEOUT   4

/* Time modes */
#define TIME_MODE_TOTAL         0
#define TIME_MODE_PER_ELEMENT   1

/* Move modes */
#define MOVE_MODE_RELATIVE   0
#define MOVE_MODE_ABSOLUTE   1
#define MOVE_MODE_HYBRID     2


/* Maximum number of motors on MM4005 */
#define MAX_AXES 8

/* Maximum # of trajectory elements.  The number of points XPS can accept
 * is almost unlimited because the data is stored in a file but  the channel 
 * access limit with a double data type is 2000.
 */
#define MAX_ELEMENTS 2000

/* Maximum # of output pulses.  At the moment the pulses are defined by a
 * timer which is synchronised with the trajectory points so the max number
 * of pulses equals the max number of elements.
 */
#define MAX_PULSES 2000

/* Polling interval for waiting for motors to reach their targets */
#define POLL_INTERVAL 0.1

/* getSocket timeout*/
#define SOCKET_TIMEOUT 	1

/* Used within the exec state as a timeout within the while loops which wait for
 * the ss xpsTrajectoryRun to catch up.  The delays are 0.1 second, so 100 loops
 * is 10 seconds. */
#define COUNT_TIMEOUT  100

/* The maximum size of the item names in gathering, e.g. "GROUP2.POSITIONER1.CurrentPosition" */
#define MAX_GATHERING_AXIS_STRING 60
/* Number of items per axis */
#define NUM_GATHERING_ITEMS 2
/* Total length of gathering string */
#define MAX_GATHERING_STRING MAX_GATHERING_AXIS_STRING * NUM_GATHERING_ITEMS * MAX_AXES

/* XPS specific PVs */
string groupName;  assign groupName 	to "{P}{R}GroupName.VAL";
string ipAddress;  assign ipAddress 	to "{P}{R}IP.VAL";
string asynPort;   assign asynPort      to "{P}{R}AsynPort.VAL";
int    xpsPort;	   assign xpsPort	to "{P}{R}XPSPort.VAL";
string axisName[8];	    
		   assign axisName 	to 
						{"{P}{R}Axis1.VAL",
					 	 "{P}{R}Axis2.VAL",
					 	 "{P}{R}Axis3.VAL",
					 	 "{P}{R}Axis4.VAL",
					 	 "{P}{R}Axis5.VAL",
					 	 "{P}{R}Axis6.VAL",
					 	 "{P}{R}Axis7.VAL",
					 	 "{P}{R}Axis8.VAL"};


/* Define PVs */
int    numAxes;     assign numAxes      to "{P}{R}NumAxes.VAL"; 
                                                        monitor numAxes;
int    nelements;   assign nelements    to "{P}{R}Nelements.VAL"; 
                                                        monitor nelements;
int    npulses;     assign npulses      to "{P}{R}Npulses.VAL"; 
                                                        monitor npulses;

int    moveMode;    assign moveMode     to "{P}{R}MoveMode.VAL";    
                                                        monitor moveMode;
double time;        assign time         to "{P}{R}Time.VAL";      
                                                        monitor time;
double timeScale;   assign timeScale    to "{P}{R}TimeScale.VAL"; 
                                                        monitor timeScale;
int    timeMode;    assign timeMode     to "{P}{R}TimeMode.VAL";    
                                                        monitor timeMode;
double accel;       assign accel        to "{P}{R}Accel.VAL";     
                                                        monitor accel;

int    build;       assign build        to "{P}{R}Build.VAL";     
                                                        monitor build;
int    buildState;  assign buildState   to "{P}{R}BuildState.VAL"; 
int    buildStatus; assign buildStatus  to "{P}{R}BuildStatus.VAL"; 
string buildMessage;assign buildMessage to "{P}{R}BuildMessage.VAL";

int    simMode;     assign simMode      to "{P}{R}SimMode.VAL";   
                                                        monitor simMode;
int    execute;     assign execute      to "{P}{R}Execute.VAL";   
                                                        monitor execute;
int    execState;   assign execState    to "{P}{R}ExecState.VAL";
							monitor execState;   
int    execStatus;  assign execStatus   to "{P}{R}ExecStatus.VAL";   
string execMessage; assign execMessage  to "{P}{R}ExecMessage.VAL";
int    abort;       assign abort        to "{P}{R}Abort.VAL";   
                                                        monitor abort;

int    detOn;       assign detOn        to "{P}{R}DetOn.PROC";
int    detOff;      assign detOff       to "{P}{R}DetOff.PROC";

int    readback;    assign readback     to "{P}{R}Readback.VAL";   
                                                        monitor readback;
int    readState;   assign readState    to "{P}{R}ReadState.VAL";   
int    readStatus;  assign readStatus   to "{P}{R}ReadStatus.VAL";   
string readMessage; assign readMessage  to "{P}{R}ReadMessage.VAL";

double timeTrajectory[MAX_ELEMENTS];
                    assign timeTrajectory to  "{P}{R}TimeTraj.VAL"; 
                                                        monitor timeTrajectory;

int    moveAxis[MAX_AXES]; 
                    assign moveAxis     to
                                           {"{P}{R}M1Move.VAL",
                                            "{P}{R}M2Move.VAL",
                                            "{P}{R}M3Move.VAL",
                                            "{P}{R}M4Move.VAL",
                                            "{P}{R}M5Move.VAL",
                                            "{P}{R}M6Move.VAL",
                                            "{P}{R}M7Move.VAL",
                                            "{P}{R}M8Move.VAL"};
                                                        monitor moveAxis;

double  motorTrajectory[MAX_AXES][MAX_ELEMENTS]; 
                    assign motorTrajectory to
                                           {"{P}{R}M1Traj.VAL",
                                            "{P}{R}M2Traj.VAL",
                                            "{P}{R}M3Traj.VAL",
                                            "{P}{R}M4Traj.VAL",
                                            "{P}{R}M5Traj.VAL",
                                            "{P}{R}M6Traj.VAL",
                                            "{P}{R}M7Traj.VAL",
                                            "{P}{R}M8Traj.VAL"};
                                                        monitor motorTrajectory;

double  motorReadbacks[MAX_AXES][MAX_PULSES]; 
                    assign  motorReadbacks to
                                           {"{P}{R}M1Actual.VAL",
                                            "{P}{R}M2Actual.VAL",
                                            "{P}{R}M3Actual.VAL",
                                            "{P}{R}M4Actual.VAL",
                                            "{P}{R}M5Actual.VAL",
                                            "{P}{R}M6Actual.VAL",
                                            "{P}{R}M7Actual.VAL",
                                            "{P}{R}M8Actual.VAL"};

double  motorError[MAX_AXES][MAX_PULSES]; 
                    assign  motorError  to
                                           {"{P}{R}M1Error.VAL",
                                            "{P}{R}M2Error.VAL",
                                            "{P}{R}M3Error.VAL",
                                            "{P}{R}M4Error.VAL",
                                            "{P}{R}M5Error.VAL",
                                            "{P}{R}M6Error.VAL",
                                            "{P}{R}M7Error.VAL",
                                            "{P}{R}M8Error.VAL"};

double  motorCurrent[MAX_AXES]; 
                    assign  motorCurrent to
                                           {"{P}{R}M1Current.VAL",
                                            "{P}{R}M2Current.VAL",
                                            "{P}{R}M3Current.VAL",
                                            "{P}{R}M4Current.VAL",
                                            "{P}{R}M5Current.VAL",
                                            "{P}{R}M6Current.VAL",
                                            "{P}{R}M7Current.VAL",
                                            "{P}{R}M8Current.VAL"};

double  motorMVA[MAX_AXES]; 
                    assign  motorMVA    to
                                           {"{P}{R}M1MVA.VAL",
                                            "{P}{R}M2MVA.VAL",
                                            "{P}{R}M3MVA.VAL",
                                            "{P}{R}M4MVA.VAL",
                                            "{P}{R}M5MVA.VAL",
                                            "{P}{R}M6MVA.VAL",
                                            "{P}{R}M7MVA.VAL",
                                            "{P}{R}M8MVA.VAL"};

double  motorMAA[MAX_AXES]; 
                    assign  motorMAA    to
                                           {"{P}{R}M1MAA.VAL",
                                            "{P}{R}M2MAA.VAL",
                                            "{P}{R}M3MAA.VAL",
                                            "{P}{R}M4MAA.VAL",
                                            "{P}{R}M5MAA.VAL",
                                            "{P}{R}M6MAA.VAL",
                                            "{P}{R}M7MAA.VAL",
                                            "{P}{R}M8MAA.VAL"};

double  epicsMotorPos[MAX_AXES]; 
                    assign  epicsMotorPos  to
                                           {"{P}{M1}.VAL",
                                            "{P}{M2}.VAL",
                                            "{P}{M3}.VAL",
                                            "{P}{M4}.VAL",
                                            "{P}{M5}.VAL",
                                            "{P}{M6}.VAL",
                                            "{P}{M7}.VAL",
                                            "{P}{M8}.VAL"};
                                                        monitor epicsMotorPos;

double  epicsMotorDir[MAX_AXES]; 
                    assign  epicsMotorDir  to
                                           {"{P}{M1}.DIR",
                                            "{P}{M2}.DIR",
                                            "{P}{M3}.DIR",
                                            "{P}{M4}.DIR",
                                            "{P}{M5}.DIR",
                                            "{P}{M6}.DIR",
                                            "{P}{M7}.DIR",
                                            "{P}{M8}.DIR"};
                                                        monitor epicsMotorDir;
double  epicsMotorOff[MAX_AXES]; 
                    assign  epicsMotorOff  to
                                           {"{P}{M1}.OFF",
                                            "{P}{M2}.OFF",
                                            "{P}{M3}.OFF",
                                            "{P}{M4}.OFF",
                                            "{P}{M5}.OFF",
                                            "{P}{M6}.OFF",
                                            "{P}{M7}.OFF",
                                            "{P}{M8}.OFF"};
                                                        monitor epicsMotorOff;
double  epicsMotorDone[MAX_AXES]; 
                    assign  epicsMotorDone  to
                                           {"{P}{M1}.DMOV",
                                            "{P}{M2}.DMOV",
                                            "{P}{M3}.DMOV",
                                            "{P}{M4}.DMOV",
                                            "{P}{M5}.DMOV",
                                            "{P}{M6}.DMOV",
                                            "{P}{M7}.DMOV",
                                            "{P}{M8}.DMOV"};
                                                        monitor epicsMotorDone;

evflag buildMon;        sync build      buildMon;
evflag executeMon;      sync execute    executeMon;
evflag abortMon;        sync abort      abortMon;
evflag readbackMon;     sync readback   readbackMon;
evflag nelementsMon;    sync nelements  nelementsMon;
evflag execStateMon;    sync execState  execStateMon;

int status;
int i;
int j;
int k;
int anyMoving;
int ncomplete;
int nextElement;
int nextra;
int dir;
int    pollSocket;
int    abortSocket;
int    socket;
int    positionSocket;
int    xpsStatus;
int    count;
double dtime;
double minPeriod;
double posActual;
double posTheory;
double expectedTime;
double initialPos[MAX_AXES];
double trajVel;
double preDistance[MAX_AXES];
double postDistance[MAX_AXES];
double motorLowLimit[MAX_AXES];
double motorHighLimit[MAX_AXES];
double motorMinPos[MAX_AXES];
double motorMaxPos[MAX_AXES];
double pulseTime;
double pulsePeriod;
char groupName[NAME_LEN];
char xpsAddress[NAME_LEN];
char *axisName[MAX_AXES];
char macroBuf[NAME_LEN];
char motorName[NAME_LEN];
char userName[NAME_LEN];
char password[NAME_LEN];
int  xpsPort;

%%char divisor[] = "1";	/* gather/pulse every # traj elements */
%%char GPIOname[] = "GPIO4.DO";	/* DB15 Connector */
%%char pulsemask[] = "63"; /* Defines which pins are pulsed, 63 in base 2 ->00011111 */
%%char strNpulses[10];
%%char strElementNumber[10];
%%char nullchar[] = " ";

/* All PVs which will be accessed in local C functions need to have their index
 * extracted with pvIndex() */
int motorCurrentIndex[MAX_AXES];
int epicsMotorDoneIndex[MAX_AXES];

/* Note, this should be time_t, but SNL doesn't understand that.  This is
 * the defininition in vxWorks. */
unsigned long startTime;  

/* Define escaped C functions at end of file */
%% static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos);
%% static int getMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitMotors(SS_ID ssId, struct UserVar *pVar);
%% static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar);
%% static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar);

%% static int getSocket(SS_ID ssId, struct UserVar *pVar);
%% static int trajectoryExecute(SS_ID ssId, struct UserVar *pVar);
%% static int buildAndVerify(SS_ID ssId, struct UserVar *pVar);
%% static int currentElement(SS_ID ssId, struct UserVar *pVar);
%% static int getGroupStatus(SS_ID ssId, struct UserVar *pVar);
%% static void readGathering(SS_ID ssId, struct UserVar *pVar);
%% static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar);


ss xpsTrajectoryScan {

    /* Initialize things when first starting */
    state init {
        when() {
            /* Get the values from the macro parameteters */ 
            strcpy(groupName, macValueGet("GROUP"));
            strcpy(xpsAddress, macValueGet("IPADDR"));
            strcpy(userName, macValueGet("USERNAME"));
            strcpy(password, macValueGet("PASSWORD"));
            xpsPort = atoi(macValueGet("PORT"));
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state init:"
                       " xpsAddress=%s groupName=%s\n",
                       xpsAddress, groupName);
            }
            for (i=0; i<numAxes; i++) {
                axisName[i] = malloc(NAME_LEN);
                sprintf(macroBuf, "P%d", i+1);
                sprintf(axisName[i], "%s.%s", groupName, macValueGet(macroBuf));
                sprintf(macroBuf, "M%d", i+1);
                sprintf(motorName, "%s%s.VAL", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorPos[i], motorName);
                sprintf(motorName, "%s%s.DIR", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorDir[i], motorName);
                sprintf(motorName, "%s%s.OFF", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorOff[i], motorName);
                sprintf(motorName, "%s%s.DMOV", macValueGet("P"), macValueGet(macroBuf));
                pvAssign(epicsMotorDone[i], motorName);
            }
            xpsStatus = 0;
            ncomplete = 1;
        
            /* Set the arrays to zero */
            for (i=0; i<MAX_ELEMENTS; i++) {
                for (j=0; j<MAX_AXES; j++) {
                    motorTrajectory[j][i] = 0.0;
                }
            }

	    /* setup a simple 10pt traj for test 
	    for (i=2; i<=8; i++) {
	    motorTrajectory[0][i] = 2;
	    }
	    motorTrajectory[1][0] = 0.1;
	    motorTrajectory[1][1] = 0.2;
	    motorTrajectory[1][2] = 0.5;
	    motorTrajectory[1][3] = 1;
	    motorTrajectory[1][4] = 1.8;
	    motorTrajectory[1][5] = 3;
	    motorTrajectory[1][6] = 2;
	    motorTrajectory[1][7] = 1;
	    motorTrajectory[1][8] = 0;
	    
	    motorTrajectory[2][1] = 0.5;
	    motorTrajectory[2][2] = 1;
	    motorTrajectory[2][3] = 0.5;
	    motorTrajectory[2][4] = 0;
	    motorTrajectory[2][5] = 0.5;
	    motorTrajectory[2][6] = 1;
	    motorTrajectory[2][7] = 0.5;
	    motorTrajectory[2][8] = 0;*/
	    
	   
	    /* Define the file name used within the XPS */
	    strcpy (trajFileName,"TrajectoryScan.trj"); 
	    
	    /* Force numAxes to be <= MAX_AXES */
            if (numAxes > MAX_AXES) numAxes = MAX_AXES;
            
	    /* Ask XPS for a socket 1 to drive 1 to poll*/
	    %%pVar->socket = getSocket(ssId, pVar);
	    %%pVar->pollSocket = getSocket(ssId, pVar);
	    %%pVar->abortSocket = getSocket(ssId, pVar);
	    %%pVar->positionSocket = getSocket(ssId, pVar);
	    
            for (j=0; j<numAxes; j++) {
                motorCurrentIndex[j]   =  pvIndex(motorCurrent[j]);
                epicsMotorDoneIndex[j] =  pvIndex(epicsMotorDone[j]);
            }

            /* Clear all event flags */
            efClear(buildMon);
            efClear(executeMon);
            efClear(abortMon);
            efClear(readbackMon);
            efClear(nelementsMon);
        } state monitor_inputs
    }

    /* Monitor inputs which control what to do (Build, Execute, Read) */
    state monitor_inputs {
        entry {
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state monitor_inputs:"
                       " entry\n");
            }
        }
        when(efTestAndClear(buildMon) && (build==1)) {
        } state build

        when(efTest(executeMon) && (execute==1) 
				&& (buildStatus == STATUS_SUCCESS)){
        } state execute

        when(efTestAndClear(readbackMon) && (readback==1)
				&& (execStatus == STATUS_SUCCESS)) {
        } state readback

    }


    /* Build and verify trajectory */
    state build {
        entry {
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state build:"
                       " entry\n");
            }
        }
        when() {
	    /* Set busy flag while building */
            buildState = BUILD_STATE_BUSY;
            pvPut(buildState);
            buildStatus=STATUS_UNDEFINED;
            pvPut(buildStatus);
            /* SPEC adds up the total time  */
            
	    /* If total time mode, calc time per element and write,
	       else use the array timeTraj as written by SPEC */
	    if (timeMode == TIME_MODE_TOTAL) {
                dtime = time/nelements;
		for (i=0; i<nelements; i++) timeTrajectory[i] = dtime;
                pvPut(timeTrajectory);
	    }
	    
            /* Compute expected time for trajectory & check element period */
            expectedTime=0;
            minPeriod = 100000;
	    for (i=0; i<nelements; i++) {
                expectedTime += timeTrajectory[i];
		
		if (minPeriod > timeTrajectory[i]) /* Used in build function */
		    minPeriod = timeTrajectory[i];
	    }
	    
	    time = expectedTime;
	    pvPut(time);
	    
	    %%buildAndVerify(ssId, pVar);
	     
	    /* Export values to PVs */
	    for (j=0; j<numAxes; j++) {       
                    pvPut(motorMVA[j]);
		    pvPut(motorMAA[j]);
	    } 
           /* Clear busy flag, post status */
            buildState = BUILD_STATE_DONE;
            pvPut(buildState);
            pvPut(buildStatus);
            pvPut(buildMessage);
            /* Clear build command, post.  This is a "busy" record, don't want
             * to do this until build is complete. */
            build=0;
            pvPut(build);
            /* Set exec status to UNDEFINED */
            execStatus = STATUS_UNDEFINED;
            pvPut(execStatus);
            strcpy(execMessage, " ");
 
        } state monitor_inputs
    }


    state execute {
        entry {
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state execute:"
                       " entry\n");
            }
	    
	    %%waitMotors(ssId, pVar);
	    
            /* Get the initial positions of the motors */
            for (j=0; j<numAxes; j++) initialPos[j] = epicsMotorPos[j];
            /* Move to start position if required */
            if (moveMode == MOVE_MODE_ABSOLUTE) {
                for (j=0; j<numAxes; j++) {
                    if (!moveAxis[j]) continue;
                    epicsMotorPos[j] = motorTrajectory[j][0];
                    pvPut(epicsMotorPos[j]);
                }
                %%waitEpicsMotors(ssId, pVar);
            }
	    
	    %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
	    /* Set busy flag */
            execState = EXECUTE_STATE_MOVE_START;
            pvPut(execState);
           /* Define pulse output for trajectory */
            if (npulses > 0) {
                /* Check validity, modify values if necessary */
                if (startPulses < 1) startPulses = 1;
                if (startPulses > nelements) startPulses = nelements;
                pvPut(startPulses);
                if (endPulses < startPulses) endPulses = startPulses;
                if (endPulses > nelements) endPulses = nelements;
                pvPut(endPulses);

                /* The XPS can only output pulses at a fixed period, not a fixed
                 * distance along the trajectory.  
                 * The trajectory elements where pulses start and stop are
                 * defined with the PVs startPulses and endPulses.
                 * Compute the time between pulses as the total time over which pulses 
                 * should be output divided by the number of pulses to be output. */
                pulseTime=0;
                for (i=startPulses; i<=endPulses; i++) {
                    pulseTime += timeTrajectory[i-1];
                }
                pulsePeriod = pulseTime/npulses;
            } else {
                pulsePeriod = 0.;
            }

            /* Check that the trajectory won't exceed the software limits */
            for (j=0; j<numAxes; j++) {
                if (moveAxis[j]) {
                    status = PositionerUserTravelLimitsGet(pollSocket,
                                                           axisName[j],
                                                           &motorLowLimit[j], 
                                                           &motorHighLimit[j]);
                    if ((epicsMotorPos[j] + motorMinPos[j]) < motorLowLimit[j]) {
                        execStatus = STATUS_FAILURE;
                        pvPut(execStatus);
                        sprintf(execMessage, "Low soft limit violation on motor %d", j);
                        pvPut(execMessage);
                    }
                    if ((epicsMotorPos[j] + motorMaxPos[j]) > motorHighLimit[j]) {
                        execStatus = STATUS_FAILURE;
                        pvPut(execStatus);
                        sprintf(execMessage, "High soft limit violation on motor %d", j);
                        pvPut(execMessage);
                    }
                }
            }
        }
 

        when (execStatus == STATUS_FAILURE) {
            /* Clear execute command, post.  This is a "busy" record, don't
             * want to do this until execution is complete. */
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state execute:"
                       " execStatus = STATUS_FAILURE\n");
            }
            execute=0;
            pvPut(execute);
        } state monitor_inputs


        when (execStatus != STATUS_FAILURE) {
            %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
            /* Setting execState here will cause the xpsTrajectoryRun SS to wake up */
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state execute:"
                       " setting execState = EXECUTE_STATE_MOVE_START\n");
            }
            execState = EXECUTE_STATE_MOVE_START;
            pvPut(execState);
            count = 0;
            while (execState != EXECUTE_STATE_EXECUTING && 
                   count < COUNT_TIMEOUT ) {
                epicsThreadSleep(0.1);
                count++;
            }
            if (count == COUNT_TIMEOUT) {
                strcpy(execMessage, "Exec Timeout!");
                pvPut(execMessage);
                execStatus = STATUS_ABORT;
                pvPut(execStatus);
            }        
            count = 0;
            /* Wait until ss xpsTrajectoryRun has started the traj scan */
            while (xpsStatus != 45 && 
                   count < COUNT_TIMEOUT && 
                   execStatus != STATUS_FAILURE ) {
                epicsThreadSleep(0.1);
                count++;
                %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
            }
            if (count == COUNT_TIMEOUT) {
                strcpy(execMessage, "Exec Timeout!");
                pvPut(execMessage);
                execStatus = STATUS_ABORT;
                pvPut(execStatus);
            }
            readStatus=STATUS_UNDEFINED;
            pvPut(readStatus);
	    /* Start the detector */
	    detOn = 1;
            pvPut(detOn);
	    
	    /* Get start time of execute */	    
	    startTime = time(0);
	    
	    /* Add Event for 2nd element because the first element has started so it would
	       not be added by the execState=EXEC code */
	    %%pVar->status = EventAdd(pVar->pollSocket,\
				     pVar->axisName[0],\
			      "PVT.ElementNumberStart",\
					  	   "2",\
				 	     "DOPulse",\
					      GPIOname,\
					     pulsemask,\
					      nullchar);
	    if (status != 0) 
                printf("Error EventAdd ElementNumberStart %i element 1\n",status);
	    
        } state wait_execute
    }


    /* Wait for trajectory to complete */
    state wait_execute {
        entry {
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state wait_execute:"
                       " entry\n");
            }
        }
        when (execStatus == STATUS_ABORT) {
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state wait_execute:"
                       " execStatus = STATUS_ABORT\n");
            }
            /* The trajectory_abort state set has detected an abort. It has
             * already posted the status and message.  Don't execute flyback
             * return to top */
            execState = EXECUTE_STATE_DONE;
            pvPut(execState);
            /* Clear execute command, post.  This is a "busy" record, don't
             * want to do this until execution is complete. */
            execute=0;
            pvPut(execute);
	    efClear(executeMon);
        } state monitor_inputs

        when (delay(0.1) && execState==EXECUTE_STATE_EXECUTING) {
            /* Get the current motor positions, post them */
            %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
            for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);

            %%pVar->xpsStatus = getGroupStatus(ssId, pVar);
	    
	    nextElement = ncomplete + 1;            
	    %%pVar->ncomplete = currentElement(ssId, pVar);
	    
	    /* Define the Pulse event once the previous one has been called */
	    if(ncomplete == nextElement && xpsStatus == 45) { /* Just started next element */
	    
	        sprintf(strElementNumber,"%i",(ncomplete + 1));	/* Convert to a string */
	    
		%%pVar->status = EventAdd(pVar->pollSocket,\
					 pVar->axisName[0],\
				  "PVT.ElementNumberStart",\
					  strElementNumber,\
						 "DOPulse",\
						  GPIOname,\
						 pulsemask,\
						  nullchar);
	        if (status != 0) 
                    printf("Error EventAdd ElementNumberStart %i element %i\n",status,i);
		
	    }
            if(xpsStatus == 45)
	        sprintf(execMessage, "Executing element %d/%d", 
                                        ncomplete, nelements);
            pvPut(execMessage);
            %%pVar->xpsStatus = getGroupStatus(ssId, pVar); 
            /* 12 = ready from move */
	    if (xpsStatus == 12) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_SUCCESS;
                strcpy(execMessage, " ");
            }
            /* See if the elapsed time is more than expected, time out */
            if (difftime(time(0), startTime) > (expectedTime+10)) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_TIMEOUT;
                strcpy(execMessage, "Timeout");
            }
            /* see if the xps group status reflects an error. */
	    /* 45 = performing a trajectory, <10 = notinitialised due to major error */
	    if (xpsStatus < 10) {
                execState = EXECUTE_STATE_FLYBACK;
                execStatus = STATUS_FAILURE;
                strcpy(execMessage,"XPS Status Error");
            }
        } state wait_execute

        when(execState==EXECUTE_STATE_FLYBACK) {
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state wait_execute:"
                       " execStatus = EXECUTE_STATE_FLYBACK\n");
            }
            /* Stop the detector */
            detOff = 1;
            pvPut(detOff);
            pvPut(execState);
            pvPut(execStatus);
            pvPut(execMessage);

            /* Only do the following if the trajectory executed OK */
            if (execStatus == STATUS_SUCCESS) { 
                /* Make sure the motors have stopped */
                %%waitMotors(ssId, pVar);
                %%waitEpicsMotors(ssId, pVar);        
            
                /* Get the current motor positions, post them */
                %%getMotorPositions(ssId, pVar, pVar->motorCurrent);
                for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
                for (j=0; j<numAxes; j++) {
                    epicsMotorPos[j] = motorCurrent[j];
                    pvPut(epicsMotorPos[j]);
                }
                %%waitEpicsMotors(ssId, pVar);
            }
            
	    execState = EXECUTE_STATE_DONE;
            pvPut(execState);
            /* Clear execute command, post.  This is a "busy" record, don't
             * want to do this until execution is complete. */
            execute=0;
	    pvPut(execute);
	    efClear(executeMon);
           
        } state monitor_inputs
    }
    /* Read back actual positions */
    state readback {
        entry {
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryScan: state readback:"
                       " entry\n");
            }
        }
        when() {
            /* Set busy flag */
            readState = READ_STATE_BUSY;
            pvPut(readState);
            readStatus=STATUS_UNDEFINED;
            pvPut(readStatus);
            /* Erase the readback and error arrays */
            for (j=0; j<numAxes; j++) {
                for (i=0; i<MAX_PULSES; i++) {
                    motorReadbacks[j][i] = 0.;
                    motorError[j][i] = 0.;
		    
                }
            }
            
            %%readGathering(ssId, pVar);
           
            /* readGathering has set the actual number of positions read back */
            pvPut(nactual);
 
	    /* Post the readback and error arrays */
            for (j=0; j<numAxes; j++) {
                pvPut(motorReadbacks[j]);
                pvPut(motorError[j]);
		pvPut(motorTrajectory[j]);
            }
            /* Clear busy flag */
            readState = READ_STATE_DONE;
            pvPut(readState);
            /* For now we are not handling read errors */
            readStatus = STATUS_SUCCESS;  
            pvPut(readStatus);
            strcpy(readMessage, " ");
            pvPut(readMessage);
            /* Clear readback command, post.  This is a "busy" record, don't
             * want to do this until readback is complete. */
            readback=0;
            pvPut(readback);
        } state monitor_inputs
    }
}


/* This state set simply monitors the abort input.  It is a separate state set
 * so that it is always active, no matter what the state of the trajectoryScan
 * state set. It also uses a separate socket. */
ss xpsTrajectoryAbort {
    state monitorAbort {
        when (efTestAndClear(abortMon) && (abort==1) &&
					(execState==EXECUTE_STATE_EXECUTING)) {

            execStatus = STATUS_ABORT;
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryAbort: state monitorAbort:"
                       " setting execStatus = STATUS_ABORT\n");
            }
            pvPut(execStatus);
            strcpy(execMessage, "Motion aborted");
            pvPut(execMessage);
	    
	    %%trajectoryAbort(ssId, pVar);
	    
            /* Clear abort command, post.  This is a "busy" record, don't
             * want to do this until abort command has been sent. */
            abort=0;
            pvPut(abort);
	    efClear(abortMon);
        } state monitorAbort
    }
}
/* This state was required because the trajectory execute call did not return
 * until the end of the trajectory. So to poll during the scan we call the 
 * scan from a separate state set.  Now the driver has been converted to Asyn the
 * socket no longer hangs so this SS could be recombined. */
ss xpsTrajectoryRun {
    state asyncExecute {
        when (efTestAndClear(execStateMon) && (execState == EXECUTE_STATE_MOVE_START)) {
            /*%%pVar->xpsStatus = getGroupStatus(ssId, pVar);*/
            if (debugLevel > 0) {
                printf("XPS_trajectoryScan: ss xpsTrajectoryRun: state asyncExecute:"
                       " setting execState = EXECUTE_STATE_EXECUTING\n");
            }
            execState = EXECUTE_STATE_EXECUTING;
            pvPut(execState);

            /* If ready to move */
            if (xpsStatus > 9 && xpsStatus < 20) {
                /* Start the detector */
                detOn = 1;
                pvPut(detOn);
                /* Get start time of execute */        
                startTime = time(0);
                /* Call the C function from here so that the main state set can poll */
                %%trajectoryExecute(ssId, pVar);
            } else {
                execStatus = STATUS_FAILURE;
                pvPut(execStatus);
            }
        } state asyncExecute
    }
}
/* This state set polls every second to keep the motor current position medm 
 * screen up to date when a scan is not running */
ss xpsTrajectoryPosition {
    state positionUpdate {
    	when (delay (1) && (execState == EXECUTE_STATE_DONE)){
	%%getMotorPositions(ssId, pVar, pVar->motorCurrent);
        for (j=0; j<numAxes; j++) pvPut(motorCurrent[j]);
	} state positionUpdate
    }
}

%{

/* getMotorPositions returns the positions of each motor */
static int getMotorPositions(SS_ID ssId, struct UserVar *pVar, double *pos)
{
    int status;

    /* Read the current positions of all the axes */
    
    status = GroupPositionCurrentGet(pVar->positionSocket,
    			pVar->groupName,pVar->numAxes,pos);
    if (status != 0)
        	printf(" Error performing GroupPositionCurrentGet%i\n",status);
    return(0);
}


/* Returns 0 when no motors are moving */
static int getMotorMoving(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int moving=0;
    int groupStatus;

    /* Read the current status of the group */

    status = GroupStatusGet(pVar->pollSocket,pVar->groupName,&groupStatus);
    if (status != 0)
        printf(" Error performing GroupStatusGet %i\n",status);
	
    if (groupStatus > 42)
        moving = 1;

    return(moving);
}

/* getEpicsMotorMoving returns the EPICS moving status of each motor, packed into 
 * a single int.  Bit 0 = motor 1, bit 1 = motor 2, etc. 0=not moving, 1=moving.
 * If the entire int is 0 then no motors are moving */
static int getEpicsMotorMoving(SS_ID ssId, struct UserVar *pVar)
{
    int j;
    int result=0, mask=0x01;

    for (j=0; j<pVar->numAxes; j++) {
        seq_pvGet(ssId, pVar->epicsMotorDoneIndex[j], 0);
        if (pVar->epicsMotorDone[j] == 0) result |= mask;
        mask = mask << 1;
    }
    return(result);
}

/* waitMotors waits for all motors to stop moving.  It reads and posts the
 * motor positions during each loop. */
static int waitMotors(SS_ID ssId, struct UserVar *pVar)
{
    int j;

    /* Logic is that we always want to post position motor positions 
     * after the end of move is detected. */
    while(getMotorMoving(ssId, pVar)) {
        /* Get the current motor positions, post them */
        getMotorPositions(ssId, pVar, pVar->motorCurrent);
        for (j=0; j<pVar->numAxes; j++) seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
        epicsThreadSleep(POLL_INTERVAL);
    }
    getMotorPositions(ssId, pVar, pVar->motorCurrent);
    for (j=0; j<pVar->numAxes; j++) seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
    return(0);
}

/* waitEpicsMotors waits for all motors to stop moving using the EPICS motor
 * records..  It reads and posts the motor positions during each loop. */
static int waitEpicsMotors(SS_ID ssId, struct UserVar *pVar)
{
    int j;

    /* Logic is that we always want to post position motor positions 
     * after the end of move is detected. */
    while(getEpicsMotorMoving(ssId, pVar)) {
        /* Get the current motor positions, post them */
        for (j=0; j<pVar->numAxes; j++) {
            pVar->motorCurrent[j] = pVar->epicsMotorPos[j];
            seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
        }
        epicsThreadSleep(POLL_INTERVAL);
    }
    for (j=0; j<pVar->numAxes; j++) {
        pVar->motorCurrent[j] = pVar->epicsMotorPos[j];
        seq_pvPut(ssId, pVar->motorCurrentIndex[j], 0);
    }
    return(0);
}
/* Function to ask the XPS for a socket this requires Socket.h */
static int getSocket(SS_ID ssId, struct UserVar *pVar)
{
    int sock = 0;
    
	/* With the asyn driver we pass the asyn port name not the ip */
	
	sock = TCP_ConnectToServer(pVar->asynPort,0,SOCKET_TIMEOUT);
	if (sock < 0) 
            printf(" Error TCP_ConnectToServer %i\n",sock);
	return (sock);
}
/* Function to setup the gathering, events and start the trajectory */
static int trajectoryExecute(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int j;
    char buffer[MAX_GATHERING_STRING];
    int eventId;
    
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " entry\n");
    }
    /* Configure Gathering */
    /* Reset gathering.  
     * This must be done because GatheringOneData just appends to in-memory list */    
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling GatheringReset(%d)\n", pVar->pollSocket);
    }
    status = GatheringReset(pVar->pollSocket);
    if (status != 0) {
        printf("Error performing GatheringReset, status=%d\n",status);
        return;
    }

    /* Write list of gathering parameters.
     * Note that there must be NUM_GATHERING_ITEMS per axis in this list. */
    strcpy(buffer, "");
    for (j=0; j<pVar->numAxes; j++) {
        strcat (buffer, pVar->axisName[j]);
        strcat (buffer, ".SetpointPosition;");
        strcat (buffer, pVar->axisName[j]);
        strcat (buffer, ".CurrentPosition;");
    }
    
    /* Define what is to be saved in the GatheringExternal.dat.  
     * 3 pieces of information per axis. */
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling GatheringConfigurationSet(%d, %d, %s)\n",
               pVar->pollSocket, pVar->numAxes*NUM_GATHERING_ITEMS, buffer);
    }
    status = GatheringConfigurationSet(pVar->pollSocket, 
                                       pVar->numAxes*NUM_GATHERING_ITEMS, buffer);
    if (status != 0)
            printf("Error performing GatheringConfigurationSet, status=%d, buffer=%p\n",
                   status, buffer);

    /* Define trajectory output pulses. 
     * startPulses and endPulses are defined as 1=first real element, need to add
     * 1 to each to skip the acceleration element.  
     * The XPS is told the element to stop outputting pulses, and it seems to stop
     * outputting at the start of that element.  So we need to have that element be
     * the decceleration endPulses is the element, which means adding another +1. */

    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling MultipleAxesPVTPulseOutputSet(%d, %s, %d, %d, %f)\n",
                                           pVar->pollSocket, pVar->groupName,
                                           pVar->startPulses+1, 
                                           pVar->endPulses+2, 
                                           pVar->pulsePeriod);
    }
    status = MultipleAxesPVTPulseOutputSet(pVar->pollSocket, pVar->groupName,
                                           pVar->startPulses+1, 
                                           pVar->endPulses+2, 
                                           pVar->pulsePeriod);

    /* Define trigger */
    sprintf(buffer, "Always;%s.PVT.TrajectoryPulse", pVar->groupName);
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling EventExtendedConfigurationTriggerSet(%d, %d, %s, %s, %s, %s, %s)\n",
                                                  pVar->pollSocket, 2, buffer, 
                                                  "", "", "", "");
    }
    if (status != 0) {
        printf("Error performing EventExtendedConfigurationTriggerSet, status=%d, buffer=%s\n", 
               status, buffer);
        return;
    }

    /* Define action */
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling EventExtendedConfigurationActionSet(%d, %d, %s, %s, %s, %s, %s)\n",
                                                 pVar->pollSocket, 1, 
                                                 "GatheringOneData", 
                                                 "", "", "", "");
    }
    status = EventExtendedConfigurationActionSet(pVar->pollSocket, 1, 
                                                 "GatheringOneData", 
                                                 "", "", "", "");
    if (status != 0) {
        printf("Error performing EventExtendedConfigurationActionSet, status=%d\n",
               status);
        return;
    }

    /* Start gathering */
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling EventExtendedStart(%d, %p)\n",
                                            pVar->pollSocket, &eventId);
    }
    status= EventExtendedStart(pVar->pollSocket, &eventId);
    if (status != 0) {
        printf("Error performing EventExtendedStart, status=%d\n",status);
        return;
    }

    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling MultipleAxesPVTExecution(%d, %s, %s, %d)\n",
                                      pVar->driveSocket, pVar->groupName,
                                      TRAJECTORY_FILE, 1);
    }
    status = MultipleAxesPVTExecution(pVar->driveSocket, pVar->groupName,
                                      TRAJECTORY_FILE, 1);
    /* status -27 means the trajectory was aborted */
    if ((status != 0) && (status != -27))
        printf("Error performing MultipleAxesPVTExecution, status=%d\n", status);
    
    /* Remove the event */
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling EventExtendedRemove(%d, %d)\n", pVar->pollSocket, eventId);
    }
    status = EventExtendedRemove(pVar->pollSocket, eventId);
    if (status != 0) {
        printf("Error performing ExtendedEventRemove, status=%d\n",status);
        return;
    }
        
    /* Save the gathered data to a file */    
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: trajectoryExecute:"
               " calling GatheringStopAndSave(%d)\n", pVar->pollSocket);
    }
    status = GatheringStopAndSave(pVar->pollSocket);
    
    /* status -30 means gathering not started i.e. aborted before the end of
       1 trajectory element */
    if ((status != 0) && (status != -30))
        printf("Error performing GatheringStopAndSave, status=%d\n", status);
    
    return;
}
/* Function to build install and verify trajectory */ 
static int buildAndVerify(SS_ID ssId, struct UserVar *pVar)
{
    FILE *trajFile;
    int i, j, status;
    int npoints;
    double trajStep;
    double trajVel;
    double P0, P1, T0, T1;
    int ftpSocket;
    char fileName[NAME_LEN];
    double distance;
    double maxVelocity[MAX_AXES], maxAcceleration[MAX_AXES];
    double minJerkTime[MAX_AXES], maxJerkTime[MAX_AXES];
    double preTimeMax, postTimeMax;
    double preVelocity[MAX_AXES], postVelocity[MAX_AXES];
    double time;
    
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: buildAndVerify:"
               " entry\n");
    }
    /* We create trajectories with an extra element at the beginning and at the end.
     * The distance and time of the first element is defined so that the motors will
     * accelerate from 0 to the velocity of the first "real" element at their 
     * maximum allowed acceleration.
     * Similarly, the distance and time of last element is defined so that the 
     * motors will decelerate from the velocity of the last "real" element to 0 
     * at the maximum allowed acceleration.

    /* Compute the velocity of each motor during the first real trajectory element, 
     * and the time required to reach this velocity. */
    preTimeMax = 0.;
    postTimeMax = 0.;
    /* Zero values since axes may not be used */
    for (j=0; j<pVar->numAxes; j++) {
        preVelocity[j] = 0.;
        postVelocity[j] = 0.;
    }
    for (j=0; j<pVar->numAxes; j++) {
        if (!pVar->moveAxis[j]) break;
        status = PositionerSGammaParametersGet(pVar->pollSocket, pVar->axisName[j], 
                                               &maxVelocity[j], &maxAcceleration[j],
                                               &minJerkTime[j], &maxJerkTime[j]);
        if (status != 0) {
            printf("Error calling positionerSGammaParametersSet, status=%d\n", 
                   status);
        }
        /* The calculation using maxAcceleration read from controller below
         * is "correct" but subject to roundoff errors when sending ASCII commands
         * to XPS.  Reduce acceleration 10% to account for this. */
        maxAcceleration[j] = 0.9 * maxAcceleration[j];
        if (pVar->moveMode == MOVE_MODE_RELATIVE) {
            distance = pVar->motorTrajectory[j][0];
        } else {
            distance = pVar->motorTrajectory[j][1] - pVar->motorTrajectory[j][0];
        }
        preVelocity[j] = distance/pVar->timeTrajectory[0];
        time = fabs(preVelocity[j]) / maxAcceleration[j];
        if (time > preTimeMax) preTimeMax = time;
        if (pVar->moveMode == MOVE_MODE_RELATIVE) {
            distance = pVar->motorTrajectory[j][pVar->nelements-1];
        } else {
            distance = pVar->motorTrajectory[j][pVar->nelements-1] - 
                       pVar->motorTrajectory[j][pVar->nelements-2];
        }
        postVelocity[j] = distance/pVar->timeTrajectory[pVar->nelements-1];
        time = fabs(postVelocity[j]) / maxAcceleration[j];
        if (time > postTimeMax) postTimeMax = time;
    }

    /* Compute the distance that each motor moves during its acceleration phase.
     * Only move it this far. */
    for (j=0; j<pVar->numAxes; j++) {
        pVar->preDistance[j] =  0.5 * preVelocity[j] *  preTimeMax; 
        pVar->postDistance[j] = 0.5 * postVelocity[j] * postTimeMax;
    }

    /* Create the trajectory file */
    trajFile =  fopen (TRAJECTORY_FILE, "w");

    /* Create the initial acceleration element */
    fprintf(trajFile,"%f", preTimeMax);
    for (j=0; j<pVar->numAxes; j++) 
        fprintf(trajFile,", %f, %f", pVar->preDistance[j], preVelocity[j]);
    fprintf(trajFile,"\n");
   
    /* The number of points in the file is nelements for MOVE_MODE_RELATIVE and
     * nelements-1 for other modes */
    if (pVar->moveMode == MOVE_MODE_RELATIVE) 
        npoints = pVar->nelements;
    else 
        npoints = pVar->nelements-1;
    for (i=0; i<npoints; i++) {
        T0 = pVar->timeTrajectory[i];
        T1 = pVar->timeTrajectory[i+1];
        for (j=0; j<pVar->numAxes; j++) {
            P0 = pVar->motorTrajectory[j][i];
            P1 = pVar->motorTrajectory[j][i+1];
        
            if (pVar->moveMode == MOVE_MODE_RELATIVE) {
                trajStep = pVar->motorTrajectory[j][i];
            } else {
                trajStep = pVar->motorTrajectory[j][i+1] - 
                           pVar->motorTrajectory[j][i];
            }
                
            /* Average either side of the point? */
            trajVel = trajStep / T0;
            if (!(pVar->moveAxis[j])) {
                trajStep = 0.0;    /* Axis turned off*/
                trajVel = 0.0;
            }
        
            if (j == 0) fprintf(trajFile,"%f", pVar->timeTrajectory[i]);
            fprintf(trajFile,", %f, %f",trajStep,trajVel);
            if (j == (pVar->numAxes-1)) fprintf(trajFile,"\n");
        }    
    }

    /* Create the final acceleration element. Final velocity must be 0. */
    fprintf(trajFile,"%f", postTimeMax);
    for (j=0; j<pVar->numAxes; j++) 
        fprintf(trajFile,", %f, %f", pVar->postDistance[j], 0.);
    fprintf(trajFile,"\n");
    fclose (trajFile);
    
    /* FTP the trajectory file from the local directory to the XPS */
    status = ftpConnect(pVar->xpsAddress, pVar->userName, pVar->password, &ftpSocket);
    if (status != 0) {
        printf("Error calling ftpConnect, status=%d\n", status);
        return;
    }
    status = ftpChangeDir(ftpSocket, TRAJECTORY_DIRECTORY);
    if (status != 0) {
        printf("Error calling ftpChangeDir, status=%d\n", status);
        return;
    }
    status = ftpStoreFile(ftpSocket, TRAJECTORY_FILE);
    if (status != 0) {
        printf("Error calling ftpStoreFile, status=%d\n", status);
        return;
    }
    status = ftpDisconnect(ftpSocket);
    if (status != 0) {
        printf("Error calling ftpDisconnect, status=%d\n", status);
        return;
    }
       
    /* Verify trajectory */
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: buildAndVerify:"
               " calling MultipleAxesPVTVerification(%d, %s, %s)\n", 
               pVar->pollSocket, pVar->groupName, TRAJECTORY_FILE);
    }
    status = MultipleAxesPVTVerification(pVar->pollSocket, pVar->groupName,
                                         TRAJECTORY_FILE);
     
    pVar->buildStatus = STATUS_FAILURE;
    if (status == 0) {
        strcpy(pVar->buildMessage, " ");
        pVar->buildStatus = STATUS_SUCCESS;
    }        
    else if (status == -69)
        strcpy(pVar->buildMessage, "Acceleration Too High");
    else if (status == -68)
        strcpy(pVar->buildMessage, "Velocity Too High");
    else if (status == -70)
        strcpy(pVar->buildMessage, "Final Velocity Non Zero");         
    else if (status == -75)
        strcpy(pVar->buildMessage, "Negative or Null Delta Time");         
    else
        sprintf(pVar->buildMessage, "Unknown trajectory verify error=%d", status);  
     
    /* Read dynamic parameters*/
     
    if (status == 0){
        pVar->buildStatus = STATUS_SUCCESS;
    }    
    if (1) {  /* We may need to test for status here */
        for (j=0; j<pVar->numAxes; j++) {
            status = MultipleAxesPVTVerificationResultGet(pVar->pollSocket,
                         pVar->axisName[j], fileName, &pVar->motorMinPos[j], &pVar->motorMaxPos[j], 
                         &pVar->motorMVA[j], &pVar->motorMAA[j]);
            if (status != 0) {
                printf("Error performing MultipleAxesPVTVerificationResultGet for axis %s, status=%d\n",
                       pVar->axisName[j], status); 
            }
        }
    } else {
        for (j=0; j<pVar->numAxes; j++) {
            pVar->motorMVA[j] = 0;
            pVar->motorMAA[j] = 0;     
        }
    }
           
    return; 
}

/* Function returns the current trajectory element*/ 
static int currentElement(SS_ID ssId, struct UserVar *pVar)
{
	int status;
	int number;
	char fileName[100];
	strcpy (fileName,pVar->trajFileName); 
	
	status = MultipleAxesPVTParametersGet(pVar->pollSocket,
		pVar->groupName,fileName,&number);

	return (number);
}

static int getGroupStatus(SS_ID ssId, struct UserVar *pVar)
{
    int status;
    int groupStatus;

    /* Read the current status of the group */

    status = GroupStatusGet(pVar->pollSocket,pVar->groupName,&groupStatus);
    if (status != 0)
        printf(" Error performing GroupStatusGet %i\n",status);
    return(groupStatus);
}

/* Function to load the GatheringExternal.dat file which was written
 * by the XPS after the trajectory was performed and read back the
 * actual motor positions and calculate the position errors */ 
static void readGathering(SS_ID ssId, struct UserVar *pVar)
{
    char vxWorksString[100];
    char buffer[100];
    FILE *trajFile;
    FILE *gatheringFile;
    int i,j;
    int nitems;
    double setpointPosition, actualPosition;
    int ftpSocket;
    int status;
    
    if (pVar->debugLevel > 0) {
        printf("XPS_trajectoryScan: readGathering:"
               " entry\n");
    }
    /* FTP the gathering file from the XPS to the local directory */
    status = ftpConnect(pVar->xpsAddress, pVar->userName, pVar->password, &ftpSocket);
    if (status != 0) {
        printf("Error calling ftpConnect, status=%d\n", status);
        return;
    }
    status = ftpChangeDir(ftpSocket, GATHERING_DIRECTORY);
    if (status != 0) {
        printf("Error calling ftpChangeDir, status=%d\n", status);
        return;
    }
    status = ftpRetrieveFile(ftpSocket, GATHERING_FILE);
    if (status != 0) {
        printf("Error calling ftpRetrieveFile, status=%d\n", status);
        return;
    }
    status = ftpDisconnect(ftpSocket);
    if (status != 0) {
        printf("Error calling ftpDisconnect, status=%d\n", status);
        return;
    }

    gatheringFile = fopen(GATHERING_FILE, "r");
    
    /* Read 1st 2 lines which only contain the axis names */
    for(i=0; i<2; ++i){
        fgets (buffer, 1000, gatheringFile);
	    /*printf("Line %i of GatheringEx = %s\n",i,buffer);*/
	}
    /* The next line is the motor positions at the start of the trajectory element
     * where pulses began to be output.  Skip this line because this pulse just
     * started the MCS counting, there are no data yet. */
    fgets (buffer, MAX_GATHERING_STRING, gatheringFile);
    
    for (i=0; i<pVar->npulses; i++) {
        /* There is a bug in the current V2.0.1 firmware that puts 2 lines in the 
         * Gathering.dat file when there should be 1.  
         * Skip the first one.  
         * THIS fgets NEEDS TO BE REMOVED WHEN V2.1 FIRMWARE IS RELEASED.
         */
        /* fgets (buffer, MAX_GATHERING_STRING, gatheringFile); */
        for (j=0; j<pVar->numAxes; j++) {
            /* Note the trailing white space in this format is required to make
             * fscanf read the newline */
            nitems = fscanf(gatheringFile, "%lf %lf ", 
                            &setpointPosition, &actualPosition);
            if (nitems != NUM_GATHERING_ITEMS) {
                printf("Error reading Gathering.dat file, nitems=%d, should be %d\n",
                       nitems, NUM_GATHERING_ITEMS);
                goto done;
            }
            pVar->motorReadbacks[j][i] = actualPosition;
            pVar->motorError[j][i] = actualPosition - setpointPosition;
        }
    }
    
    done:
    fclose (gatheringFile);
    pVar->nactual = i; 

    return (0);
}
/* Function aborts the trajectory/motion */ 
static int trajectoryAbort(SS_ID ssId, struct UserVar *pVar)
{
	int status;
	
	status = GroupMoveAbort(pVar->abortSocket,pVar->groupName);
	if (status != 0)
           printf(" Error performing GroupMoveAbort %i\n",status);
	   
	return (0);
}


}%















