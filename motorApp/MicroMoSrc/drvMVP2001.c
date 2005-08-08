/*
FILENAME...	drvMVP2001.c
USAGE...	Motor record driver level support for MicroMo
		MVP 2001 B02 (Linear, RS-485).

Version:	1.1.2.2.2.2
Modified By:	sluiter
Last Modified:	2004/05/18 16:19:39
*/

/*
 *        Original Author: Kevin Peterson
 *        Date: 08/27/2002
 *
 *
 *  Illinois Open Source License
 *  University of Illinois
 *  Open Source License
 *
 *
 *  Copyright (c) 2004,  UNICAT.  All rights reserved.
 *
 *
 *  Developed by:
 *
 *  UNICAT, Advanced Photon Source, Argonne National Laboratory
 *
 *  Frederick Seitz Materials Research Laboratory,
 *  University of Illinois at Urbana-Champaign
 *
 *  http://www.uni.aps.anl.gov
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the
 *  "Software"), to deal with the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *
 *  Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimers.
 *
 *
 *  Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimers in the
 *  documentation and/or other materials provided with the distribution.
 *
 *
 *  Neither the names of UNICAT, Frederick Seitz Materials Research 
 *  Laboratory, University of Illinois at Urbana-Champaign,
 *  nor the names of its contributors may be used to endorse or promote
 *  products derived from this Software without specific prior written
 *  permission.
 *
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 *  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 *  ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.
 *
 *
 * Modification Log:
 * -----------------
 * .01 08/27/02	 kmp  	copied from drvIM483PL.c (rev 1.7, mod .03) and
 *			customized for the MVP2001.
 * .02 08/27/02  kmp	changed message construction to allow for addresses
 * 			larger than 9.
 * .03 09/06/02  kmp	added an extra loop to motor_init() that sends the HO
 *			command a second time to ensure the position is set to
 *			zero.  Previously, saved positions would not be loaded
 *			if the controller was power-cycled.
 * .04 02/06/04  rls    Eliminate erroneous "Motor motion timeout ERROR".
 * .05 04/07/04  kmp    added an extra call to recv_mess during the controller- 
 *                      detection part of motor_init to clear "MVP2001 Ready"  
 *                      messages that some MVP2001s append to the first response
 *                      after power-up.
 * .06 05/03/04  kmp    added a task delay to send_message because MVME2700 processors
 *                      send commands to the MVP2001 faster than it can process them.
 *
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the MVP2001 must be powered-on when EPICS is first
	booted up.
    2 - The MVP2001 cannot be power cycled while EPICS is up and running.  The
	consequences are permanent communication loss with the MVP2001 until
	EPICS is rebooted.
    3 - Translation between the MVP2001 and the ACCL/BACC fields is not obvious.	
*/
/*
MORE DESIGN LIMITATIONS
    1 - For the most part the standard terminology (card-signal-axis) has been
	used here so that this code resembles other drivers.  Unfortunately, 
	the terminology is not the best for this controller.  The MVP2001 is a
	single-axis, RS-485-daisy-chainable, DC controller.  The following 
	equations succinctly illustrate the relationship between the physical 
	setup and the standard terminology:
	card = chain of MVP2001 controllers
	signal = axis = one of the MVP2001 controllers on a chain
    2 - Strtol and strtoul have been switched in the vxWorks that KMP used 
    	at the time that this was being written.  If your vxWorks functions
	behave correctly, then they will have to be switched in the code.
    3 - Factors that currently limit the number of controllers on one chain:
    	A.  MVP2001 addresses
	  The MVP2001 can have an address of 1-64 for serial communication.
	B.  RS-485 communication degradation
	  There is a practical limit to how many controllers can be on one chain
	C.  The motor_info array of the controller structure in motordrvCom.h
	  For a chain to work correctly, there needs to be one element in the 
	  motor_info array for every controller on the chain.  The limit is 
	  set by the constant MAX_AXIS, which is defined in motor.h.  The end
	  result is that the number of controllers is limited by the motor 
	  record.  The current maximum number of controllers is 10.  
*/

#include	<vxWorks.h>
#include	<stdioLib.h>
#include	<sysLib.h>
#include	<string.h>
#include	<taskLib.h>
#include	<math.h>
#include        <rngLib.h>
#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<fast_lock.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<drvSup.h>
#include        <errMdef.h>
#include	<logLib.h>
#include	<osiSleep.h>
#include	<ctype.h>

#include	"motor.h"
#include	"drvMVP2001.h"
#include        "serialIO.h"

#define STATIC static
#define INPUT_TERMINATOR  '\n'

#define MVP2001_NUM_CARDS	8
#define BUFF_SIZE 20		/* Maximum length of string to/from MVP2001 */

/*----------------debugging-----------------*/
#ifdef	DEBUG
    #define Debug(l, f, args...) { if(l<=drvMVP2001debug) printf(f,## args); }
    volatile int drvMVP2001debug = 0;
#else
    #define Debug(l, f, args...)
#endif

/* --- Local data. --- */
int MVP2001_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC int send_mess(int card, char const *com, char c);
STATIC int set_status(int card, int signal);
STATIC long report(int level);
STATIC long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MVP2001_access =
{
    motor_init,
    motor_send,
    motor_free,
    motor_card_info,
    motor_axis_info,
    &mess_queue,
    &queue_lock,
    &free_list,
    &freelist_lock,
    &motor_sem,
    &motor_state,
    &total_cards,
    &any_motor_in_motion,
    send_mess,
    recv_mess,
    set_status,
    query_done,
    NULL,
    &initialized,
    NULL
};

struct
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvMVP2001 = {2, report, init};


/*********************************************************
 * Print out driver status report
 *********************************************************/
STATIC long report(int level)
{
    int card;

    if (MVP2001_num_cards <=0)
	printf("    No MVP2001 CHAINS configured.\n");
    else
    {
	for (card = 0; card < MVP2001_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    MVP2001 controller chain %d connection failed.\n", card);
	    else
	    {
		struct MVPcontroller *cntrl;
		cntrl = (struct MVPcontroller *) brdptr->DevicePrivate;
		switch (cntrl->port_type)
		{
		    case RS232_PORT:							  
		    	printf("    MVP2001 controller chain %d port type = RS-232, id: %s \n", 
		    	       card,							  
		    	       brdptr->ident);  					  
		    	break;  							  
		    default:								  
		    	printf("    MVP2001 controller chain %d port type = Unknown, id: %s \n",
		    	       card,							  
		    	       brdptr->ident);  					  
		    	break;  							  
		}
	    }
	}
    }
    return (0);
}


STATIC long init()
{
    /* initialize all hardware and software */
    motor_init();
    
    /* Check for setup */
    if (MVP2001_num_cards <= 0)
    {
	Debug(1, "init(): MVP2001 driver disabled. MVP2001Setup() missing \
		from startup script.\n");
    }

    return ((long) 0);
}


STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/********************************************************************************
*										*
* FUNCTION NAME: set_status							*
*										*
* LOGIC:									*
*   Initialize.									*
*   Send "Moving Status" query.							*
*   Read response.								*
*   IF normal response to query.						*
*	Set communication status to NORMAL.					*
*   ELSE									*
*	IF communication status is NORMAL.					*
*	    Set communication status to RETRY.					*
*	    NORMAL EXIT.							*
*	ELSE									*
*	    Set communication status error.					*
*	    ERROR EXIT.								*
*	ENDIF									*
*   ENDIF									*
*										*
*   IF "Moving Status" indicates any motion (i.e. status != 0).			*
*	Clear "Done Moving" status bit.						*
*   ELSE									*
*	Set "Done Moving" status bit.						*
*   ENDIF									*
*										*
*   										*
********************************************************************************/

/* 
 * When the motor record calls set_status it passes it card and signal values
 * that are found in the mvpMotors template (or the OUT field of the M.R.)
 */

STATIC int set_status(int card, int signal)
{
    struct MVPcontroller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    char statusStr[BUFF_SIZE], positionStr[BUFF_SIZE];
    int rtn_state;
    epicsInt32 motorData;
    MOTOR_STATUS mstat;
    BOOLEAN plusdir, ls_active = OFF;

    cntrl = (struct MVPcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    statusStr[0] = positionStr[0] = buff[0] = '\0';

    sprintf(buff, "%d ST", (signal + 1));
    send_mess(card, buff, NULL);
    rtn_state = recv_mess(card, buff, 1);
    if (rtn_state > 0)
    {
	cntrl->status = NORMAL;
	motor_info->status &= ~CNTRL_COMM_ERR;
    }
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    return(0);
	}
	else
	{
	    cntrl->status = COMM_ERR;
	    motor_info->status |= CNTRL_COMM_ERR;
	    motor_info->status |= RA_PROBLEM;
	    return(1);
	}
    }

    /*
     * Parse status string
     * Status string format: 0001 FFFF
     * Skip to status substring for this motor, convert from hex to int
     */
    strncat(statusStr, &buff[5], 4);
    mstat.All = strtoul(statusStr, NULL, 16);
    buff[0] = '\0';

    if (mstat.Bits.inMotion == OFF)
	motor_info->status |= RA_DONE;
    else
	motor_info->status &= ~RA_DONE;

    sprintf(buff, "%d POS", (signal + 1));
    send_mess(card, buff, NULL);
    recv_mess(card, buff, 1);

    /* 
     * Parse motor position
     * Position string format: 0001 FFFFFFFF
     * Skip to position substring for this motor, convert from hex to int
     */
    strncat(positionStr, &buff[5], 8);
    motorData = (epicsInt32) strtoul(positionStr, NULL, 16);
    buff[0] = '\0';    

    /*
     * Set direction by comparing positions since the MVP2001 
     * does not have a direction bit.
     */
    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
	epicsInt32 newposition;

	newposition = NINT(motorData);
	if (newposition >= motor_info->position)
	    motor_info->status |= RA_DIRECTION;
	else
	    motor_info->status &= ~RA_DIRECTION;
	motor_info->position = newposition;
	motor_info->no_motion_count = 0;
    }

    plusdir = (motor_info->status & RA_DIRECTION) ? ON : OFF;

    /* Set limit switch error indicators. */
    if (mstat.Bits.plusLS == OFF)
	motor_info->status &= ~RA_PLUS_LS;
    else
    {
	motor_info->status |= RA_PLUS_LS;
	if (plusdir == ON)
	    ls_active = ON;
    }

    if (mstat.Bits.minusLS == OFF)
	motor_info->status &= ~RA_MINUS_LS;
    else
    {
	motor_info->status |= RA_MINUS_LS;
	if (plusdir == OFF)
	    ls_active = ON;
    }

    /* The MVP2001 doesn't have a home feature */
    motor_info->status &= ~RA_HOME;

    /* !!! Assume no closed-looped control!!!*/
    motor_info->status &= ~EA_POSITION;

    /* encoder status */
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;

    if (motor_state[card]->motor_info[signal].encoder_present == NO)
	motor_info->encoder_position = 0;
    else
    {
	/*
	 * There is not a seperate call for "encoder_position" as every call 
	 * for the position of the DC motor reads the encoder.
	 */ 
	motor_info->encoder_position = motorData;
    }

    motor_info->status &= ~RA_PROBLEM;

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!(motor_info->status & RA_DIRECTION))
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == ON ||
     (motor_info->status & (RA_DONE | RA_PROBLEM))) ? 1 : 0;

    /* Test for post-move string. */
    if ((motor_info->status & RA_DONE || ls_active == ON) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
	strcpy(buff, nodeptr->postmsgptr);
	send_mess(card, buff, NULL);
	nodeptr->postmsgptr = NULL;
    }

    return(rtn_state);
}


/*****************************************************/
/* send a message to the MVP2001 board		     */
/* send_mess()			                     */
/*****************************************************/
STATIC int send_mess(int card, char const *com, char inchar)
{
    char local_buff[MAX_MSG_SIZE];
    struct MVPcontroller *cntrl;
    int size;

    size = strlen(com);
    
    /* This task delay is needed because the MVME2700 sends commands 
       to the MVP2001 faster than the MVP2001 can accept them */
    taskDelay(2);
    
    if (size > MAX_MSG_SIZE)
    {
	logMsg((char *) "drvMVP2001.c:send_mess(); message size violation.\n",
	       0, 0, 0, 0, 0, 0);
	return(-1);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(0);

    if (!motor_state[card])
    {
	logMsg((char *) "drvMVP2001.c:send_mess() - invalid card #%d\n", card,
	       0, 0, 0, 0, 0);
	return(-1);
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcpy(local_buff, com);
    strcat(local_buff, "\r");

    if (inchar != (char) NULL)
	local_buff[0] = inchar;	    /* put in axis */

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct MVPcontroller *) motor_state[card]->DevicePrivate;
    serialIOSend(cntrl->serialInfo, local_buff, strlen(local_buff), SERIAL_TIMEOUT);

    return(0);
}


/*****************************************************/
/* receive a message from the MVP2001 board          */
/* recv_mess()			                     */
/*****************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    struct MVPcontroller *cntrl;
    char temp[BUFF_SIZE];
    int timeout;
    int len=0, lenTemp=0;

    /* Check that card exists */
    if (!motor_state[card])
	return (-1);

    cntrl = (struct MVPcontroller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= SERIAL_TIMEOUT;

    lenTemp = serialIORecv(cntrl->serialInfo, temp, BUFF_SIZE, INPUT_TERMINATOR, timeout);
    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE, INPUT_TERMINATOR, timeout);
    Debug(5, "bytes: 1st call: %d\t2nd call: %d\n", lenTemp, len);
    
    if (len == 0)
	com[0] = '\0';
    else
	com[len - 1] = '\0';

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (len);
}


/*****************************************************/
/* Setup system configuration                        */
/* MVP2001Setup()                                    */
/*****************************************************/
int MVP2001Setup(int num_cards,	/* number of CHAINS of controllers 	*/
	    int num_channels,	/* NOT Used. 				*/
	    int scan_rate)	/* polling rate  (Min=1Hz, max=60Hz) 	*/
{
    if (num_cards < 1 || num_cards > MVP2001_NUM_CARDS)
	MVP2001_num_cards = MVP2001_NUM_CARDS;
    else
	MVP2001_num_cards = num_cards;

    /* Set motor polling task rate */
    /* motor_scan_rate is actually the polling period in clock tics */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before MVP2001Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) calloc(MVP2001_num_cards,
						sizeof(struct controller *));

    return (0);
}


/*******************************************************
*  Configure a CHAIN of controllers		       *
* 						       *
*  Note: Addresses of controllers on a chain must      *
* 	 begin at 1 and follow sequentially.	       *
* 						       *
*  MVP2001Config()				       *
********************************************************/
int MVP2001Config(int card,	/* CHAIN being configured 		*/
            int port_type,      /* 1:RS232_PORT 			*/
	    int addr1,          /* VME Card 				*/
            int addr2)          /* Serial port name (SerialServer task) */
{
    struct MVPcontroller *cntrl;

    if (card < 0 || card >= MVP2001_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) calloc(1, sizeof(struct controller));
    motor_state[card]->DevicePrivate = calloc(1, sizeof(struct MVPcontroller));
    cntrl = (struct MVPcontroller *) motor_state[card]->DevicePrivate;

    switch (port_type)
    {
    case GPIB_PORT:
	/* GPIB not possible with MVP2001 */
        break;
    case RS232_PORT:
        cntrl->port_type = port_type;
        cntrl->serial_card = addr1;
        strcpy(cntrl->serial_task, (char *) addr2);
        break;
    /* DeviceNet not yet implemented */
    default:
        return (ERROR);
    }
    return (0);
}


/*****************************************************/
/* initialize all software and hardware		     */
/* motor_init()			                     */
/*****************************************************/
STATIC int motor_init()
{
    struct controller *brdptr;
    struct MVPcontroller *cntrl;
    int card_index, motor_index, arg3, arg4;
    char buff[BUFF_SIZE], limitStr[BUFF_SIZE];
    int total_axis = 0;
    int status;
    BOOLEAN errind;

    buff[0] = limitStr[0] = '\0';

    initialized = ON;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (MVP2001_num_cards <= 0)
	return (ERROR);

    for (card_index = 0; card_index < MVP2001_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->ident[0] = NULL;	/* No controller identification message. */
	brdptr->cmnd_response = OFF; /* The MVP doesn't respond to every command */
	total_cards = card_index + 1;
	cntrl = (struct MVPcontroller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	errind = OFF;
	cntrl->serialInfo = serialIOInit(cntrl->serial_card,
					 cntrl->serial_task);
	if (cntrl->serialInfo == NULL)
	    errind = ON;

	if (errind == OFF)
	{
	    /* Send a message to the board, see if it exists */    
	    for (total_axis = 0; total_axis < MAX_AXIS; total_axis++)
	    {
	    	/* flush any junk at input port - should not be any data available */
	    	do
		    recv_mess(card_index, buff, FLUSH);
	    	while (strlen(buff) != 0);	
	    
		sprintf(buff, "%d ST", (total_axis + 1));
		send_mess(card_index, buff, NULL);
		status = recv_mess(card_index, buff, 1);

	    	/* Some versions of MVP2001 respond with a ready message after the
                first response after powerup.  Flush this message. */
	        recv_mess(card_index, buff, 1);

		if (status <= 0)
		    break;
	    }
	    brdptr->total_axis = total_axis;
	    Debug(5, "brdptr->total_axis (number of controllers on chain %d) = %d\n", 
	    		card_index, brdptr->total_axis);
	}

	if (errind == OFF && total_axis > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		/* stop and initialize the controller */
		sprintf(buff, "%d V 0", (motor_index + 1));
		send_mess(card_index, buff, NULL);
		sprintf(buff, "%d HO", (motor_index + 1));
		send_mess(card_index, buff, NULL);
		sprintf(buff, "%d EN", (motor_index + 1));
		send_mess(card_index, buff, NULL);

		motor_info->status = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;

		/* no encoder support for correct DC controller interaction */
		motor_info->encoder_present = NO;
		motor_info->status &= ~EA_PRESENT;

		/* MVP2001 has PID capabilities */
		motor_info->pid_present = YES;	       
		motor_info->status |= GAIN_SUPPORT;    

		limitStr[0] = '\0';
		/* Determine low limit */
		sprintf(buff, "%d LL -", (motor_index + 1));
		send_mess(card_index, buff, NULL);
		recv_mess(card_index, buff, 1);
		strncat(limitStr, &buff[5], 8);
		motor_info->low_limit = (epicsInt32) strtoul(limitStr, NULL, 16);

		limitStr[0] = '\0';
		/* Determine high limit */
		sprintf(buff, "%d LL", (motor_index + 1));
		send_mess(card_index, buff, NULL);
		recv_mess(card_index, buff, 1);
		strncat(limitStr, &buff[5], 8);
		motor_info->high_limit = (epicsInt32) strtoul(limitStr, NULL, 16);
	    }

	    /*
	     * Ensure that the position is correctly set to zero so that auto_sr
	     * loads the saved positions.  The task delay is necessary because
	     * sending the HO command too soon after the EN command results in
	     * reading back a position within ten encoder pulses away from zero.
	     */
	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		taskDelay((int)(.2 * sysClkRateGet()));

		sprintf(buff, "%d HO", (motor_index + 1));
		send_mess(card_index, buff, NULL);

		set_status(card_index, motor_index);  /* Read status of each motor */		
	    }
	}
	else
	    motor_state[card_index] = (struct controller *) NULL;
    }

    motor_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
    any_motor_in_motion = 0;

    FASTLOCKINIT(&queue_lock);
    FASTLOCK(&queue_lock);
    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&queue_lock);

    FASTLOCKINIT(&freelist_lock);
    FASTLOCK(&freelist_lock);
    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;
    FASTUNLOCK(&freelist_lock);

    if (sizeof(int) >= sizeof(char *))
    {
	arg3 = (int) (&MVP2001_access);
	arg4 = 0;
    }
    else
    {
	arg3 = (int) ((long) &MVP2001_access >> 16);
	arg4 = (int) ((long) &MVP2001_access & 0xFFFF);
    }
    taskSpawn((char *) "MVP2001_motor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}

