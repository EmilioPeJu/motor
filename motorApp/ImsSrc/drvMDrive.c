/*
FILENAME...	drvMDrive.c
USAGE...	Motor record driver level support for Intelligent Motion
		Systems, Inc. IM483(I/IE).

Version:	1.1.2.2.2.4
Modified By:	sluiter
Last Modified:	2004/05/03 15:55:20
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 03/21/03
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the Regents of the University of California,
 *      and the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contracts:
 *      (W-7405-ENG-36) at the Los Alamos National Laboratory,
 *      and (W-31-109-ENG-38) at Argonne National Laboratory.
 *
 *      Initial development by:
 *	      The Controls and Automation Group (AT-8)
 *	      Ground Test Accelerator
 *	      Accelerator Technology Division
 *	      Los Alamos National Laboratory
 *
 *      Co-developed with
 *	      The Controls and Computing Group
 *	      Accelerator Systems Division
 *	      Advanced Photon Source
 *	      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 * .01 03/21/03 rls copied from drvIM483PL.c
 * .02 12-17-03 rls Fixed DLY; removed ReadbackDelay variable.
 * .03 02-06-04 rls Eliminate erroneous "Motor motion timeout ERROR".
 * .04 03-22-04 rls Previous driver releases not working.  Fixed by adding
 *                  Kevin Peterson's eat_garbage() function.  Added support
 *                  for encoder detection via "ident".
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the MDrive must be powered-on when EPICS is first
	booted up.
    2 - The MDrive cannot be power cycled while EPICS is up and running.  The
	consequences are permanent communication lose with the MDrive until
	EPICS is rebooted.
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
#include        <errMdef.h>
#include	<logLib.h>
#include	<osiSleep.h>
#include	<ctype.h>

#include	"motor.h"
#include	"drvIM483.h"
#include        "gpibIO.h"
#include        "serialIO.h"

#define INPUT_TERMINATOR  '\n'

/* Read Limit Status response values. */
#define L_ALIMIT	1
#define L_BLIMIT	2
#define L_BOTH_LIMITS	3


#define MDrive_NUM_CARDS	8
#define MAX_AXES		8
#define BUFF_SIZE 13		/* Maximum length of string to/from MDrive */

/*----------------debugging-----------------*/
#ifdef	DEBUG
    volatile int drvMDrivedebug = 0;
    #define Debug(l, f, args...) { if(l<=drvMDrivedebug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

/* --- Local data. --- */
int MDrive_num_cards = 0;
static char MDrive_axis[8] = {'1', '2', '3', '4', '5', '6', '7', '8'};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
static int eat_garbage(int, char *, int);
static int recv_mess(int, char *, int);
static int send_mess(int card, char const *com, char c);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table MDrive_access =
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
    eat_garbage,
    set_status,
    query_done,
    NULL,
    &initialized,
    MDrive_axis
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
} drvMDrive = {2, report, init};


/* Single Inputs - response from PR IN command */
typedef union
{
    epicsUInt8 All;
    struct
    {
	unsigned int na7:1;
	unsigned int na6:1;
	unsigned int na5:1;
	unsigned int na4:1;
	unsigned int na3:1;
	unsigned int homels:1;		/* Home limit switch.  */
	unsigned int ls_plus:1;		/* Plus limit switch.  */
	unsigned int ls_minus:1;	/* Minus limit switch. */
    } Bits;
} SINPUTS;

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (MDrive_num_cards <=0)
	printf("    No MDrive controllers configured.\n");
    else
    {
	for (card = 0; card < MDrive_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    MDrive controller %d connection failed.\n", card);
	    else
	    {
		struct IM483controller *cntrl;

		cntrl = (struct IM483controller *) brdptr->DevicePrivate;
		switch (cntrl->port_type)
		{
		case RS232_PORT: 
		    printf("    MDrive controller %d port type = RS-232, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		default:
		    printf("    MDrive controller %d port type = Unknown, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		}
	    }
	}
    }
    return (0);
}


static long init()
{
   /* 
    * We cannot call motor_init() here, because that function can do GPIB I/O,
    * and hence requires that the drvGPIB have already been initialized.
    * That cannot be guaranteed, so we need to call motor_init from device
    * support
    */
    /* Check for setup */
    if (MDrive_num_cards <= 0)
    {
	Debug(1, "init(): MDrive driver disabled. MDriveSetup() missing from startup script.\n");
    }
    return ((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
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

static int set_status(int card, int signal)
{
    struct IM483controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    int status;
    int rtn_state;
    double motorData;
    SINPUTS inputs;
    BOOLEAN plusdir, ls_active = OFF;

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    send_mess(card, "?PR MV", MDrive_axis[signal]);
    eat_garbage(card, buff, 1);
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

    status = atoi(buff);

    if (status != 0)
	motor_info->status &= ~RA_DONE;
    else
	motor_info->status |= RA_DONE;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    send_mess(card, "?PR P", MDrive_axis[signal]);
    eat_garbage(card, buff, 1);
    recv_mess(card, buff, 1);

    motorData = atof(buff);

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

    send_mess(card, "?PR IN", MDrive_axis[signal]);
    eat_garbage(card, buff, 1);
    recv_mess(card, buff, 1);
    inputs.All = atoi(buff);

    /* Set limit switch error indicators. */
    if (inputs.Bits.ls_plus == 0)
    {
	motor_info->status |= RA_PLUS_LS;
	if (plusdir == ON)
	    ls_active = ON;
    }
    else
	motor_info->status &= ~RA_PLUS_LS;

    if (inputs.Bits.ls_minus == 0)
    {
	motor_info->status |= RA_MINUS_LS;
	if (plusdir == OFF)
	    ls_active = ON;
    }
    else
	motor_info->status &= ~RA_MINUS_LS;


    if (inputs.Bits.homels == 0)
	motor_info->status |= RA_HOME;
    else
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
	send_mess(card, "?PR C2", MDrive_axis[signal]);
	eat_garbage(card, buff, 1);
	recv_mess(card, buff, 1);
	motorData = atof(buff);
	motor_info->encoder_position = (int32_t) motorData;
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
	send_mess(card, buff, MDrive_axis[signal]);
	nodeptr->postmsgptr = NULL;
    }

    return(rtn_state);
}


/*****************************************************/
/* send a message to the MDrive board		     */
/* send_mess()			                     */
/*****************************************************/
static int send_mess(int card, char const *com, char inchar)
{
    char local_buff[MAX_MSG_SIZE];
    struct IM483controller *cntrl;
    int size;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	logMsg((char *) "drvMDrive.c:send_mess(); message size violation.\n",
	       0, 0, 0, 0, 0, 0);
	return(-1);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(0);

    if (!motor_state[card])
    {
	logMsg((char *) "drvMDrive.c:send_mess() - invalid card #%d\n", card,
	       0, 0, 0, 0, 0);
	return(-1);
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcpy(local_buff, com);
    strcat(local_buff, "\n");

    if (inchar != (char) NULL)
	local_buff[0] = inchar;	    /* put in axis */

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;
    serialIOSend(cntrl->serialInfo, local_buff, strlen(local_buff), SERIAL_TIMEOUT);

    return(0);
}


/*****************************************************/
/* eat garbage characters from the MDrive board      */
/* eat_garbage()			             */
/*****************************************************/
static int eat_garbage(int card, char *com, int flag)
{
    struct IM483controller *cntrl;
    char localbuf[BUFF_SIZE];
    int timeout;
    int len=0;

    /* Check that card exists */
    if (!motor_state[card])
	return (-1);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= SERIAL_TIMEOUT;

    /* Get the response. */      
    len = serialIORecv(cntrl->serialInfo, localbuf, BUFF_SIZE, INPUT_TERMINATOR, timeout);

    Debug(2, "eat_garbage(): len = %i\n", len);
    if (len != 2)
        Debug(2, "eat_garbage(): localbuf = \"%s\"\n", localbuf);

    com[0] = '\0';

    /*Debug(2, "eat_garbage(): message = \"%s\"\n", com);*/
    return (len);
}


/*****************************************************/
/* receive a message from the MDrive board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct IM483controller *cntrl;
    char localbuf[BUFF_SIZE];
    int timeout;
    int len=0;

    /* Check that card exists */
    if (!motor_state[card])
	return (-1);

    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= SERIAL_TIMEOUT;

    /* Get the response. */      
    len = serialIORecv(cntrl->serialInfo, localbuf, BUFF_SIZE, INPUT_TERMINATOR, timeout);

    if (len == 0)
	com[0] = '\0';
    else
    {
	localbuf[len - 2] = '\0'; /* Strip off trailing "<CR><LF>". */
	strcpy(com, localbuf);
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return (len);
}


/*****************************************************/
/* Setup system configuration                        */
/* MDriveSetup()                                     */
/*****************************************************/
int MDriveSetup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > MDrive_NUM_CARDS)
	MDrive_num_cards = MDrive_NUM_CARDS;
    else
	MDrive_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before IM483Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(MDrive_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < MDrive_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return (0);
}


/*****************************************************/
/* Configure a controller                            */
/* MDriveConfig()                                    */
/*****************************************************/
int MDriveConfig(int card,	/* card being configured */
            int port_type,      /* GPIB_PORT or RS232_PORT */
	    int addr1,          /* = link for GPIB or hideos_card for RS-232 */
            int addr2)          /* GPIB address or hideos_task */
{
    struct IM483controller *cntrl;

    if (card < 0 || card >= MDrive_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct IM483controller));
    cntrl = (struct IM483controller *) motor_state[card]->DevicePrivate;

    switch (port_type)
    {
    case GPIB_PORT:
        cntrl->port_type = port_type;
        cntrl->gpib_link = addr1;
        cntrl->gpib_address = addr2;
        break;
    case RS232_PORT:
        cntrl->port_type = port_type;
        cntrl->serial_card = addr1;
        strcpy(cntrl->serial_task, (char *) addr2);
        break;
    default:
        return (ERROR);
    }
    return (0);
}


/*****************************************************/
/* initialize all software and hardware		     */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()			                     */
/*****************************************************/
static int motor_init()
{
    struct controller *brdptr;
    struct IM483controller *cntrl;
    int card_index, motor_index, arg3, arg4;
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status;
    BOOLEAN errind;

    initialized = ON;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (MDrive_num_cards <= 0)
	return (ERROR);

    for (card_index = 0; card_index < MDrive_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->ident[0] = NULL;	/* No controller identification message. */
	brdptr->cmnd_response = ON;
	total_cards = card_index + 1;
	cntrl = (struct IM483controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	errind = OFF;
	cntrl->serialInfo = serialIOInit(cntrl->serial_card,
					 cntrl->serial_task);
	if (cntrl->serialInfo == NULL)
	    errind = ON;

	if (errind == OFF)
	{
	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    do
		recv_mess(card_index, buff, FLUSH);
	    while (strlen(buff) != 0);
    
	    for (total_axis = 0; total_axis < MAX_AXES; total_axis++)
	    {
		int retry = 0;
		
		/* Try 3 times to connect to controller. */
		do
		{
		    send_mess(card_index, "?PR VR", MDrive_axis[total_axis]);
                    eat_garbage(card_index, buff, 1);
		    status = recv_mess(card_index, buff, 1);
		    retry++;
		} while (status == 0 && retry < 3);

		if (status <= 0)
		    break;
		else if (total_axis == 0)
		    strcpy(brdptr->ident, buff);
	    }
	    brdptr->total_axis = total_axis;
	}

	if (errind == OFF && total_axis > 0)
	{
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* Assume no encoder support. */
		motor_info->encoder_present = NO;

                /* Determine if encoder present based on open/closed loop mode. */
		if (brdptr->ident[strlen(brdptr->ident) - 1] == 'E')
		{
		    motor_info->pid_present = YES;
		    motor_info->status |= GAIN_SUPPORT;
		    motor_info->encoder_present = YES;
		    motor_info->status |= EA_PRESENT;
		}

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
	arg3 = (int) (&MDrive_access);
	arg4 = 0;
    }
    else
    {
	arg3 = (int) ((long) &MDrive_access >> 16);
	arg4 = (int) ((long) &MDrive_access & 0xFFFF);
    }
    taskSpawn((char *) "MDrive_motor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}

