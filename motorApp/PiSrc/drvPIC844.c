/*
FILENAME...	drvPIC844.c
USAGE...	Motor record driver level support for Physik Instrumente (PI)
		GmbH & Co. C-844 motor controller.

Version:	1.1.2.1.2.3
Modified By:	sluiter
Last Modified:	2004/09/10 16:01:25
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 12/17/03
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
 * .01 12/17/03 rls - copied from drvIM483PL.c
 * .02 02-06-04 rls - Eliminate erroneous "Motor motion timeout ERROR".
 * .03 09-09-04 rls - Retry on initial comm. (PIC844 comm. locks up when IOC
 *                    is power cycled).
 */

/*
DESIGN LIMITATIONS...
    1 - Like all controllers, the PIC844 must be powered-on when EPICS is first
	booted up.
    2 - The PIC844 cannot be power cycled while EPICS is up and running.  The
	consequences are permanent communication lose with the PIC844 until
	EPICS is rebooted.
    3 - Like the Newport MM3000, the PIC844's position can only be set to zero.
    4 - The PIC844 uses an internal look-up table for acceleration/deceleration.
	Translation between the PIC844 and the ACCL/BACC fields is not obvious.
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
#include	"drvPI.h"
#include        "gpibIO.h"
#include        "serialIO.h"

#define INPUT_TERMINATOR  '\n'
#define GET_IDENT "*IDN?"

#define PIC844_NUM_CARDS	8
#define MAX_AXES		8
#define BUFF_SIZE 100		/* Maximum length of string to/from PIC844 */

/*----------------debugging-----------------*/
#ifdef	DEBUG
    volatile int drvPIC844debug = 0;
    #define Debug(l, f, args...) { if(l<=drvPIC844debug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

/* --- Local data. --- */
int PIC844_num_cards = 0;
static char PIC844_axis[4] = {'1', '2', '3', '4'};

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/*----------------functions-----------------*/
static int recv_mess(int, char *, int);
static int send_mess(int card, char const *com, char c);
static int set_status(int card, int signal);
static long report(int level);
static long init();
static int motor_init();
static void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PIC844_access =
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
    PIC844_axis
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
} drvPIC844 = {2, report, init};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
    int card;

    if (PIC844_num_cards <=0)
	printf("    No PIC844 controllers configured.\n");
    else
    {
	for (card = 0; card < PIC844_num_cards; card++)
	{
	    struct controller *brdptr = motor_state[card];

	    if (brdptr == NULL)
		printf("    PIC844 controller %d connection failed.\n", card);
	    else
	    {
		struct PIC844controller *cntrl;

		cntrl = (struct PIC844controller *) brdptr->DevicePrivate;
		switch (cntrl->port_type)
		{
		case RS232_PORT: 
		    printf("    PIC844 controller %d port type = RS-232, id: %s \n", 
			   card, 
			   brdptr->ident);
		    break;
		default:
		    printf("    PIC844 controller %d port type = Unknown, id: %s \n", 
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
    if (PIC844_num_cards <= 0)
    {
	Debug(1, "init(): PIC844 driver disabled. PIC844Setup() missing from startup script.\n");
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
    struct PIC844controller *cntrl;
    struct mess_node *nodeptr;
    register struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE];
    C844_Cond_Reg status;
    int rtn_state, charcnt;
    double motorData;
    BOOLEAN plusdir, ls_active = OFF, inmotion, plusLS, minusLS;

    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    send_mess(card, "AXIS:STAT?", PIC844_axis[signal]);
    recv_mess(card, buff, 1);

    if (strcmp(buff, "ON") == 0)
	motor_info->status |= EA_POSITION;
    else if (strcmp(buff, "OFF") == 0)
	motor_info->status &= ~EA_POSITION;
    else
    {
	if (cntrl->status == NORMAL)
	{
	    cntrl->status = RETRY;
	    rtn_state = 0;
	}
	else
	{
	    cntrl->status = COMM_ERR;
	    motor_info->status |= CNTRL_COMM_ERR;
	    motor_info->status |= RA_PROBLEM;
	    rtn_state = 1;
	}
	goto exit;
    }
    
    cntrl->status = NORMAL;
    motor_info->status &= ~CNTRL_COMM_ERR;

    send_mess(card, "MOT:COND?", (char) NULL);
    recv_mess(card, buff, 1);

    status.All = atoi(&buff[0]);
    switch(signal)
    {
	case 0:
	    inmotion = status.Bits.axis1IM;
	    break;
	case 1:
	    inmotion = status.Bits.axis2IM;
	    break;
	case 2:
	    inmotion = status.Bits.axis3IM;
	    break;
	case 3:
	    inmotion = status.Bits.axis4IM;
	default:
	    rtn_state = 1;
	    goto exit;
    }

    if (inmotion == YES)
	motor_info->status &= ~RA_DONE;
    else
	motor_info->status |= RA_DONE;

    /* 
     * Parse motor position
     * Position string format: 1TP5.012,2TP1.123,3TP-100.567,...
     * Skip to substring for this motor, convert to double
     */

    send_mess(card, "CURR:TPOS?", (char) NULL);
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

    switch(signal)
    {
	case 0:
	    plusLS  = status.Bits.axis1PL;
	    minusLS = status.Bits.axis1ML;
	    break;
	case 1:
	    plusLS  = status.Bits.axis2PL;
	    minusLS = status.Bits.axis2ML;
	    break;
	case 2:
	    plusLS  = status.Bits.axis3PL;
	    minusLS = status.Bits.axis3ML;
	    break;
	case 3:
	    plusLS  = status.Bits.axis4PL;
	    minusLS = status.Bits.axis4ML;
	default:
	    rtn_state = 1;
	    goto exit;
    }

    /* Set limit switch error indicators. */
    if (plusLS == ON)
    {
	motor_info->status |= RA_PLUS_LS;
	if (plusdir == ON)
	    ls_active = ON;
    }
    else
	motor_info->status &= ~RA_PLUS_LS;

    if (minusLS == ON)
    {
	motor_info->status |= RA_MINUS_LS;
	if (plusdir == OFF)
	    ls_active = ON;
    }
    else
	motor_info->status &= ~RA_MINUS_LS;

    /* encoder status */
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;

    send_mess(card, "AXIS:POS?", NULL);
    recv_mess(card, buff, 1);
    motorData = atof(buff);
    motor_info->encoder_position = (int32_t) motorData;

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

exit:
    return(rtn_state);
}


/*****************************************************/
/* send a message to the PIC844 board		     */
/* send_mess()			                     */
/*****************************************************/
static int send_mess(int card, char const *com, char inchar)
{
    char local_buff[MAX_MSG_SIZE];
    struct PIC844controller *cntrl;
    int size;

    size = strlen(com);

    if (size > MAX_MSG_SIZE)
    {
	logMsg((char *) "drvPIC844.c:send_mess(); message size violation.\n",
	       0, 0, 0, 0, 0, 0);
	return(-1);
    }
    else if (size == 0)	/* Normal exit on empty input message. */
	return(0);

    if (!motor_state[card])
    {
	logMsg((char *) "drvPIC844.c:send_mess() - invalid card #%d\n", card,
	       0, 0, 0, 0, 0);
	return(-1);
    }

    local_buff[0] = NULL;	    /* Terminate local buffer. */

    if (inchar != (char) NULL)
    {
	strcpy(local_buff, "AXIS ");
	local_buff[5] = inchar;		/* put in axis. */
	local_buff[6] = NULL;
	strcat(local_buff, ";");	/* put in comman seperator. */
    }

    /* Make a local copy of the string and add the command line terminator. */
    strcat(local_buff, com);
    strcat(local_buff, "\n");

    Debug(2, "send_mess(): message = %s\n", local_buff);

    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;
    serialIOSend(cntrl->serialInfo, local_buff, strlen(local_buff), SERIAL_TIMEOUT);

    return(0);
}


/*****************************************************/
/* receive a message from the PIC844 board           */
/* recv_mess()			                     */
/*****************************************************/
static int recv_mess(int card, char *com, int flag)
{
    struct PIC844controller *cntrl;
    int timeout;
    int len=0;

    /* Check that card exists */
    if (!motor_state[card])
	return (-1);

    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
	timeout = 0;
    else
	timeout	= SERIAL_TIMEOUT;

    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE, INPUT_TERMINATOR, timeout);

    if (len == 0)
	com[0] = '\0';
    else
    {
	com[len - 1] = '\0';
	len -= 1;
    }

    Debug(2, "recv_mess(): message = \"%s\"\n", com);
    return(len);
}


/*****************************************************/
/* Setup system configuration                        */
/* PIC844Setup()                                     */
/*****************************************************/
int PIC844Setup(int num_cards,	/* maximum number of controllers in system.  */
	    int num_channels,	/* NOT Used. */
	    int scan_rate)	/* polling rate - 1/60 sec units.  */
{
    int itera;

    if (num_cards < 1 || num_cards > PIC844_NUM_CARDS)
	PIC844_num_cards = PIC844_NUM_CARDS;
    else
	PIC844_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
	motor_scan_rate = sysClkRateGet() / scan_rate;
    else
	motor_scan_rate = SCAN_RATE;

   /* 
    * Allocate space for motor_state structures.  Note this must be done
    * before PIC844Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem
    */
    motor_state = (struct controller **) malloc(PIC844_num_cards *
						sizeof(struct controller *));

    for (itera = 0; itera < PIC844_num_cards; itera++)
	motor_state[itera] = (struct controller *) NULL;

    return (0);
}


/*****************************************************/
/* Configure a controller                            */
/* PIC844Config()                                    */
/*****************************************************/
int PIC844Config(int card,	/* card being configured */
            int port_type,      /* GPIB_PORT or RS232_PORT */
	    int addr1,          /* = link for GPIB or hideos_card for RS-232 */
            int addr2)          /* GPIB address or hideos_task */
{
    struct PIC844controller *cntrl;

    if (card < 0 || card >= PIC844_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PIC844controller));
    cntrl = (struct PIC844controller *) motor_state[card]->DevicePrivate;

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
    struct PIC844controller *cntrl;
    int card_index, motor_index, arg3, arg4;
    char buff[BUFF_SIZE];
    int total_axis;
    int status;
    BOOLEAN errind;

    initialized = ON;	/* Indicate that driver is initialized. */

    /* Check for setup */
    if (PIC844_num_cards <= 0)
	return (ERROR);

    for (card_index = 0; card_index < PIC844_num_cards; card_index++)
    {
	if (!motor_state[card_index])
	    continue;
	
	brdptr = motor_state[card_index];
	brdptr->ident[0] = NULL;	/* No controller identification message. */
	brdptr->cmnd_response = OFF;
	total_cards = card_index + 1;
	cntrl = (struct PIC844controller *) brdptr->DevicePrivate;

	/* Initialize communications channel */
	errind = OFF;
	cntrl->serialInfo = serialIOInit(cntrl->serial_card,
					 cntrl->serial_task);
	if (cntrl->serialInfo == NULL)
	    errind = ON;

	if (errind == OFF)
	{
            int retry = 0;

	    /* Send a message to the board, see if it exists */
	    /* flush any junk at input port - should not be any data available */
	    do
		recv_mess(card_index, buff, FLUSH);
	    while (strlen(buff) != 0);
    
	    do
	    {
		send_mess(card_index, GET_IDENT, NULL);
		status = recv_mess(card_index, buff, 1);
                retry++;
	    } while (status == 0 && retry < 3);
	}

	if (errind == OFF && status > 0)
	{
	    strcpy(brdptr->ident, &buff[0]);
	    brdptr->localaddr = (char *) NULL;
	    brdptr->motor_in_motion = 0;
	    brdptr->total_axis = total_axis = 4;

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		struct mess_info *motor_info = &brdptr->motor_info[motor_index];

		motor_info->status = 0;
		motor_info->no_motion_count = 0;
		motor_info->encoder_position = 0;
		motor_info->position = 0;
		brdptr->motor_info[motor_index].motor_motion = NULL;
		/* PIC844 has DC motor support only */
		motor_info->encoder_present = YES;
		motor_info->status |= EA_PRESENT;
		motor_info->pid_present = YES;
		motor_info->status |= GAIN_SUPPORT;

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
	arg3 = (int) (&PIC844_access);
	arg4 = 0;
    }
    else
    {
	arg3 = (int) ((long) &PIC844_access >> 16);
	arg4 = (int) ((long) &PIC844_access & 0xFFFF);
    }
    taskSpawn((char *) "PIC844_motor", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
	      motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}

