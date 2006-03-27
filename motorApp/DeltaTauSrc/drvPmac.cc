/*
FILENAME...	drvPmac.cc
USAGE...	Driver level support for Delta Tau PMAC model.

Version:	1.8
Modified By:	sluiter
Last Modified:	2005/05/10 16:32:41
*/

/*
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
 *
 * NOTES
 * -----
 * Verified with firmware:
 *
 * Modification Log:
 * ----------------------------------------------------------------------------
 * .00 04/17/04 rls - Copied from drvOms.cc
 * .01 09/21/04 rls - support for 32axes/controller.
 * .02 09/27/04 rls - convert "Mbox_addrs" from logical to physical address.
 * .03 10/07/04 rls - mask off high order bits when setting DPRAM address lines.
 * .04 12/21/04 rls - MS Visual C compiler support.
 *                  - eliminate calls to devConnectInterrupt() due to C++
 *		      problems with devLib.h; i.e. "sorry, not implemented:
 *		      `tree_list' not supported..." compiler error message.
 */

#ifdef vxWorks
#include	<vxLib.h>
#include	<sysLib.h>
#include	<taskLib.h>
#include	<rebootLib.h>
#include	<logLib.h>
#endif

#include	<string.h>
#include	<drvSup.h>
#include	<epicsVersion.h>
#include	<devLib.h>
#include	<dbAccess.h>
#include	<epicsThread.h>
#include	<epicsInterrupt.h>

#include	"motor.h"
#include	"drvPmac.h"
#include	"epicsExport.h"
#include        "dbDefs.h"
#include        "registryFunction.h"
#ifdef BUILD_SIMULATION
#include        "Python.h"
#endif
#define CMD_CLEAR       '\030'	/* Control-X, clears command errors only */

#define	ALL_INFO	"QA RP RE EA"	/* jps: move QA to top. */
#define	AXIS_INFO	"QA RP"		/* jps: move QA to top. */
#define	ENCODER_QUERY	"EA"
#define	DONE_QUERY	"RA"

/* Control character responses. */
#define CMNDERR	0x03
#define ACK	0x06
#define CR	0x0D


/*----------------debugging-----------------*/
volatile int drvPmacdebug = 0;
#ifdef	DEBUG
    #define Debug(l, f, args...) { if(l<=drvPmacdebug) printf(f,## args); }
#else
    #define Debug(l, f, args...)
#endif

#define SET_BIT(val,mask,set) ((set)? ((val) | (mask)): ((val) & ~(mask)))
#define BIT_SET( val, bit ) (((val) & (0x1<<(bit)))? 1 : 0 )

/* Global data. */
int Pmac_num_cards = 0;
#ifdef BUILD_SIMULATION
int simulation_mode = 1;
#else
int simulation_mode = 0;
#endif
extern "C" {epicsExportAddress(int, simulation_mode);}
extern "C" {epicsExportAddress(int, drvPmacdebug);}

/* Local data required for every driver; see "motordrvComCode.h" */
#include	"motordrvComCode.h"

/* --- Local data common to all Pmac drivers. --- */
#ifdef vxWorks
SEM_ID test_sem;
#endif
static char * Pmac_addrs = (char *) 0x700000;	/* Base address of DPRAM. */
static epicsAddressType Pmac_ADDRS_TYPE;
static volatile unsigned PmacInterruptVector = 0;
static volatile epicsUInt8 PmacInterruptLevel = Pmac_INT_LEVEL;
static char *Pmac_axis[] =
    {"1",  "2",  "3",  "4",  "5",  "6",  "7",  "8",  "9", "10",
    "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
    "21", "22", "23", "24", "25", "26", "27", "28", "29", "30",
    "31", "32"};

static double quantum;

/*----------------functions-----------------*/

/* Common local function declarations. */
long Pmac_report(int);
static long init();
static void query_done(int, int, struct mess_node *);
static int set_status(int, int);
#if 0
static RTN_STATUS send_mess(int card, char const *com, char *name);
static int recv_mess(int, char *, int);
#else
RTN_STATUS send_mess(int card, char const *com, char *name);
int recv_mess(int, char *, int);
#endif
static void motorIsr(int);
static int motor_init();
static void Pmac_reset();
#ifdef BUILD_SIMULATION
void *SimInit(void *);
#endif
static RTN_STATUS PmacPut(int, char *);
static int motorIsrEnable(int);
static void motorIsrDisable(int);

struct driver_table Pmac_access =
{
    NULL,
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
    Pmac_axis
};

struct
{
    long number;
    long (*report) (int);
    long (*init) (void);
} drvPmac = {2, Pmac_report, init};

extern "C" {epicsExportAddress(drvet, drvPmac);}

static struct thread_args targs = {SCAN_RATE, &Pmac_access, 0.0};

/*----------------functions-----------------*/

long Pmac_report(int level)
{
    int card;

    if (Pmac_num_cards <= 0)
	printf("    No VME8/44 controllers configured.\n");
    else
    {
	for (card = 0; card < Pmac_num_cards; card++)
	    if (motor_state[card])
		printf("    Pmac VME8/44 motor card %d @ 0x%X, id: %s \n",
		       card, (epicsUInt32) motor_state[card]->localaddr,
		       motor_state[card]->ident);
    }
    return (0);
}

static long init()
{
    initialized = true;	/* Indicate that driver is initialized. */
    (void) motor_init();
    return ((long) 0);
}


static void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


static int set_status(int card, int signal)
{
    struct PMACcontroller *cntrl;
    struct mess_node *nodeptr;
    struct mess_info *motor_info;
    /* Message parsing variables */
    char buff[BUFF_SIZE], outbuf[20];
    int rtn_state;
    int motprogram;
    int axishomed;
    double motorData;
    static long int word1debug= 0;
    bool plusdir, ls_active = false, plusLS, minusLS;
    msta_field status;
    long int status1, status2;

    cntrl = (struct PMACcontroller *) motor_state[card]->DevicePrivate;
    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    send_mess(card, "?", Pmac_axis[signal]);
    recv_mess(card, buff, 1);
    rtn_state = sscanf(buff, "%6lx%6lx", &status1, &status2 );

	/* MN DEBUG */
	if( word1debug != status1 )
	{
            word1debug = status1;
            printf("Status words = 0x%lx 0x%lx\n", status1, status2 );
	}


        status.Bits.RA_DONE = BIT_SET( status2, w2_in_position );

        status.Bits.EA_POSITION = BIT_SET( status1, w1_open_loop );

	/* PMAC specific error states (MN 4/5/05) */
	if( BIT_SET( status1, w1_motor_on ) )
	{
		status.All |= PM_MOTACTIVATE;
	}
	else
	{
		status.All &= ~PM_MOTACTIVATE;
	}
	
	if( BIT_SET( status1, w1_amp_enabled ) )
	{
		status.All |= PM_AMPENABLE;
	}
	else
	{
		status.All &= ~PM_AMPENABLE;
	}

	if( BIT_SET( status1, w1_move_time_on ) )
	{
		status.All |= PM_MOTTIMACT;
	}
	else
	{
		status.All &= ~PM_MOTTIMACT;
	}
	
	if( BIT_SET( status1, w1_dwell ))
	{
		status.All |= PM_DWELLACT;
	}
	else
	{
		status.All &= ~PM_DWELLACT;
	}
	
	if(  BIT_SET( status1, w1_DataBlkErr ) )
	{
		status.All |= PM_DATAERR;
	}
	else
	{
		status.All &= ~PM_DATAERR;
	}
	
	if( BIT_SET( status1, w1_CmndVelZero ) )
	{
		status.All |= PM_DESVELZERO;
	}
	else
	{
		status.All &= ~PM_DESVELZERO;
	}
	
	if( BIT_SET( status1, w1_homing ) )
	{
		status.All |= PM_HOMESEARCH;
	}
	else
	{
		status.All &= ~PM_HOMESEARCH;
	}
	
	if( BIT_SET( status2, w2_pos_limit_stop ) )
	{
		status.All |= PM_POSLIMIT;
	}
	else
	{
		status.All &= ~PM_POSLIMIT;
	}
	
	if( BIT_SET( status2, w2_home_complete ) )
	{
		status.All |= PM_HOMCOMPLETE;
	}
	else
	{
		status.All &= ~PM_HOMCOMPLETE;
	}
	
	if( BIT_SET( status2, w2_trigger_move ) )
	{
		status.All |= PM_TRIGGERMOVE;
	}
	else
	{
		status.All &= ~PM_TRIGGERMOVE;
	}
	
	if(  BIT_SET( status2, w2_i2_follow_err ) )
	{
		status.All |= PM_INTFOLERR;
	}
	else
	{
		status.All &= ~PM_INTFOLERR;
	}
		
	if( (BIT_SET( status2, w2_i2t_amp_fault ) ) || 
	    (BIT_SET( status2, w2_amp_fault ) )       )
	{
		status.All |= PM_AMPFAULT;
	}
	else
	{
		status.All &= ~PM_AMPFAULT;
	}
	
	if( (BIT_SET( status2, w2_err_follow_err ) ) ||
	    (BIT_SET( status2, w2_warn_follow_err ) )  )
	{
		status.All |= PM_FOLERR;
	}
	else
	{
		status.All &= ~PM_FOLERR;
	}
	
	if( BIT_SET( status2, w2_in_position )  )
	{
		status.All |= PM_INPOSITION;
	}
	else
	{
		status.All &= ~PM_INPOSITION;
	}
	
    /* Determine if there is a problem */
    if (BIT_SET( status1, w1_DataBlkErr ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Data Block Error\n");
    }   
    else if (BIT_SET( status1, w1_error_trigger ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Trigger error\n");
    }
    else if (BIT_SET( status2, w2_pos_limit_stop ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Stopped on position limit\n");
    }
    else if (BIT_SET( status2, w2_phase_ref_err ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Phasing Reference Error\n");
    }
    else if (BIT_SET( status2, w2_i2_follow_err ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Integrated Fatal Following Error\n");
    }
    else if (BIT_SET( status2, w2_i2t_amp_fault ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("I^2T Amplifier Fault Error\n");
    }
    else if (BIT_SET( status2, w2_amp_fault ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Amplifier Fault Error\n");
    }
    else if (BIT_SET( status2, w2_err_follow_err ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Fatal Following Error\n");
    }
    else if (BIT_SET( status2, w2_warn_follow_err ) )
    {
      status.Bits.RA_PROBLEM = 1;
      errlogPrintf("Warning following error\n");	
    }
	    
    else
    {
      /* No problem detected */
      status.Bits.RA_PROBLEM = 0;
    }

    #if 0
    if( motorstat.word1.Bits.move_time_on ) )
    {
      status.Bits.RA_MOVING = 1;
    }
    else
    { /* bastard Rok 
	    if(signal==15) {
		    status.All |= RA_MOVING;
		    errlogPrintf("PMAC Phantom Cheating...\n");
	    }
            else     */  status.Bits.RA_MOVING = 0;
    }
    #endif
 

    /* Get PMAC motion program status bits (MN 13/04/05) */
    sprintf(outbuf, "M%.2d34", (signal + 1)); /* Get Motion program status */
    send_mess(card, outbuf, (char) NULL);
    recv_mess(card, buff, 1);

    motprogram = atoi( buff );
    if( motprogram )
    {
      status.All |= PM_MOTPROGRAM;
    }
    else
    {
      status.All &= ~PM_MOTPROGRAM;
    }

    sprintf(outbuf, "M%.2d44", (signal + 1)); /* Get homed status bit */
    send_mess(card, outbuf, (char) NULL);
    recv_mess(card, buff, 1);

    axishomed = atoi( buff );
    if( axishomed )
    {
      status.All |= PM_AXISHOMED;
    }
    else
    {
      status.All &= ~PM_AXISHOMED;
    }



    sprintf(outbuf, "M%.2d61", (signal + 1));	/* Get Commanded Position. */
    send_mess(card, outbuf, (char) NULL);
    recv_mess(card, buff, 1);

    motorData = atof(buff);
    motorData /= cntrl->pos_scaleFac[signal + 1];

    /* Replaced with PMAC scale factor */
    /* motorData /= 32.0;  Shift out fractionial data. */
    /* motorData /= 96.0;  Divide by position scale factor (Ixx08) */

    if (motorData == motor_info->position)
    {
	    if (nodeptr != 0)	{ /* Increment counter only if motor is moving. */
	    	 motor_info->no_motion_count++;
                 errlogPrintf("PMAC: no_motion_count=%d\n", motor_info->no_motion_count);
		}
    }
    else
    {
	epicsInt32 newposition;

	newposition = NINT(motorData);
	status.Bits.RA_DIRECTION = (newposition >= motor_info->position) ? 1 : 0;
	motor_info->position = newposition;
	motor_info->no_motion_count = 0;
    }

    plusdir = (status.Bits.RA_DIRECTION) ? true : false;
    plusLS  = BIT_SET( status1, w1_pos_limit_set);
    minusLS = BIT_SET( status1, w1_neg_limit_set);

    /* Set limit switch error indicators. */
    if (plusLS == true)
    {
	status.Bits.RA_PLUS_LS = 1;
	if (plusdir == true)
	    ls_active = true;
    }
    else
	status.Bits.RA_PLUS_LS = 0;

    if (minusLS == true)
    {
	status.Bits.RA_MINUS_LS = 1;
	if (plusdir == false)
	    ls_active = true;
    }
    else
	status.Bits.RA_MINUS_LS = 0;

    /* encoder status */
    status.Bits.EA_SLIP	      = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME	      = 0;

    sprintf(outbuf, "M%.2d62", (signal + 1));
    send_mess(card, outbuf, (char) NULL);	// Get Actual Position.
    recv_mess(card, buff, 1);
    motorData = atof(buff);
    motorData /= cntrl->pos_scaleFac[signal + 1];
    
    printf("MOTOR CODE: motorData = %d\n",(int32_t) motorData);
    
    /* Replaced by PMAC scale factor */
    /* motorData /= 32.0;     Shift out fractional data */
    /* motorData /= 96.0;     Divide by position scale factor (Ixx08) */
    motor_info->encoder_position = (int32_t) motorData;

    /*motor_info->status &= ~RA_PROBLEM;*/

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!status.Bits.RA_DIRECTION)
	motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		status.Bits.RA_DONE || status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if ((status.Bits.RA_DONE || ls_active == true) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
	strcpy(buff, nodeptr->postmsgptr);
	send_mess(card, buff, (char) NULL);
	nodeptr->postmsgptr = NULL;
    }

    motor_info->status.All = status.All;
    return(rtn_state);
}


/*****************************************************/
/* send a message to the Pmac board		     */
/*		send_mess()			     */
/*****************************************************/
#if 0
static RTN_STATUS send_mess(int card, char const *com, char *name)
#else
RTN_STATUS send_mess(int card, char const *com, char *name)
#endif
{
    char outbuf[MAX_MSG_SIZE];
    RTN_STATUS return_code;

#ifdef vxWorks
    STATUS sem_ret;

    sem_ret = semTake( test_sem, 5000);
    if( sem_ret == OK )
    {
/*      printf("send_mess: GOT SEMAPHORE\n");*/
#endif
      if (strlen(com) > MAX_MSG_SIZE)
      {
	  errlogMessage("drvPmac.cc:send_mess(); message size violation.\n");
	  return (ERROR);
      }

      /* Check that card exists */
      if (!motor_state[card])
      {
	errlogMessage("drvPmac.cc:send_mess() - invalid card \n");

#ifdef vxWorks
        sem_ret = semGive( test_sem );
      	if( sem_ret != OK )
      	{
			printf("Error returning semaphore\n");
        }
#endif
	return (ERROR);
      }

      /* Flush receive buffer */
      recv_mess(card, (char *) NULL, -1);

      if (name == NULL)
	strcpy(outbuf, com);
      else
      {
	strcpy(outbuf, "#");
	strcat(outbuf, name);
	strcat(outbuf, com);
      }

      Debug(9, "send_mess: ready to send message.\n");

      return_code = PmacPut(card, outbuf);

      if (return_code == OK)
      {
	Debug(4, "sent message: (%s)\n", outbuf);
      }
      else
      {
	Debug(4, "unable to send message (%s)\n", outbuf);
#ifdef vxWorks
		sem_ret = semGive( test_sem );
      	if( sem_ret != OK )
      	{
			printf("Error returning semaphore\n");
        }
#endif
	return (ERROR);
      }

#ifdef vxWorks
      sem_ret = semGive( test_sem );
      if( sem_ret != OK )
      {
	printf("Error returning semaphore\n");
	return_code = ERROR;
      }
      
    }
    else
    {
       printf("Error taking semaphore\n");
       return_code = ERROR;
    }
#endif
    
    return (return_code);
}


/*
 * FUNCTION... recv_mess(int card, char *com, int amount)
 *
 * INPUT ARGUMENTS...
 *	card - controller card # (0,1,...).
 *	*com - caller's response buffer.
 *	amount	| -1 = flush controller's output buffer.
 *		| >= 1 = the # of command responses to retrieve into caller's
 *				response buffer.
 *
 * LOGIC...
 *  IF controller card does not exist.
 *	ERROR RETURN.
 *  ENDIF
 *  IF "amount" indicates buffer flush.
 *	WHILE characters left in input buffer.
 *	    Call PmacGet().
 *	ENDWHILE
 *  ENDIF
 *
 *  FOR each message requested (i.e. "amount").
 *	Initialize head and tail pointers.
 *	Initialize retry counter and state indicator.
 *	WHILE retry count not exhausted, AND, state indicator is NOT at END.
 *	    IF characters left in controller's input buffer.
 *		Process input character.
 *	    ELSE IF command error occured - call PmacError().
 *		ERROR RETURN.
 *	    ENDIF
 *	ENDWHILE
 *	IF retry count exhausted.
 *	    Terminate receive buffer.
 *	    ERROR RETURN.
 *	ENDIF
 *	Terminate command response.
 *  ENDFOR
 *
 *  IF commands processed.
 *	Terminate response buffer.
 *  ELSE
 *	Clear response buffer.
 *  ENDIF
 *  NORMAL RETURN.
 */

#if 0
static int recv_mess(int card, char *com, int amount)
#else
int recv_mess(int card, char *com, int amount)
#endif
{
    volatile struct controller *pmotorState;
    volatile struct pmac_dpram *pmotor;
    volatile epicsUInt8 *stptr;
    int trys;
    char control;

    pmotorState = motor_state[card];
    pmotor = (struct pmac_dpram *) pmotorState->localaddr;
    stptr = &pmotor->reply_status.Bits.term;
    Debug(9, "recv_mess() - pmotor = %p, out_cntrl_wd %p, out_ctrl_char %p, cmdbuff %p reply_status %p reply count %p response %p\n",
          pmotor, &(pmotor->out_cntrl_wd), &(pmotor->out_cntrl_char), &(pmotor->cmndbuff[0]), 
          &(pmotor->reply_status), &(pmotor->reply_count), &(pmotor->response) );
    
    /* Check that card exists */
    if (card >= total_cards)
    {
	Debug(1, "recv_mess - invalid card #%d\n", card);
	return (-1);
    }

    if (amount == -1)
    {
	bool timeout = false, flushed = false;
	control = *stptr;

	while (timeout == false && flushed == false)
	{
	    const double flush_delay = quantum;

	    if (control == (char) NULL)
	    {
		Debug(6, "recv_mess() - flush wait on NULL\n");
		epicsThreadSleep(flush_delay);
                control = *stptr;
		// control = stptr->Bits.cntrl_char;
		if (control == (char)NULL)
		    flushed = true;
		else
		    Debug(6, "recv_mess() - NULL -> %c\n", control);
	    }
	    else if (control == ACK)
	    {
		pmotor->reply_status.All  = 0;
		Debug(6, "recv_mess() - flush wait on ACK\n");
		epicsThreadSleep(flush_delay);
		// control = stptr->Bits.cntrl_char;
                control = *stptr;
	    }
	    else if (control == CR)
	    {
		pmotor->reply_status.All = 0;
		Debug(6, "recv_mess() - flush wait on CR\n");
		for (trys = 0; trys < 10 && (*stptr) == 0; trys++)
		{
		    epicsThreadSleep(quantum * trys);
		    Debug(6, "recv_mess() - flush wait #%d\n", trys);
		}
		if (trys >= 10)
		    timeout = true;

		// control = stptr->Bits.cntrl_char;
                control = *stptr;
	    }
	    else
	    {
		pmotor->reply_status.All = 0;
		errlogPrintf("%s(%d): ERROR = 0x%X\n", __FILE__, __LINE__,
			     (epicsUInt32) control);
		epicsThreadSleep(flush_delay);
                flushed = true;

		// control = stptr->Bits.cntrl_char;
                control = *stptr;
	    }
	}

	if (timeout == true)
	    errlogPrintf("%s(%d): flush timeout\n", __FILE__, __LINE__);

	return(0);
    }

    for (trys = 0; trys < 10;)
    {
	if (pmotor->reply_status.All == 0)
	{
	    trys++;
	    epicsThreadSleep(quantum * 2.0);
	}
	else
	    break;
    }
    
    if (trys >= 10)
    {
	Debug(1, "recv_mess() timeout.\n");
	return(-1);
    }

    control = *stptr;

    if (control == CMNDERR)
    {
        pmotor->reply_status.All = 0;
	Debug(1, "recv_mess(): command error.\n");
	return(-1);
    }
    else if (control == ACK)
    {
	Debug(4, "recv_mess(): control = ACK\n");
        pmotor->reply_status.All = 0;
	return(recv_mess(card, com, amount));
    }
    else if (control == CR)
    {
	strcpy(com, (char *) &pmotor->response[0]);
        pmotor->reply_status.All = 0;
	Debug(4, "recv_mess(): card %d, msg: (%s) from address %p\n", card, com, &pmotor->response[0] );
	return(0);
    }
    else
    {
        pmotor->reply_status.All = 0;
	errlogPrintf("%s(%d): ERROR = 0x%X\n", __FILE__, __LINE__,
		     (unsigned int) control);
	return(-1);
    }
}


/*****************************************************/
/* Send Message to Pmac                               */
/*		PmacPut()			     */
/*****************************************************/
static RTN_STATUS PmacPut(int card, char *pmess)
{
    volatile struct controller *pmotorState;
    volatile struct pmac_dpram *pmotor;
    volatile epicsUInt8 *stptr;
    int itera;

    pmotorState = motor_state[card];
    pmotor = (struct pmac_dpram *) pmotorState->localaddr;
    stptr = &(pmotor->reply_status.Bits.term);
    
    for(itera = 0; itera < 10; itera++)
    {
	if(pmotor->out_cntrl_wd == 0)
	    break;
	else
	    epicsThreadSleep(0.010);
    }

    if(itera >= 10)
	return(ERROR);
    else
    {
	strcpy((char *) &pmotor->cmndbuff[0], pmess);
	pmotor->out_cntrl_wd = 1;
    }
    
    /* Wait for response. */
    for (itera = 0; itera < 10 && (*stptr == 0); itera++)
    {
	epicsThreadSleep(quantum * itera);
	Debug(7, "PmacPut() - response wait #%d\n", itera);      
    }

    if (itera >= 10)
    {
	errlogPrintf("%s(%d): response timeout.\n", __FILE__, __LINE__);
	return(ERROR);
    }

    return (OK);
}



/*****************************************************/
/* Interrupt service routine.                        */
/* motorIsr()		                     */
/*****************************************************/
static void motorIsr(int card)
{
}

static int motorIsrEnable(int card)
{
#ifdef vxWorks
    long status;
    
    status = pdevLibVirtualOS->pDevConnectInterruptVME(
	PmacInterruptVector + card, (void (*)()) motorIsr, (void *) card);

    status = devEnableInterruptLevel(Pmac_INTERRUPT_TYPE,
				     PmacInterruptLevel);
#endif

    return (OK);
}

static void motorIsrDisable(int card)
{
#ifdef vxWorks
    long status;

    status = pdevLibVirtualOS->pDevDisconnectInterruptVME(
	      PmacInterruptVector + card, (void (*)(void *)) motorIsr);

    if (!RTN_SUCCESS(status))
	errPrintf(status, __FILE__, __LINE__, "Can't disconnect vector %d\n",
		  PmacInterruptVector + card);
#endif
}


/*****************************************************
 * FUNCTION... PmacSetup()
 *
 * USAGE...Configuration function for PMAC.
 *                                
 * LOGIC...
 *  Check for valid input on maximum number of cards.
 *  Based on VMEbus address type, check for valid Mailbox and DPRAM addresses.
 *  Bus probe the logical mailbox address.
 *  IF mailbox address valid.
 *	Register 122 bytes of memory (0-121) based on mailbox base address.
 *	Save physical address of mailbox base address in "Mbox_addrs".
 *	Page-select DPRAM by writing to Mbox_addrs+0x121.
 *  ELSE
 *	Log error and set both Mbox_addrs and Pmac_num_cards to zero.
 *  ENDIF
 *****************************************************/

int PmacSetup(int num_cards,	/* maximum number of cards in rack */
	     int addrs_type,	/* VME address type; 24 - A24 or 32 - A32. */
	     void *mbox,	/* Mailbox base address. */
	     void *addrs,	/* DPRAM Base Address */
	     unsigned vector,	/* noninterrupting(0), valid vectors(64-255) */
	     int int_level,	/* interrupt level (1-6) */
	     int scan_rate)	/* polling rate - in HZ */
{
    char *Mbox_addrs;		/* Base address of Mailbox. */
    volatile void *localaddr;
    void *probeAddr, *erraddr = 0;
    long status;

    if (num_cards < 1 || num_cards > Pmac_NUM_CARDS)
    {
      errlogPrintf("Invalid number of cards(%d) setting to %d",num_cards,Pmac_NUM_CARDS);
      Pmac_num_cards = Pmac_NUM_CARDS;
    }
    else
    {
	Pmac_num_cards = num_cards;
    }

    switch(addrs_type)
    {
	case 24:
	    Pmac_ADDRS_TYPE = atVMEA24;

	    if ((epicsUInt32) mbox & 0xF0000000)
		erraddr = mbox;
	    else if ((epicsUInt32) addrs & 0xF)
		erraddr = addrs;

	    if (erraddr != 0)
		Debug(1, "PmacSetup: invalid A24 address 0x%X\n", (epicsUInt32) mbox);

	    break;
	case 32:
	    Pmac_ADDRS_TYPE = atVMEA32;
	    break;
	default:
	    Debug(1, "PmacSetup: invalid Address Type %d\n", (epicsUInt32) addrs);
	    break;
    }

#ifdef vxWorks
    /* Test MailBox address. */
    /* I don't want to do this --- Rok Sabjan */
    Mbox_addrs = (char *) mbox;
    /*
    status = devNoResponseProbe(Pmac_ADDRS_TYPE, (epicsUInt32)
				(Mbox_addrs + 0x121), 1);
    */
    Debug(1,"Bypassing mailbox probe\n");
#endif

    /* Force this probe to always be successful */
	  status = S_dev_addressOverlap;

    if( !simulation_mode )
    {


    if (PROBE_SUCCESS(status))
    {
	char A19A14;	 /* Select VME A19-A14 for DPRAM. */
#ifdef vxWorks
	status = devRegisterAddress(__FILE__, Pmac_ADDRS_TYPE, (size_t)
			    Mbox_addrs, 122, (volatile void **) &localaddr);
	Debug(9, "motor_init: devRegisterAddress() status = %d\n", (int) status);
#endif

	if (!RTN_SUCCESS(status))
	{
	    errPrintf(status, __FILE__, __LINE__, "Can't register address 0x%x\n",
		      (epicsUInt32) probeAddr);
	    return (ERROR);
	}

	Mbox_addrs = (char *) localaddr;	/* Convert to physical address.*/
	Pmac_addrs = (char *) addrs;
	/* A19A14 = (char) ((unsigned long) Pmac_addrs >> 14); */
        /* A19A14 &= 0x3F; */
        /* *(Mbox_addrs + 0x121) = A19A14; */
	*(Mbox_addrs + 0x121) = (char) ((unsigned long) Pmac_addrs >> 14); /* Select VME A19-A14 for DPRAM. */
    }
    else
    {
	errlogPrintf("%s(%d): Mailbox bus error - 0x%X\n", __FILE__, __LINE__,
		     (epicsUInt32) (Mbox_addrs + 0x121));
	Mbox_addrs = (char *) NULL;
	/* Pmac_num_cards = 0; */
    }

    }/* End of if (!simulation_mode) */

    PmacInterruptVector = vector;
    if (vector < 64 || vector > 255)
    {
	if (vector != 0)
	{
	    Debug(1, "PmacSetup: invalid interrupt vector %d\n", vector);
	    PmacInterruptVector = (unsigned) Pmac_INT_VECTOR;
	}
    }

    if (int_level < 1 || int_level > 6)
    {
	Debug(1, "PmacSetup: invalid interrupt level %d\n", int_level);
	PmacInterruptLevel = Pmac_INT_LEVEL;
    }
    else
	PmacInterruptLevel = int_level;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= MAX_SCAN_RATE)
	targs.motor_scan_rate = scan_rate;
    else
    {
	targs.motor_scan_rate = SCAN_RATE;
	errlogPrintf("%s(%d): invalid poll rate - %d HZ\n", __FILE__, __LINE__,
		      scan_rate);
    }
    return(0);
}

/*****************************************************/
/* initialize all software and hardware		     */
/*		motor_init()			     */
/*****************************************************/
static int motor_init()
{
    volatile struct controller *pmotorState;
    volatile struct pmac_dpram *pmotor;
    struct PMACcontroller *cntrl;
    long status;
    int card_index, motor_index;
    char axis_pos[50];
    char *tok_save;
    int total_encoders = 0, total_axis = 0;
    volatile void *localaddr;
    void *probeAddr;
    bool errind;
#ifdef BUILD_SIMULATION
    volatile struct pmac_dpram *psimdpram[Pmac_NUM_CARDS];
    pthread_t python_thread;
    simargs_t simargs;
#endif
    

    tok_save = NULL;
    quantum = epicsThreadSleepQuantum();
    Debug(5, "motor_init: epicsThreadSleepQuantum = %f\n", quantum);

#ifdef vxWorks
    test_sem = semBCreate(SEM_Q_PRIORITY, SEM_FULL);    
    printf("test_sem = 0x%x\n",test_sem);
    semShow(test_sem,1);
#endif

    /* Check for setup */
    if (Pmac_num_cards <= 0)
    {
	Debug(1, "motor_init: *Pmac driver disabled* \n PmacSetup() is missing from startup script.\n");
	return (ERROR);
    }

#ifdef BUILD_SIMULATION
    if( simulation_mode )
    {
      printf("simulation mode\n");
      /* Initialise DPRAM */
      for(card_index=0;card_index<Pmac_NUM_CARDS;card_index++)
	psimdpram[card_index]=NULL;

      /* Allocate memory for simulation DPRAM */            
      for(card_index = 0; card_index < Pmac_num_cards; card_index++)
      {
          psimdpram[card_index]=(struct pmac_dpram *)calloc( 1, sizeof( struct
								   pmac_dpram ) );
	if( psimdpram[card_index] != NULL )
	{
	  simargs.pdpram[card_index] = (struct pmac_dpram *) psimdpram[card_index];
	} 
	else
	{
	  errlogPrintf("%s(%d): Error in DPRAM calloc\n",__FILE__,__LINE__);
	}
	

	Debug(1,"psimdpram[%d] = 0x%p\n",card_index,psimdpram[card_index]);

      }/* End of for loop */
	
      simargs.numcards = Pmac_num_cards;

      /* Spawn the python thread */
      status = pthread_create(&python_thread,NULL,SimInit,(void *)&simargs);
      if( status )
      {
	errlogPrintf("Error starting python thread\n");
	return( ERROR );
      }
      
      sleep(10);
      
    }/* End of if( simulation_mode ) */
#endif
    
    /* allocate space for total number of motors */
    motor_state = (struct controller **) calloc(Pmac_num_cards,
						sizeof(struct controller *));

    /* allocate structure space for each motor present */

    total_cards = Pmac_num_cards;

#ifdef vxWorks
    if (rebootHookAdd((FUNCPTR) Pmac_reset) == ERROR)
	Debug(1, "vme8/44 motor_init: Pmac_reset disabled\n");
#endif

    for (card_index = 0; card_index < Pmac_num_cards; card_index++)
    {
	int8_t *startAddr;
	int8_t *endAddr;

	Debug(2, "motor_init: card %d\n", card_index);

	probeAddr = Pmac_addrs + (card_index * Pmac_BRD_SIZE);
	startAddr = (int8_t *) probeAddr + 1;
	endAddr = startAddr + Pmac_BRD_SIZE;

	Debug(9, "motor_init: devNoResponseProbe() on addr 0x%x\n", (epicsUInt32) probeAddr);
	/* Scan memory space to assure card id */
	do
	{
#ifdef vxWorks
            status = devNoResponseProbe(Pmac_ADDRS_TYPE, (unsigned int) startAddr, 1);
#endif
	    startAddr += 0x100;
	} while (PROBE_SUCCESS(status) && startAddr < endAddr);

	/* Force this probe to always be successful */
	/* if( simulation_mode ) */
	  status = S_dev_addressOverlap;

	if (PROBE_SUCCESS(status))
	{

#ifdef vxWorks
	    status = devRegisterAddress(__FILE__, Pmac_ADDRS_TYPE,
					(size_t) probeAddr, Pmac_BRD_SIZE,
					(volatile void **) &localaddr);
	    Debug(9, "motor_init: devRegisterAddress() status = %d\n",
		  (int) status);
#endif
	    if( !simulation_mode )
	    {
	      if (!RTN_SUCCESS(status))
	      {
	  	errPrintf(status, __FILE__, __LINE__,
			  "Can't register address 0x%x\n", (unsigned) probeAddr);
		return (ERROR);
	      }
	    }

	    Debug(9, "motor_init: localaddr = %x\n", (int) localaddr);

	    Debug(9, "motor_init: calloc'ing motor_state\n");
	    motor_state[card_index] = (struct controller *) calloc(1, sizeof(struct controller));
	    pmotorState = motor_state[card_index];

#ifdef BUILD_SIMULATION
	    if( simulation_mode )
	    {
	      /* Point to simulated DPRAM */
	       Debug(1,"Setting localaddr to simulated DPRAM\n");
               pmotorState->localaddr = (char *) psimdpram[card_index];
            }
            else
#endif
            {
              /* Point to DPRAM */
              Debug(1,"Setting localaddr to PMAC DPRAM\n");
              pmotorState->localaddr = (char *) localaddr;
            }

	    pmotorState->motor_in_motion = 0;
	    pmotorState->cmnd_response = false;

	    cntrl = (struct PMACcontroller *) calloc(1, sizeof(struct PMACcontroller));
	    pmotorState->DevicePrivate = cntrl;
	    cntrl->irqEnable = FALSE;

	    /* Initialize DPRAM communication. */
	    pmotor = (struct pmac_dpram *) pmotorState->localaddr;
	    pmotor->out_cntrl_wd = 0;	/* Clear "Data ready from host" bit indicator. */
	    pmotor->out_cntrl_char = 0;	/* Clear "Buffer Control Character. */
	    pmotor->reply_status.All = 0;

	    send_mess(card_index, "TYPE", (char) NULL);
	    recv_mess(card_index, (char *) pmotorState->ident, 1);

	    send_mess(card_index, "VERSION", (char) NULL);
	    recv_mess(card_index, axis_pos, 1);
	    strcat((char *) &pmotorState->ident, ", ");
	    strcat((char *) &pmotorState->ident, axis_pos);

	    Debug(3, "Identification = %s\n", pmotorState->ident);

	    for (total_axis = 0, errind = false; errind == false &&
		 total_axis <= Pmac_MAX_AXES; total_axis++)
	    {
		char outbuf[10];

		sprintf(outbuf, "I%.2d00", (total_axis + 1));
		send_mess(card_index, outbuf, (char) NULL);
		recv_mess(card_index, axis_pos, 1);
		if (strcmp(axis_pos, "0") == 0)
                {
		    // errind = true;
                }
		else if (strcmp(axis_pos, "1") == 0)
		{
		    pmotorState->motor_info[total_axis].motor_motion = NULL;
		    pmotorState->motor_info[total_axis].status.All = 0;

		    // Set Ixx20=1 and Ixx21=0; control acceleration via Ixx19.
		    sprintf(outbuf, "I%.2d20=1", (total_axis + 1));
		    send_mess(card_index, outbuf, (char) NULL);
		    sprintf(outbuf, "I%.2d21=0", (total_axis + 1));
		    send_mess(card_index, outbuf, (char) NULL);

            /* Read and save Position Scale Factor */
		    sprintf(outbuf, "I%.2d08", (total_axis + 1));
		    send_mess(card_index, outbuf, (char) NULL);
		    recv_mess(card_index, axis_pos, 1);
		    cntrl->pos_scaleFac[total_axis+1] = atof(axis_pos) * 32.0;
		    Debug(1, "Pos scale factor %f\n",cntrl->pos_scaleFac[total_axis]);

		}
		else
		{
		    Debug(1, "Invalid response = \"%s\" to msg = \"%s\"\n", axis_pos, outbuf);
		}
	    }

	    pmotorState->total_axis = --total_axis;
	    Debug(3, "Total axis = %d\n", total_axis);

	    /*
	     * Enable interrupt-when-done if selected - driver depends on
	     * motor_state->total_axis  being set.
	     */
	    if (PmacInterruptVector)
	    {
		if (motorIsrEnable(card_index) == ERROR)
		    errPrintf(0, __FILE__, __LINE__, "Interrupts Disabled!\n");
	    }

	    for (total_encoders = 0, motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		total_encoders++;
		pmotorState->motor_info[motor_index].encoder_present = YES;
	    }

	    for (motor_index = 0; motor_index < total_axis; motor_index++)
	    {
		pmotorState->motor_info[motor_index].status.All = 0;
		pmotorState->motor_info[motor_index].no_motion_count = 0;
		pmotorState->motor_info[motor_index].encoder_position = 0;
		pmotorState->motor_info[motor_index].position = 0;

		if (pmotorState->motor_info[motor_index].encoder_present == YES)
		    pmotorState->motor_info[motor_index].status.Bits.EA_PRESENT = 1;
		set_status(card_index, motor_index);
	    }

	    Debug(2, "Init Address=0x%8.8x\n", (epicsUInt32) localaddr);
	    Debug(3, "Total encoders = %d\n\n", (int) total_encoders);
	}
	else
	{
	    Debug(3, "motor_init: Card NOT found!\n");
	    motor_state[card_index] = (struct controller *) NULL;
	}
    }

    // motor_sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);

    any_motor_in_motion = 0;

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;
    
    Debug(3, "Motors initialized\n");

    epicsThreadCreate((const char *) "Pmac_motor", epicsThreadPriorityMedium,
		      epicsThreadGetStackSize(epicsThreadStackMedium),
		      (EPICSTHREADFUNC) motor_task, (void *) &targs);

    Debug(3, "Started motor_task\n");
    return (0);
}

epicsRegisterFunction(PmacSetup);

/* Disables interrupts. Called on CTL X reboot. */

static void Pmac_reset()
{
}


#ifdef BUILD_SIMULATION
/*---------------------------------------------------------------------*/
void *SimInit( void *ptr )
{
  struct pmac_dpram **psimdpram;
  PyObject *module,*funcdict,*function, *funccall, *shmemobj, *args;
  simargs_t *psimargs;
  int numcards,card;

  psimargs = (simargs_t *) ptr;

  numcards = psimargs->numcards;

  psimdpram = (struct pmac_dpram **) psimargs->pdpram;

  /* Launch the python interpreter */
  Debug(1,"Initilising Python interpreter\n");
  Py_Initialize();
  
  /* Initialise the python search path to include local directory */
  Debug(1,"Importing sys module\n");
  PyRun_SimpleString("import sys\n");
  PyRun_SimpleString("sys.path\n");
  
  /* Import simulation module */
  Debug(1,"Importing dpramsimulation module\n");
  module = PyImport_ImportModule("dpramsimulation");
  if( !module )
  {
    errlogPrintf("Error importing module\n");
    PyErr_Print();
    PyErr_Clear();
    pthread_exit(NULL);
  }
  
  /* Get a dictionary of module functions */
  Debug(1,"Retrieving module functions\n");
  funcdict = PyModule_GetDict(module);
  if( !funcdict )
  {
    errlogPrintf("Error importing dictionary\n");
    PyErr_Print();
    PyErr_Clear();
    pthread_exit(NULL);
  }
  
  /* Get starting function */
  #if 1
  Debug(1,"Get the starting function\n");
  function = PyDict_GetItemString(funcdict,"startfunc");
  if( !function )
  {
    errlogPrintf("Error getting function\n");
    PyErr_Print();
    PyErr_Clear();
    pthread_exit(NULL);
  }
  #else
  Debug(1,"Get the starting function\n");
  function = PyDict_GetItemString(funcdict,"testfunc");
  if( !function )
  {
    errlogPrintf("Error getting function\n");
    PyErr_Print();
    PyErr_Clear();
    pthread_exit(NULL);
  }
  #endif


  for( card=0; card<numcards; card++ )
  {
    
    /* Create shared memory object */
    Debug(1,"Create shared memory object\n");
    shmemobj = PyBuffer_FromReadWriteMemory((void *) psimdpram[card],sizeof( struct pmac_dpram ));
    if( !shmemobj )
    {
      errlogPrintf("Error creating shared memory object\n");
      PyErr_Print();
      PyErr_Clear();
      pthread_exit(NULL);
    }
    
    /* Build argument list */
    args = Py_BuildValue("(O)",shmemobj);
    if( !args )
    {
      errlogPrintf("Error building argument list\n");
      PyErr_Print();
      PyErr_Clear();
      pthread_exit(NULL);
    }
    
    /* Call starting function */
    printf("Calling starting function\n");
    funccall = PyObject_CallObject(function,args);
    if( !funccall )
    {
      errlogPrintf("Error calling function\n");
      PyErr_Print();
      PyErr_Clear();
      pthread_exit(NULL);
    }
    
  }/* End of for loop */
  
  /* Enable python threading */
  Py_BEGIN_ALLOW_THREADS
  pause();
  Py_END_ALLOW_THREADS

}
#endif
