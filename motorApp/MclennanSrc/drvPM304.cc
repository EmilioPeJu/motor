/* File: drvPM304.cc                    */
/* Version: 2.01                        */
/* Date Last Modified: 10/26/99         */


/* Device Driver Support routines for motor */
/*
 *      Original Author: Mark Rivers
 *      Date: 11/20/98
 *
 * Modification Log:
 * -----------------
 * .01  11-20-98   mlr  initialized from drvMM4000
 * .02  09-29-99   mlr  Converted to motor record V4.04
 * .03  10-26-99   mlr  Minor fixes for motor record V4.0
 * .04  08-16-00   mlr  Fixed serious problem with limits - they were not
 *                      correct, bring extract from wrong character in response
 *                      Minor fixes to avoid compiler warnings
 * .05  11-27-01   mlr  Added global variable drvPM304ReadbackDelay.  This is a
 *                      double time in seconds to wait after the PM304 says the move
 *                      is complete before reading the encoder position the final
 *                      time.
 * .06  07-03-2002 rls  replaced RA_OVERTRAVEL with RA_PLUS_LS and RA_MINUS_LS
 * .07  02-11-2003 mlr  Added support for PM600 model.  Added send_recv_mess which
 *                      simplifies and cleans up.
 * .08  03/27/03   rls  R3.14 conversion.
 */


#include 	<string.h>
#include 	<epicsThread.h>
#include 	<drvSup.h>
#include        "motor.h"
#include        "drvPM304.h"
#include        "serialIO.h"
#include 	"epicsExport.h"

#define STATIC static

#define SERIAL_TIMEOUT 2000 /* Command timeout in msec */

#define BUFF_SIZE 200       /* Maximum length of string to/from PM304 */

/* This is a temporary fix to introduce a delayed reading of the motor
 * position after a move completes
 */
volatile double drvPM304ReadbackDelay = 0.;

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};


#ifdef __GNUG__
    #ifdef DEBUG
	volatile int drvPM304Debug = 0;
	#define Debug(L, FMT, V...) { if(L <= drvPM304Debug) \
                        { errlogPrintf("%s(%d):",__FILE__,__LINE__); \
                          errlogPrintf(FMT,##V); } }
    #else
	#define Debug(L, FMT, V...)
    #endif
#else
    #define Debug()
#endif

/* Debugging notes:
 *   drvPM304Debug == 0  No debugging information is printed
 *   drvPM304Debug >= 1  Warning information is printed
 *   drvPM304Debug >= 2  Time-stamped messages are printed for each string 
 *                       sent to and received from the controller
 *   drvPM304Debug >= 3  Additional debugging messages
 */    

int PM304_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/*----------------functions-----------------*/
STATIC int recv_mess(int card, char *buff, int len);
STATIC RTN_STATUS send_mess(int card, const char *com, char c);
STATIC int send_recv_mess(int card, const char *out, char *in);
STATIC void start_status(int card);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table PM304_access =
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
    start_status,
    &initialized
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
} drvPM304 = {2, report, init};

epicsExportAddress(drvet, drvPM304);

STATIC struct thread_args targs = {SCAN_RATE, &PM304_access};


/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
  int card;

  if (PM304_num_cards <=0)
    printf("    NO PM304 controllers found\n");
  else
    {
      for (card = 0; card < PM304_num_cards; card++)
          if (motor_state[card])
             printf("    PM304 controller %d, id: %s \n",
                   card,
                   motor_state[card]->ident);
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
    if (PM304_num_cards <= 0)
    {
        Debug(1, "init: *PM304 driver disabled*\n");
        Debug(1, "PM304Setup() is missing from startup script.\n");
        return (ERROR);
    }

    return ((long) 0);
}

STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{
}


/*********************************************************
 * Read the status and position of all motors on a card
 * start_status(int card)
 *            if card == -1 then start all cards
 *********************************************************/
STATIC void start_status(int card)
{
    /* The PM304 cannot query status or positions of all axes with a
     * single command.  This needs to be done on an axis-by-axis basis,
     * so this function does nothing
     */
}


/**************************************************************
 * Query position and status for an axis
 * set_status()
 ************************************************************/

STATIC int set_status(int card, int signal)
{
    register struct mess_info *motor_info;
    char command[BUFF_SIZE];
    char response[BUFF_SIZE];
    struct mess_node *nodeptr;
    int rtn_state;
    long motorData;
    char buff[BUFF_SIZE];
    bool ls_active = false;
    struct PM304controller *cntrl;

    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    /* Request the status of this motor */
    sprintf(command, "%dOS;", signal+1);
    send_recv_mess(card, command, response);
    Debug(2, "set_status, status query, card %d, response=%s\n", card, response);

    if (cntrl->model == MODEL_PM304) {
        /* The response string is an eight character string of ones and zeroes */

        if (strcmp(response, "00000000") == 0)
            motor_info->status &= ~RA_DONE;
        else {
            motor_info->status |= RA_DONE;
            if (drvPM304ReadbackDelay != 0.)
                epicsThreadSleep(drvPM304ReadbackDelay);
        }

        if (response[2] == '1')
            motor_info->status |= RA_PROBLEM;
        else
            motor_info->status &= ~RA_PROBLEM;

        motor_info->status &= ~(RA_PLUS_LS | RA_MINUS_LS);
        if (response[1] == '1') {
            motor_info->status |= RA_PLUS_LS;
            motor_info->status |= RA_DIRECTION;
	      ls_active = true;
        }
        if (response[0] == '1') {
            motor_info->status |= RA_MINUS_LS;
            motor_info->status &= ~RA_DIRECTION;
	      ls_active = true;
        }
    } else {
        /* The response string is 01: followed by an eight character string of ones and zeroes */
        strcpy(response, &response[3]);
        if (response[0] == '0')
            motor_info->status &= ~RA_DONE;
        else {
            motor_info->status |= RA_DONE;
            if (drvPM304ReadbackDelay != 0.)
                epicsThreadSleep(drvPM304ReadbackDelay);
        }

        if (response[1] == '1')
            motor_info->status |= RA_PROBLEM;
        else
            motor_info->status &= ~RA_PROBLEM;

        motor_info->status &= ~(RA_PLUS_LS | RA_MINUS_LS);
        if (response[2] == '1') {
            motor_info->status |= RA_PLUS_LS;
        }
        if (response[3] == '1') {
            motor_info->status |= RA_MINUS_LS;
        }
    }


    /* encoder status */
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_POSITION;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;

    /* Request the position of this motor */
    if (cntrl->use_encoder[signal]) {
        sprintf(command, "%dOA;", signal+1);
    } else {
        sprintf(command, "%dOC;", signal+1);
    }
    send_recv_mess(card, command, response);
    /* Parse the response string which is of the form "AP=10234" (PM304) or 01:10234 (PM600)*/
    motorData = atoi(&response[3]);
    Debug(2, "set_status, position query, card %d, response=%s\n", card, response);

    if (motorData == motor_info->position)
        motor_info->no_motion_count++;
    else
    {
        if (motorData >= motor_info->position)
            motor_info->status |= RA_DIRECTION;
        else
            motor_info->status &= ~RA_DIRECTION;
        motor_info->position = motorData;
        motor_info->encoder_position = motorData;
        motor_info->no_motion_count = 0;
    }

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0;

    if (!(motor_info->status & RA_DIRECTION))
        motor_info->velocity *= -1;

    rtn_state = (!motor_info->no_motion_count || ls_active == true ||
		 (motor_info->status & (RA_DONE | RA_PROBLEM))) ? 1 : 0;

    /* Test for post-move string. */
    if ((motor_info->status & RA_DONE || ls_active == true) && nodeptr != 0 &&
	nodeptr->postmsgptr != 0)
    {
        strcpy(buff, nodeptr->postmsgptr);
        strcat(buff, "\r");
        send_mess(card, buff, (char) NULL);
        nodeptr->postmsgptr = NULL;
    }

    Debug(2, "set_status, return value=%d\n", rtn_state);
    return (rtn_state);
}


/*****************************************************/
/* send a message to the PM304 board                 */
/* this function should be used when the PM304       */
/* response string should be ignorred.               */
/* It reads the response string, since the PM304     */
/* always sends one, but discards it.                */
/* Note that since it uses serialIOSendRecv it       */
/* flushes any remaining characters in the input     */
/* ring buffer                                       */
/* send_mess()                                       */
/*****************************************************/
STATIC RTN_STATUS send_mess(int card, const char *com, char c)
{
    char *p, *tok_save;
    char buff[BUFF_SIZE];
    char response[BUFF_SIZE];
    struct PM304controller *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("send_mess - invalid card #%d\n", card);
	return(ERROR);
    }

    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;

    /* Device support can send us multiple commands separated with ';'
     * characters.  The PM304 cannot handle more than 1 command on a line
     * so send them separately */
    for (p = strtok_r((char *) com, ";", &tok_save);
                ((p != NULL) && (strlen(p) != 0));
                p = strtok_r(NULL, ";", &tok_save)) {
        strcpy(buff, p);
        strcat(buff, OUTPUT_TERMINATOR);
        Debug(2, "send_mess: sending message to card %d, message=%s\n", card, buff);
	cntrl->serialInfo->serialIOSendRecv(buff, strlen(buff), response,
			    BUFF_SIZE, INPUT_TERMINATOR, SERIAL_TIMEOUT);
        Debug(2, "send_mess: card %d, response=%s\n", card, response);
    }

    return(OK);
}



/*****************************************************/
/* receive a message from the PM304 board            */
/* NOTE: This function is required by motordrvCom,   */
/* but it should not be used.  Use send_recv_mess    */
/* instead, since it sends a receives a message as   */
/* atomic operation.                                 */
/* recv_mess()                                       */
/*****************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    int timeout;
    int len=0;
    char *pos;
    char temp[BUFF_SIZE];
    struct PM304controller *cntrl;

    com[0] = '\0';
    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("resv_mess - invalid card #%d\n", card);
        return(-1);
    }

    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH)
        timeout = 0;
    else
        timeout = SERIAL_TIMEOUT;
    len = cntrl->serialInfo->serialIORecv(com, BUFF_SIZE, INPUT_TERMINATOR, timeout);

    /* The response from the PM304 is terminated with CR/LF.  Remove these */
    if (len < 2) com[0] = '\0'; else com[len-2] = '\0';
    if (len > 0) {
        Debug(2, "recv_mess: card %d, flag=%d, message = \"%s\"\n", card, flag, com);
    }
    if (len == 0) {
        if (flag != FLUSH)  {
            Debug(1, "recv_mess: card %d read ERROR: no response\n", card);
        } else {
            Debug(3, "recv_mess: card %d flush returned no characters\n", card);
        }
    }
    /* The PM600 always echoes the command sent to it, before sending the response.  It is terminated
       with a carriage return.  So we need to delete all characters up to and including the first
       carriage return */
    if (cntrl->model == MODEL_PM600) {
       pos = strchr(com, '\r');
       if (pos != NULL) {
           strcpy(temp, pos+1);
           strcpy(com, temp);
       }
    }
    return(strlen(com));
}



/*****************************************************/
/* Send a message to the PM304 board and receive a   */
/* response as an atomic operation.                  */
/* This function should be used when the PM304       */
/* response string is required.                      */
/* Note that since it uses serialIOSendRecv it       */
/* flushes any remaining characters in the input     */
/* ring buffer                                       */
/* send_recv_mess()                                  */
/*****************************************************/
STATIC int send_recv_mess(int card, const char *out, char *response)
{
    char *p, *tok_save;
    char buff[BUFF_SIZE];
    struct PM304controller *cntrl;
    int len=0;
    char *pos;
    char temp[BUFF_SIZE];

    response[0] = '\0';

    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("send_recv_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;

    /* Device support can send us multiple commands separated with ';'
     * characters.  The PM304 cannot handle more than 1 command on a line
     * so send them separately */
    for (p = strtok_r((char *) out, ";", &tok_save);
                ((p != NULL) && (strlen(p) != 0));
                p = strtok_r(NULL, ";", &tok_save)) {
        strcpy(buff, p);
        strcat(buff, OUTPUT_TERMINATOR);
        Debug(2, "send_recv_mess: sending message to card %d, message=%s\n", card, buff);
	len = cntrl->serialInfo->serialIOSendRecv(buff, strlen(buff), 
                         response, BUFF_SIZE, INPUT_TERMINATOR, SERIAL_TIMEOUT);
    }

    /* The response from the PM304 is terminated with CR/LF.  Remove these */
    if (len < 2) response[0] = '\0'; else response[len-2] = '\0';
    if (len > 0) {
        Debug(2, "send_recv_mess: card %d, response = \"%s\"\n", card, response);
    }
    if (len == 0) {
        Debug(1, "send_recv_mess: card %d ERROR: no response\n", card);
    }
    /* The PM600 always echoes the command sent to it, before sending the response.  It is terminated
       with a carriage return.  So we need to delete all characters up to and including the first
       carriage return */
    if (cntrl->model == MODEL_PM600) {
       pos = strchr(response, '\r');
       if (pos != NULL) {
           strcpy(temp, pos+1);
           strcpy(response, temp);
       }
    }
    return(strlen(response));
}



/*****************************************************/
/* Setup system configuration                        */
/* PM304Setup()                                      */
/*****************************************************/
RTN_STATUS
PM304Setup(int num_cards,   	/* maximum number of controllers in system */
           int num_channels,	/* NOT USED            */
           int scan_rate)       /* polling rate - 1/60 sec units */
{
    int itera;

    if (num_cards < 1 || num_cards > PM304_NUM_CARDS)
        PM304_num_cards = PM304_NUM_CARDS;
    else
        PM304_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
	targs.motor_scan_rate = scan_rate;
    else
	targs.motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structure pointers.  Note this must be done
    * before PM304Config is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem since this is just
    * an array of pointers.
    */
    motor_state = (struct controller **) malloc(PM304_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < PM304_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;
    return(OK);
}


/*****************************************************/
/* Configure a controller                            */
/* PM304Config()                                    */
/*****************************************************/
RTN_STATUS
PM304Config(int card,       	/* card being configured */
            int location,	/* card for RS-232 */
            const char *name,	/* server_task for RS-232 */
            int n_axes)         /* Number of axes */
{
    struct PM304controller *cntrl;

    if (card < 0 || card >= PM304_num_cards)
        return (ERROR);

    if (n_axes == 0) n_axes=1;  /* This is a new parameter, some startup files don't have it yet */
    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct PM304controller));
    cntrl = (struct PM304controller *) motor_state[card]->DevicePrivate;
    cntrl->serial_card = location;
    cntrl->n_axes = n_axes;
    cntrl->model = MODEL_PM304;  /* Assume PM304 initially */
    strcpy(cntrl->serial_task, name);
    return(OK);
}



/*****************************************************/
/* initialize all software and hardware              */
/* This is called from the initialization routine in */
/* device support.                                   */
/* motor_init()                                      */
/*****************************************************/
STATIC int motor_init()
{
    struct controller *brdptr;
    struct PM304controller *cntrl;
    int card_index, motor_index;
    char buff[BUFF_SIZE];
    char command[20];
    int total_axis = 0;
    bool success_rtn;

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (PM304_num_cards <= 0)
    {
        Debug(1, "motor_init: *PM304 driver disabled*\n");
        Debug(1, "PM304Setup() is missing from startup script.\n");
        return (ERROR);
    }

    for (card_index = 0; card_index < PM304_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        cntrl = (struct PM304controller *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        success_rtn = false;

	cntrl->serialInfo = new serialIO(cntrl->serial_card,
				     cntrl->serial_task, &success_rtn);

        if (success_rtn == true)
        {
            int retry = 0;

            /* Send a message to the board, see if it exists */
            /* flush any junk at input port - should not be any data available */
            do {
                recv_mess(card_index, buff, FLUSH);
            } while (strlen(buff) != 0);

            do
            {
                send_recv_mess(card_index, "1OA;", buff);
                retry++;
                /* Return value is length of response string */
            } while(strlen(buff) == 0 && retry < 3);
        }

        if (success_rtn == true && strlen(buff) > 0)
        {
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;
            /* Leave bdptr->cmnd_response false because we read each response */
            /* in send_mess and send_recv_mess. */
            brdptr->cmnd_response = false;

            /* Don't turn on motor power, too dangerous */
            /* send_mess(i, "1RSES;", buff); */
            send_mess(card_index, "1ST;", 0);     /* Stop motor */
            send_recv_mess(card_index, "1ID;", buff);    /* Read controller ID string */
            strncpy(brdptr->ident, buff, MAX_IDENT_LEN);
            /* Parse the response to figure out what model this is */
            if (strstr(brdptr->ident, "PM304") != NULL) {
                cntrl->model = MODEL_PM304;
            } else {
                cntrl->model = MODEL_PM600;
            }

            total_axis = cntrl->n_axes;
            brdptr->total_axis = total_axis;
            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];

                motor_info->motor_motion = NULL;
                motor_info->status = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
                /* Figure out if we have an encoder or not.  If so use 0A, else use OC for readback. */
                sprintf(command, "%dID", motor_index+1);
                send_recv_mess(card_index, command, buff);    /* Read controller ID string for this axis */
                if (cntrl->model == MODEL_PM304) {
                    /* For now always assume encoder if PM304 - needs work */
                    cntrl->use_encoder[motor_index] = 1; 
                } else {
                    /* PM600 ident string identifies open loop stepper motors */
                    if (strstr(buff, "Open loop stepper mode") != NULL) {
                        cntrl->use_encoder[motor_index] = 0;
                    } else {
                        cntrl->use_encoder[motor_index] = 1;
                    }
                }

                Debug(3, "PM304 motor_init(), cntrl->model=%d, cntrl->use_encoder[%d]=%d.\n", cntrl->model, motor_index, cntrl->use_encoder[motor_index]);

                set_status(card_index, motor_index);  /* Read status of each motor */
            }

        }
        else
            motor_state[card_index] = (struct controller *) NULL;
    }

    any_motor_in_motion = 0;

    Debug(3, "motor_init: spawning motor task\n");

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "tPM304", 64, 5000, (EPICSTHREADFUNC) motor_task, (void *) &targs);
    
    return(OK);
}