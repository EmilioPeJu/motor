/* File: drvMicos.c                     */

/* Device Driver Support routines for motor */
/*
 *      Original Author: Kurt Goetze
 *      Date: 11/24/2003
 *
 * Modification Log:
 * -----------------
 * .00  11-24-2003   kag  initialized from drvMCB4B.c
 * .01  02-06-2004   rls  Eliminate erroneous "Motor motion timeout ERROR".
 */


#include        <vxWorks.h>
#include        <stdioLib.h>
#include        <sysLib.h>
#include        <string.h>
#include        <taskLib.h>
#include        <tickLib.h>
#include        <rngLib.h>
#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbAccess.h>
#include        <fast_lock.h>
#include        <recSup.h>
#include        <devSup.h>
#include        <drvSup.h>
#include        <errMdef.h>
#include        <logLib.h>

#include        "motor.h"
#include        "drvMicos.h"
#include        "serialIO.h"

#define STATIC static

#define WAIT 1

#define SERIAL_TIMEOUT 2000 /* Command timeout in msec */

#define BUFF_SIZE 100       /* Maximum length of string to/from Micos */

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};


#ifdef NODEBUG
#define Debug(L,FMT,V) ;
#else
#define Debug(L,FMT,V...) {  if(L <= drvMicosDebug) \
                        { errlogPrintf("%s(%d):",__FILE__,__LINE__); \
                          errlogPrintf(FMT,##V); } }
#endif

/* Debugging notes:
 *   drvMicosDebug == 0  No debugging information is printed
 *   drvMicosDebug >= 1  Warning information is printed
 *   drvMicosDebug >= 2  Time-stamped messages are printed for each string 
 *                       sent to and received from the controller
 *   drvMicosDebug >= 3  Additional debugging messages
 */    

volatile int Micos_num_cards = 0;
volatile int Micos_num_axis = 0;
volatile int drvMicosDebug = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include        "motordrvComCode.h"


/*----------------functions-----------------*/
STATIC int recv_mess(int, char *, int);
STATIC int send_mess(int card, const char *com, char c);
STATIC void start_status(int card);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/

struct driver_table Micos_access =
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
} drvMicos = {2, report, init};




/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{
  int card;

  if (Micos_num_cards <=0)
    printf("    NO Micos controllers found\n");
  else
    {
      for (card = 0; card < Micos_num_cards; card++)
          if (motor_state[card])
             printf("    Micos controller group %d, id: %s \n",
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
    if (Micos_num_cards <= 0)
    {
        Debug(1, "init: *Micos driver disabled*\n");
        Debug(1, "MicosSetup() is missing from startup script.\n");
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
    /* The Micos cannot query status or positions of all axes with a
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
    int rtn_state, i, j;
    long motorData;
    long bytes[7];
    char temp[5];
    char buff[BUFF_SIZE];
    BOOLEAN ls_active = OFF;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;

    /* Request the moving status of this motor */

    /* MM4000 has a delay in case the motor is not done due to servo PID-related settling.. do we need this delay? */
    /* ...after some testing, it doesn't look like we need the delay */

    /* Get the motor status (ts) */
    sprintf(command, "%c%dts", CTLA, signal);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);
    /* The response string is of the form "byte0 byte1 ... byte6" */

    /* Convert ASCII characters to hex */
    temp[0]='0'; temp[1]='x'; temp[2]='0', temp[3]='0', temp[4]='\0';
    if (signal > 9) j = 5;
    else j = 4;
    for (i = 0; i < 7; i++) {
        temp[2] = response[j];
        temp[3] = response[j+1];
        bytes[i] = strtol(temp, (char **)NULL, 0);
        j += 3;
    }
    /* check to see if motor is moving */
    if (bytes[0] & 0x04)
        motor_info->status |= RA_DONE;
    else
        motor_info->status &= ~RA_DONE;

    /* check limits */
    motor_info->status &= ~(RA_PLUS_LS | RA_MINUS_LS);
    if ((bytes[5] & 0x04) & (bytes[3] & 0x04)) {  /* if +lim AND pos move */
        motor_info->status |= RA_PLUS_LS;
	ls_active = ON;
    }
    if ((bytes[5] & 0x01) & !(bytes[3] & 0x04)) {  /* if -lim AND neg move */
        motor_info->status |= RA_MINUS_LS;
	ls_active = ON;
    }

    /* encoder status */
    motor_info->status &= ~EA_SLIP;
    motor_info->status &= ~EA_POSITION;
    motor_info->status &= ~EA_SLIP_STALL;
    motor_info->status &= ~EA_HOME;
    if ((bytes[3] & 0x08) | (bytes[3] & 0x40)) {
        printf("drvMicos: set_status: EA_SLIP_STALL = 1, %ld\n", bytes[3]);
        motor_info->status |= EA_SLIP_STALL;
    }

    /* Request the position of this motor */
    sprintf(command, "%c%dtp", CTLA, signal);
    send_mess(card, command, 0);
    recv_mess(card, response, WAIT);
    /* The response string is of the form "0P0:+00001000" */
    if (signal > 9)
        motorData = atoi(&response[5]);
    else
        motorData = atoi(&response[4]);

    /* derive direction information */
    if (motorData == motor_info->position)
    {
	if (nodeptr != 0)	/* Increment counter only if motor is moving. */
	    motor_info->no_motion_count++;
    }
    else
    {
        if (bytes[3] & 0x04)
            motor_info->status |= RA_DIRECTION;
        else
            motor_info->status &= ~RA_DIRECTION;
        motor_info->position = motorData;
        motor_info->encoder_position = motorData;
        motor_info->no_motion_count = 0;
    }

    /* Parse motor velocity? */
    /* NEEDS WORK */

    motor_info->velocity = 0.;

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
        /* The Micos will not send back a response for a 'set' command, don't need next line */
        /* recv_mess(card, buff, WAIT); */
        nodeptr->postmsgptr = NULL;
    }

    return (rtn_state);
}


/*****************************************************/
/* send a message to the Micos board                 */
/* send_mess()                                       */
/*****************************************************/
STATIC int send_mess(int card, const char *com, char c)
{
    char buff[BUFF_SIZE];
    struct MicosController *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("send_mess - invalid card #%d\n", card);
        return (-1);
    }

    /* If the string is NULL just return */
    if (strlen(com) == 0) return(OK);
    cntrl = (struct MicosController *) motor_state[card]->DevicePrivate;

    strcpy(buff, com);
    strcat(buff, OUTPUT_TERMINATOR);
    Debug(2, "%.2f : send_mess: sending message to card %d, message=%s\n",
                    tickGet()/60., card, buff);
    serialIOSend(cntrl->serialInfo, buff, strlen(buff), SERIAL_TIMEOUT);

    return (OK);
}


/*****************************************************/
/* Read a response string from the Micos board */
/* recv_mess()                                       */
/*****************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    int timeout;
    int len=0;
    struct MicosController *cntrl;

    /* Check that card exists */
    if (!motor_state[card])
    {
        errlogPrintf("recv_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct MicosController *) motor_state[card]->DevicePrivate;

    Debug(3, "%.2f : recv_mess entry: card %d, flag=%d\n", 
            tickGet()/60., card, flag);
    if (flag == FLUSH)
        timeout = 0;
    else
        timeout = SERIAL_TIMEOUT;
    len = serialIORecv(cntrl->serialInfo, com, BUFF_SIZE,
                       INPUT_TERMINATOR, timeout);

    /* The response from the Micos is terminated with <CR><LF><ETX>.  Remove */
    if (len < 3) com[0] = '\0'; 
    else com[len-3] = '\0';
    if (len > 0) {
        Debug(2, "%.2f : recv_mess: card %d, message = \"%s\"\n", 
            tickGet()/60., card, com);
    }
    if (len == 0) {
        if (flag != FLUSH)  {
            Debug(1, "%.2f: recv_mess: card %d ERROR: no response\n", 
                tickGet()/60., card);
        } else {
            Debug(3, "%.2f: recv_mess: card %d flush returned no characters\n", 
                tickGet()/60., card);
        }
    }
    return (len);
}



/*****************************************************/
/* Setup system configuration                        */
/* MicosSetup()                                     */
/*****************************************************/
int MicosSetup(int num_cards,   /* maximum number of "controllers" in system */
            int num_channels,   /* max number of drivers            */
           int scan_rate)       /* polling rate - 1/60 sec units */
{
    int itera;

    if (num_cards < 1 || num_cards > MICOS_NUM_CARDS)
        Micos_num_cards = MICOS_NUM_CARDS;
    else
        Micos_num_cards = num_cards;

    if (num_channels < 1 || num_channels > MICOS_NUM_AXIS)
        Micos_num_axis = MICOS_NUM_AXIS;
    else
        Micos_num_axis = num_channels;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= sysClkRateGet())
        motor_scan_rate = sysClkRateGet() / scan_rate;
    else
        motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structure pointers.  Note this must be done
    * before MicosConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem since this is just
    * an array of pointers.
    */
    motor_state = (struct controller **) malloc(Micos_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < Micos_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;
    return (0);
}


/*****************************************************/
/* Configure a controller                            */
/* MicosConfig()                                    */
/*****************************************************/
int MicosConfig(int card,       /* "controller" being configured */
            int addr1,          /* card for RS-232 */
            int addr2)          /* server_task for RS-232 */
{
    struct MicosController *cntrl;

    if (card < 0 || card >= Micos_num_cards)
        return (ERROR);

    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct MicosController));
    cntrl = (struct MicosController *) motor_state[card]->DevicePrivate;
    cntrl->serial_card = addr1;
    strcpy(cntrl->serial_task, (char *) addr2);
    return (0);
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
    struct MicosController *cntrl;
    int card_index, motor_index, arg3, arg4;
    char cmd[BUFF_SIZE];
    char buff[BUFF_SIZE];
    int total_axis = 0;
    int status = 0;
    BOOLEAN errind;

    initialized = ON;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (Micos_num_cards <= 0)
    {
        Debug(1, "motor_init: *Micos driver disabled*\n");
        Debug(1, "MicosSetup() is missing from startup script.\n");
        return (ERROR);
    }

    for (card_index = 0; card_index < Micos_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        cntrl = (struct MicosController *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        errind = OFF;

        cntrl->serialInfo = serialIOInit(cntrl->serial_card,
                                         cntrl->serial_task);
        if (cntrl->serialInfo == NULL) 
            errind = ON;

        if (errind == OFF)
        {
            int retry = 0;

            /* Each "controller" can have max 16 axes. */
            total_axis = Micos_num_axis;
            brdptr->total_axis = total_axis;

            /* flush any junk at input port - should not be any data available */
            do {
                recv_mess(card_index, buff, FLUSH);
            } while (strlen(buff) != 0);

            /* Send a message to each Micos driver.  See if it responds */
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                do
                {
                    sprintf(cmd, "%c%dts", CTLA, motor_index);
                    send_mess(card_index, cmd, 0);
                    status = recv_mess(card_index, buff, WAIT);
                    retry++;
                    /* Return value is length of response string */
                } while(status == 0 && retry < 3);
                if (status == 0) break;
            }
        }


        if (errind == OFF && status > 0)
        {
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;
            brdptr->cmnd_response = OFF;

            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];
                brdptr->motor_info[motor_index].motor_motion = NULL;
                /* turn off echo */
                sprintf(buff, "%c%def", CTLA, motor_index);
                send_mess(card_index, buff, 0);
                /* Don't turn on motor power, too dangerous */
		   /*sprintf(buff,"#%02dW=1", motor_index); */
                /* send_mess(card_index, buff, 0); */
                /* Stop motor */
                sprintf(buff,"%c%dab1", CTLA, motor_index);
                send_mess(card_index, buff, 0);
		   /* recv_mess(card_index, buff, WAIT);    Throw away response */
                strcpy(brdptr->ident, "MICOS");

                motor_info->status = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;

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
        arg3 = (int) (&Micos_access);
        arg4 = 0;
    }
    else
    {
        arg3 = (int) ((long) &Micos_access >> 16);
        arg4 = (int) ((long) &Micos_access & 0xFFFF);
    }
    Debug(3, "motor_init: spawning motor task\n");
    taskSpawn((char *) "tMicos", 64, VX_FP_TASK | VX_STDIO, 5000, motor_task,
              motor_scan_rate, arg3, arg4, 0, 0, 0, 0, 0, 0, 0);
    return (0);
}
