/*
FILENAME...	devPIC844.c
USAGE...	Motor record device level support for Physik Instrumente (PI)
		GmbH & Co. C-844 motor controller.

Version:	1.1.2.2
Modified By:	sluiter
Last Modified:	2004/01/27 20:34:18
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
 * .01 12/17/03 rls - copied from devIM483PL.c
 * .02 01/22/03 rls - support PID commands.
 */


#include	<vxWorks.h>
#include	<stdioLib.h>
#include	<string.h>
#include        <semLib.h>	/* jps: include for init_record wait */
#include	<logLib.h>

#ifdef __cplusplus
extern "C" {
#include	<epicsDynLink.h>
}
#else
#include	<epicsDynLink.h>
#endif
#include	<sysSymTbl.h>	/* for sysSymTbl*/

#include	<alarm.h>
#include	<callback.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<dbCommon.h>
#include	<fast_lock.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<drvSup.h>

#include	"motorRecord.h"
#include	"motor.h"
#include	"motordevCom.h"
#include	"drvPI.h"

#define STATIC static

/* ----------------Create the dsets for devPIC844----------------- */
STATIC struct driver_table *drvtabptr;
STATIC long PIC844_init(int);
STATIC long PIC844_init_record(struct motorRecord *);
STATIC long PIC844_start_trans(struct motorRecord *);
STATIC long PIC844_build_trans(motor_cmnd, double *, struct motorRecord *);
STATIC long PIC844_end_trans(struct motorRecord *);

struct motor_dset devPIC844 =
{
    {8, NULL, PIC844_init, PIC844_init_record, NULL},
    motor_update_values,
    PIC844_start_trans,
    PIC844_build_trans,
    PIC844_end_trans
};


/* --------------------------- program data --------------------- */

/* This table is used to define the command types */
/* WARNING! this must match "motor_cmnd" in motor.h */

static int PIC844_table[] = {
    MOTION, 	/* MOVE_ABS */
    MOTION, 	/* MOVE_REL */
    MOTION, 	/* HOME_FOR */
    MOTION, 	/* HOME_REV */
    IMMEDIATE,	/* LOAD_POS */
    IMMEDIATE,	/* SET_VEL_BASE */
    IMMEDIATE,	/* SET_VELOCITY */
    IMMEDIATE,	/* SET_ACCEL */
    IMMEDIATE,	/* GO */
    IMMEDIATE,	/* SET_ENC_RATIO */
    INFO,	/* GET_INFO */
    MOVE_TERM,	/* STOP_AXIS */
    VELOCITY,	/* JOG */
    IMMEDIATE,	/* SET_PGAIN */
    IMMEDIATE,	/* SET_IGAIN */
    IMMEDIATE,	/* SET_DGAIN */
    IMMEDIATE,	/* ENABLE_TORQUE */
    IMMEDIATE,	/* DISABL_TORQUE */
    IMMEDIATE,	/* PRIMITIVE */
    IMMEDIATE,	/* SET_HIGH_LIMIT */
    IMMEDIATE,	/* SET_LOW_LIMIT */
    VELOCITY	/* JOG_VELOCITY */
};


static struct board_stat **PIC844_cards;

/* --------------------------- program data --------------------- */


/* initialize device support for PIC844 stepper motor */
STATIC long PIC844_init(int after)
{
    SYM_TYPE type;
    long rtnval;

    if (after == 0)
    {
	rtnval = symFindByNameEPICS(sysSymTbl, "_PIC844_access",
		    (char **) &drvtabptr, &type);
	if (rtnval != OK)
	    return(rtnval);
    /*
	IF before DB initialization.
	    Initialize PIC844 driver (i.e., call init()). See comment in
		drvPIC844.c init().
	ENDIF
    */
	(drvtabptr->init)();
    }

    rtnval = motor_init_com(after, *drvtabptr->cardcnt_ptr, drvtabptr, &PIC844_cards);
    return(rtnval);
}


/* initialize a record instance */
STATIC long PIC844_init_record(struct motorRecord *mr)
{
    return(motor_init_record_com(mr, *drvtabptr->cardcnt_ptr, drvtabptr, PIC844_cards));
}


/* start building a transaction */
STATIC long PIC844_start_trans(struct motorRecord *mr)
{
    motor_start_trans_com(mr, PIC844_cards);
    return(OK);
}


/* end building a transaction */
STATIC long PIC844_end_trans(struct motorRecord *mr)
{
    motor_end_trans_com(mr, drvtabptr);
    return(OK);
}


/* add a part to the transaction */
STATIC long PIC844_build_trans(motor_cmnd command, double *parms, struct motorRecord *mr)
{
    struct motor_trans *trans = (struct motor_trans *) mr->dpvt;
    struct mess_node *motor_call;
    struct controller *brdptr;
    struct PIC844controller *cntrl;
    char buff[110];
    int axis, card, maxdigits, size;
    double dval, cntrl_units;
    long rtnval;
    BOOLEAN send;

    send = ON;		/* Default to send motor command. */
    rtnval = OK;
    buff[0] = '\0';
    
    /* Protect against NULL pointer with WRTITE_MSG(GO/STOP_AXIS/GET_INFO, NULL). */
    dval = (parms == NULL) ? 0.0 : *parms;

    motor_call = &(trans->motor_call);
    card = motor_call->card;
    axis = motor_call->signal + 1;
    brdptr = (*trans->tabptr->card_array)[card];
    if (brdptr == NULL)
	return(rtnval = ERROR);

    cntrl = (struct PIC844controller *) brdptr->DevicePrivate;
    cntrl_units = dval;
    maxdigits = 2;
    
    if (PIC844_table[command] > motor_call->type)
	motor_call->type = PIC844_table[command];

    if (trans->state != BUILD_STATE)
	return(rtnval = ERROR);

    if (command == PRIMITIVE && mr->init != NULL && strlen(mr->init) != 0)
	strcat(motor_call->message, mr->init);

    switch (command)
    {
	case MOVE_ABS:
	case MOVE_REL:
	case HOME_FOR:
	case HOME_REV:
	case JOG:
	    if (strlen(mr->prem) != 0)
	    {
		strcat(motor_call->message, mr->prem);
		strcat(motor_call->message, ";");
	    }
	    if (strlen(mr->post) != 0)
		motor_call->postmsgptr = (char *) &mr->post;
	    break;
        
	default:
	    break;
    }


    switch (command)
    {
	case MOVE_ABS:
	    sprintf(buff, "TARG %.*f", maxdigits, cntrl_units);
	    break;
	
	case MOVE_REL:
	    sprintf(buff, "TARG:RPOS %+.*f", maxdigits, cntrl_units);
	    break;
	
	case HOME_FOR:
	    sprintf(buff, "TARG:FIND POS");
	    break;
	case HOME_REV:
	    sprintf(buff, "TARG:FIND NEG");
	    break;
	
	case LOAD_POS:
	    sprintf(buff, "AXIS:POS %+.*f;TARG CURR", maxdigits, cntrl_units);
	    break;
	
	case SET_VEL_BASE:
	    send = OFF;		/* DC motor; not base velocity. */
	    break;
	
	case SET_VELOCITY:
	    sprintf(buff, "MVEL %.*f;", maxdigits, cntrl_units);
	    break;
	
	case SET_ACCEL:
	    sprintf(buff, "ACC %.*f;", maxdigits, cntrl_units);
	    break;
	
	case GO:
	    /* The PIC844 starts moving immediately on move commands, GO command
	     * does nothing. */
	    send = OFF;
	    break;
	
	case PRIMITIVE:
	case GET_INFO:
	    /* These commands are not actually done by sending a message, but
	       rather they will indirectly cause the driver to read the status
	       of all motors */
	    break;
	
	case STOP_AXIS:
	    sprintf(buff, "HALT");
	    break;
	
	case JOG_VELOCITY:
	case JOG:
	    sprintf(buff, "TARG:VEL %.*f", maxdigits, cntrl_units);
	    break;
	
	case SET_PGAIN:
	    cntrl_units *= 32767;
	    sprintf(buff, "PID %.*f,,", maxdigits, cntrl_units);
	    break;
	case SET_IGAIN:
	    cntrl_units *= 32767;
	    sprintf(buff, "PID ,%.*f,", maxdigits, cntrl_units);
	    break;
	case SET_DGAIN:
	    cntrl_units *= 32767;
	    sprintf(buff, "PID ,,%.*f", maxdigits, cntrl_units);
	    break;
	
	case ENABLE_TORQUE:
	    sprintf(buff, "AXIS:STAT ON");
	    break;
	
	case DISABL_TORQUE:
	    sprintf(buff, "AXIS:STAT OFF");
	    break;
	
	case SET_HIGH_LIMIT:
	case SET_LOW_LIMIT:
	case SET_ENC_RATIO:
	    trans->state = IDLE_STATE;	/* No command sent to the controller. */
	    send = OFF;
	    break;
	
	default:
	    send = OFF;
	    rtnval = ERROR;
    }

    size = strlen(buff);
    if (send == OFF)
	return(rtnval);
    else if (size > sizeof(buff) || (strlen(motor_call->message) + size) > MAX_MSG_SIZE)
	logMsg((char *) "PIC844_build_trans(): buffer overflow.\n", 0, 0, 0, 0, 0, 0);
    else
	strcat(motor_call->message, buff);
    return(rtnval);
}
