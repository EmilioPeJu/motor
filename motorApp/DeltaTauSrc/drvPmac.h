/*
FILENAME...	drvPmac.h
USAGE... This file contains Delta Tau PMAC driver "include" information.

Version:	1.2
Modified By:	sluiter
Last Modified:	2004/09/15 18:48:35
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
 * Modification Log:
 * -----------------
 * 00 04/16/04 rls - Copied from drvOms.h
 */

#ifndef	INCdrvPmach
#define	INCdrvPmach 1

#include <devLib.h>
#include "motor.h"
#include "motordrvCom.h"

/* Define for return test on devNoResponseProbe() */
#define PROBE_SUCCESS(STATUS) ((STATUS)==S_dev_addressOverlap)

#define BUFF_SIZE 100       /* Maximum length of string to/from PMAC */

/* Default profile. */

#define Pmac_NUM_CARDS		10	/* Maximum number of cards. */
#define Pmac_BRD_SIZE		0x4000	/* card address boundary */
#define Pmac_MAX_AXES		32
#define Pmac_INTERRUPT_TYPE	intVME
#define Pmac_INT_VECTOR		180	/* default interrupt vector (64-255) */
#define Pmac_INT_LEVEL		5	/* default interrupt level (1-6) */

/* PMAC Commands. */

#define AXIS_STOP "\\"

struct PMACcontroller
{
    int	status;
    bool irqEnable;
    char *mbox_addr;
    char *dpram_addr;
    int interrupt_vector;
    int interrupt_level;
    epicsAddressType addr_type;
    double pos_scaleFac[Pmac_MAX_AXES];	/* Position scale factor (Ixx08 * 32). */
};

typedef enum
{
    w1_maxrapid_speed,
    w1_alt_cmndout_mode,
    w1_soft_pos_capture,
    w1_error_trigger,
    w1_follow_enable,
    w1_follow_offset,
    w1_phased_motor,
    w1_alt_src_dest,
    w1_user_servo,
    w1_user_phase,
    w1_homing,
    w1_block_request,
    w1_decel_abort,
    w1_CmndVelZero,
    w1_DataBlkErr,
    w1_dwell,
    w1_integrate_mode,
    w1_move_time_on,
    w1_open_loop,
    w1_amp_enabled,
    w1_x_servo_on,
    w1_pos_limit_set,
    w1_neg_limit_set,
    w1_motor_on
} pmacStatusWordOneBits;

typedef enum
{
    w2_in_position,
    w2_warn_follow_err,
    w2_err_follow_err,
    w2_amp_fault,
    w2_neg_backlash,
    w2_i2t_amp_fault,
    w2_i2_follow_err,
    w2_trigger_move,
    w2_phase_ref_err,
    w2_phase_search,
    w2_home_complete,
    w2_pos_limit_stop,
    w2_desired_stop,
    w2_fore_in_pos,
    w2_na14,
    w2_assigned_CS
} pmacStatusWordTwoBits;


typedef union
{
    volatile epicsUInt16 All;
    struct
    {
        volatile epicsUInt8 term;
        volatile epicsUInt8 ctrl;
#if 0
#ifdef MSB_First
        epicsUInt16 cntrl_char :8;  /* Response control charcter. */
        epicsUInt16 type       :2;  /* Response type. */
        epicsUInt16 na1        :5;  /* n/a bit #1-4. */
        epicsUInt16 error      :1;  /* Error indicator. */
#else
        epicsUInt16 error      :1;  /* Error indicator. */
        epicsUInt16 na1        :5;  /* n/a bit #1-4. */
        epicsUInt16 type       :2;  /* Response type. */
        epicsUInt16 cntrl_char :8;  /* Response control charcter. */
#endif
#endif
    } Bits;                                
} REPLY_STATUS;


/* PMAC DPRAM structure. */
struct pmac_dpram
{
    epicsUInt8 na0[0xE9C];
    volatile epicsUInt8 out_cntrl_wd;	/* Control Word at 0x0E9C. */
    epicsUInt8 na1;
    volatile epicsUInt16 out_cntrl_char;	/* Control Character at 0x0E9E. */
    volatile epicsUInt8 cmndbuff[160];	/* Command Buffer at 0x0EA0. */
    REPLY_STATUS reply_status;	        /* Response Buffer Control Characters. */
    volatile epicsUInt8 reply_count;	/* Response Character count - 1. */
    epicsUInt8 na2;
    volatile epicsUInt8 response[256];	/* Response Buffer at 0x0F44. */
};

typedef struct simargs_s
{
  int numcards;
  struct pmac_dpram *pdpram[Pmac_NUM_CARDS];
} simargs_t;

/* Driver status bits (MN 13/4/05) */
#define PM_MOTPROGRAM 0x4000
#define PM_AXISHOMED  0x8000
/* Driver status bits (MN 13/4/05) */
#define PM_MOTPROGRAM   0x4000
#define PM_AXISHOMED    0x8000
#define PM_MOTACTIVATE  0x10000
#define PM_AMPENABLE    0x20000
#define PM_MOTTIMACT    0x40000
#define PM_DWELLACT     0x80000
#define PM_DATAERR      0x100000
#define PM_DESVELZERO   0x200000
#define PM_HOMESEARCH   0x400000
#define PM_POSLIMIT     0x800000
#define PM_HOMCOMPLETE  0x1000000
#define PM_TRIGGERMOVE  0x2000000
#define PM_INTFOLERR    0x4000000
#define PM_AMPFAULT     0x8000000
#define PM_FOLERR       0x10000000
#define PM_INPOSITION   0x20000000


#endif	/* INCdrvPmach */
