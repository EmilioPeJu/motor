/*
FILENAME: motordrvComCode.h
USAGE... This file contains local variables that are allocated
	in each motor record driver.  The variables are allocated
	in each driver by including this file.

Version:	1.1.10.1
Modified By:	sluiter
Last Modified:	2003/12/11 19:32:10
*/

/*
 *      Original Author: Ron Sluiter
 *      Date: 08/20/99
 *
 *      Experimental Physics and Industrial Control System (EPICS)
 *
 *      Copyright 1991, the University of Chicago Board of Governors.
 *
 *      This software was produced under  U.S. Government contract
 *      W-31-109-ENG-38 at Argonne National Laboratory.
 *
 *      Beamline Controls & Data Acquisition Group
 *      Experimental Facilities Division
 *      Advanced Photon Source
 *      Argonne National Laboratory
 *
 * Modification Log:
 * -----------------
 */

#ifndef	INCmotordrvComCode
#define	INCmotordrvComCode 1

/* --- Local data common to each driver. --- */
static volatile int motor_scan_rate = SCAN_RATE;
static struct controller **motor_state;
static int total_cards;
static int any_motor_in_motion;
static struct circ_queue mess_queue;	/* in message queue head */
static FAST_LOCK queue_lock;
static struct circ_queue free_list;
static FAST_LOCK freelist_lock;
static SEM_ID motor_sem;
static BOOLEAN initialized = OFF;	/* Driver initialized indicator. */

#endif	/* INCmotordrvComCode */
