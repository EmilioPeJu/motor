/******************************************************************************
**
** Program:
**    devSimPmac.h
**
** Description:
**    Header file for devSimPmac.c
**
** Author:
**    Martin Norbury (1/3/05)
**
** $Id: devSimPmac.h,v 1.2 2005/04/13 15:58:13 mn Exp $
**
** $Name:  $
** 
** $Log: devSimPmac.h,v $
** Revision 1.2  2005/04/13 15:58:13  mn
** Modifications to incorporate a simulation layer
**
** Revision 1.1  2005/03/07 09:21:53  mn
** First import to CVS
**
**
******************************************************************************/
#ifndef INC_SIM_H
#define INC_SIM_H

/* Standard include files */
#include        <vxWorks.h>
#include        <stdlib.h>
#include        <stdio.h>
#include        <string.h>
#include        <assert.h>
 
#include        <alarm.h>
#include        <dbDefs.h>
#include        <dbAccess.h>
#include        <recSup.h>
#include        <devSup.h>
#include        <module_types.h>
#include        <stringoutRecord.h>

#include "drvPmac.h"


#ifdef DEVSIMPMAC_C
#define EXTERN
#else
#define EXTERN extern
#endif

#define MAXTOK     20
#define MAXTOKLEN  50
#define MAX_CARDS  1
#define MAX_AXIS   32
#define MSEC       1000

/* Control character responses. */
#define CMNDERR	0x03
#define ACK	0x06
#define CR	0x0D

/* I-Variable definitions */
#define IVAR_ACTIVATION 0
#define IVAR_JOGACCTIME 20
#define IVAR_JOGSCRVTIME 21

#define MVAR_COMMANDPOS 61
#define MVAR_ACTUALPOS 62


#define TYPEREPLY "PMAC SIMULATION"
#define VERSIONREPLY "1.00"

enum state
{
  STOPPED,
  MOVING,
  PLIMIT,
  NLIMIT
};


/* Local structures */
typedef struct simmotor_t
{
  int    act;                  /* Activation state */
  int    state;
  int    status;               /* Status */
  int    dempos;               /* Demand position */
  int    actpos;               /* Actual position */
  float  vel;                  /* Velocity (counts/msec) */
  float  acc;                  /* Acceleration (counts/msec^2)*/
  float  time;           /* Time of motion */
} simmotor;

/* Function prototypes */

EXTERN void simmotor_task( int a1, 
                           int a2, 
                           int a3, 
                           int a4, 
                           int a5, 
                           int a6,
                           int a7, 
                           int a8,
                           int a9,
                           int a10
			   );

EXTERN void stopAxis( struct pmac_dpram * );
EXTERN void getStatus( struct pmac_dpram * );
EXTERN void setAxis( struct pmac_dpram * );
EXTERN void setIvariable( struct pmac_dpram * );
EXTERN void getIvariable( struct pmac_dpram * );
EXTERN void getMvariable( struct pmac_dpram * );
EXTERN void getVersion( struct pmac_dpram * );
EXTERN void getType( struct pmac_dpram * );
EXTERN void pmacmessage( struct pmac_dpram * );
EXTERN char *calcnewpos( int axis );

EXTERN int   MotorConfig(int card, 
			int axis
			);


#endif

