/******************************************************************************
** Program:
**    devSiPmac.c
**
** Description:
**    Device support for the stringin record, using the PMAC driver code.
**
** Author:
**    Martin Norbury
**
** $Id: devSiPmac.c,v 1.3 2005/05/18 09:19:36 mn Exp $
**
** $Name:  $
******************************************************************************/
#include	<vxWorks.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<wdLib.h>
#include	<memLib.h>
#include	<string.h>

#include	<alarm.h>
#include	<callback.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<link.h>
#include	<dbCommon.h>
#include	<stringinRecord.h>

#include <epicsExport.h>

SEM_ID *semptr;

/* Create the dset for devSiTestAsyn */
static long init_record();
static long read_stringin();
struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_stringin;
	DEVSUPFUN	special_linconv;
}devSiPmac={
	6,
	NULL,
	NULL,
	init_record,
	NULL,
	read_stringin,
	NULL};

epicsExportAddress(dset,devSiPmac);


int send_mess(int card, char const *com, char *name);
int recv_mess(int, char *, int);

static long init_record(pstringin)
struct stringinRecord	*pstringin;
{
  char * MyName = "init_record";
  
  printf("MN: stringin init_record\n");

  
  printf("init_record: semptr = 0x%x\n",semptr);

  /* stringin.inp must be a CONSTANT*/
  switch (pstringin->inp.type) 
    {
  
    case (CONSTANT) :
      printf("CONSTANT\n");
      break;

    case (VME_IO):
      printf("VME_IO\n");
      printf("%s: card %d signal %d parm %s\n",
	     MyName,
	     pstringin->inp.value.vmeio.card,
	     pstringin->inp.value.vmeio.signal,
	     pstringin->inp.value.vmeio.parm);
      break;
      
    default :
      printf("Error invalid type\n");

    }
    return(0);
}

static long read_stringin(pstringin)
struct stringinRecord	*pstringin;
{
  int ret;
  char buff[256];
  char * MyName = "read_stringin";

  printf("Inside read_stringin\n");

  printf("Send %s\n",pstringin->inp.value.vmeio.parm);
  
  printf("STRINGIN HAS TAKEN THE SEMAPHORE\n");
  ret = send_mess(0,pstringin->inp.value.vmeio.parm,NULL);
  printf("%s: return = 0x%x\n",MyName,ret);
  
  recv_mess(0,buff,1);
  printf("recv_mess: %s\n",buff);
  
  strcpy(pstringin->val,buff);
  
  return(0);
}
