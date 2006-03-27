/*************************************************************************\
* ** Program:
**    devAoPmac.c
**
** Description:
**    Device support for the analogue out (Ao) record, using the PMAC driver code.
**
** Author:
**    Joy Morris
**
** $Id: devAoPmac.c,v 1.3 2005/05/18 09:19:36 mn Exp $
**
** $Name:  $
*************************************************************************/


#include	<vxWorks.h>
#include	<stdlib.h>
#include	<string.h>
#include	<stdio.h>
#include	<wdLib.h>
#include	<memLib.h>

#include	<alarm.h>
#include	<callback.h>
#include	<cvtTable.h>
#include	<dbDefs.h>
#include	<recGbl.h>
#include	<dbAccess.h>
#include	<recSup.h>
#include	<devSup.h>
#include	<link.h>
#include	<dbCommon.h>
#include	<aoRecord.h>

#include "epicsExport.h"

int send_mess(int card, char const *com, char *name);

/* Create the dset for devAoPmac */
static long init_record();
static long write_ao();
struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_ao;
        DEVSUPFUN	special_linconv;

}devAoPmac={
	6,
	NULL,
	NULL,
	init_record,
	NULL,
	write_ao,
	NULL
};
    
epicsExportAddress(dset,devAoPmac);
   
static long init_record(pao)
    struct aoRecord	*pao;
{
   
char  * MyName = "init_record pao";

   printf("delta tau ao: init_record\n");

   
    switch (pao->out.type) {
    case (VME_IO) :
      printf("%s: card %d signal %d parm %s\n",
	      MyName,
	      pao->out.value.vmeio.card,
	      pao->out.value.vmeio.signal,
	      pao->out.value.vmeio.parm);
	break;
    default :
	recGblRecordError(S_db_badField,(void *)pao,
		"devAoPmac (init_record) Illegal OUT field");
	return(S_db_badField);
    }
    return(2);
}

static long write_ao(pao)
    struct aoRecord	*pao;
{
    
    int ret;
    char buff[256]="";
    char * MyName = "write_ao";

    
    sprintf(buff, "%s=%f",pao->out.value.vmeio.parm,pao->val);
    printf("%s: %s \n",MyName, buff);
    
    switch (pao->out.type) {
    case (VME_IO) :
      ret = send_mess( 0, buff, NULL);
      printf("%s: return = 0x%x\n",MyName,ret);
      /*      recv_mess(0,buff,1);
	      printf("recv_mess: %s\n",buff); */
      break;
    default :
        if(recGblSetSevr(pao,SOFT_ALARM,INVALID_ALARM)){
		if(pao->stat!=SOFT_ALARM) {
			recGblRecordError(S_db_badField,(void *)pao,
			   "devAoPmac (read_ao) Illegal OUT field");
		}
	}
    }
    return(0);
}
