/******************************************************************************
** Program:
**    devSoPmac.c
**
** Description:
**    Device support for the stringout record, using the PMAC driver code.
**
** Author:
**    Martin Norbury
**
** $Id: devSoPmac.c,v 1.3 2005/05/18 09:19:36 mn Exp $
**
** $Name:  $
******************************************************************************/
#include	<vxWorks.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>

#include	<alarm.h>
#include	<dbDefs.h>
#include	<dbAccess.h>
#include        <recSup.h>
#include	<devSup.h>
#include	<module_types.h>
#include	<stringoutRecord.h>

#include <epicsExport.h>

int send_mess(int card, char const *com, char *name);
int recv_mess(int, char *, int);

static long init_record();
static long write_stringout();
struct {
	long		number;
	DEVSUPFUN	report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record;
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_stringout;
}devSoPmac={
	5,
        NULL,
	NULL,
	init_record,
	NULL,
	write_stringout
};

epicsExportAddress(dset,devSoPmac);

static long init_record(pstringout)
struct stringoutRecord *pstringout;
{
  char  * MyName = "init_record";

   printf("MN: init_record\n");

   switch(pstringout->out.type)
     {
     case (VME_IO):
       printf("VME_IO\n");
       printf("%s: card %d signal %d parm %s\n",
	      MyName,
	      pstringout->out.value.vmeio.card,
	      pstringout->out.value.vmeio.signal,
	      pstringout->out.value.vmeio.parm);
       break;

     default:
       printf("Error invalid type\n");
     }

   return 0;
} /* end init_record() */

static long write_stringout(pstringout)
    struct stringoutRecord	*pstringout;
{
    long status;
    int ret;
    char buff[256];
    char * MyName = "write_stringout";

    printf("%s: %s\n",MyName,pstringout->val);

    ret = send_mess( 0, pstringout->val, NULL);
    printf("%s: return = 0x%x\n",MyName,ret);

    recv_mess(0,buff,1);
    printf("recv_mess: %s\n",buff);
    return(0);
}
