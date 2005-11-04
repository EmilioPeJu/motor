#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <iocsh.h>
#include "epicsExport.h"

int PmacSetup(int num_cards,    /* maximum number of cards in rack */
             int addrs_type,    /* VME address type; 24 - A24 or 32 - A32. */
             void *mbox,        /* Mailbox base address. */
             void *addrs,       /* DPRAM Base Address */
             unsigned vector,   /* noninterrupting(0), valid vectors(64-255) */
             int int_level,     /* interrupt level (1-6) */
             int scan_rate);     /* polling rate - in HZ */



extern "C"
{

/* Pmac Setup */
static const iocshArg setupArg0 = {"num_cards", iocshArgInt};
static const iocshArg setupArg1 = {"addrs_type",iocshArgInt};
static const iocshArg setupArg2 = {"*mbox",iocshArgInt};
static const iocshArg setupArg3 = {"*addrs",iocshArgInt};
static const iocshArg setupArg4 = {"interrupt vector",iocshArgInt};
static const iocshArg setupArg5 = {"interrupt level",iocshArgInt};
static const iocshArg setupArg6 = {"scan rate",iocshArgInt};

static const iocshArg * const PmacSetuparg[7] = {&setupArg0,&setupArg1,&setupArg2,&setupArg3,&setupArg4,&setupArg5,&setupArg6};
static const iocshFuncDef setupFuncDef = {"PmacSetup", 7, PmacSetuparg};
static void setupCallFunc(const iocshArgBuf *args)
{
    PmacSetup(args[0].ival,args[1].ival,args[2].vval,args[3].vval,args[4].ival,args[5].ival,args[6].ival);
}


static void PmacSetupRegister(void)
{
    iocshRegister(&setupFuncDef, setupCallFunc);
}

epicsExportRegistrar(PmacSetupRegister);

} // extern "C"

