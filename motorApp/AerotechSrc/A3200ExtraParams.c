#include <A3200ExtraParams.h>
#include <epicsExport.h>
#include <iocsh.h>

/* AsynDriver routines */
static asynStatus drvUserCreate     (void *drvPvt, asynUser *pasynUser,
                                     const char *drvInfo,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserGetType    (void *drvPvt, asynUser *pasynUser,
                                     const char **pptypeName, size_t *psize);
static asynStatus drvUserDestroy    (void *drvPvt, asynUser *pasynUser);


static asynDrvUser A3200DrvUser = {
    drvUserCreate,
    drvUserGetType,
    drvUserDestroy
};

// Interposer to existing A3200 Asyn port
int A3200Interpose(const char *portName)
{
    A3200Pvt *pPvt;
    asynInterface *drvUserPrev;
    asynStatus status;

    pPvt = callocMustSucceed(1, sizeof(*pPvt), "A3200Interpose");
    pPvt->portName = epicsStrDup(portName);

    pPvt->drvUser.interfaceType = asynDrvUserType;
    pPvt->drvUser.pinterface  = (void *)&A3200DrvUser;
    pPvt->drvUser.drvPvt = pPvt;

    status = pasynManager->interposeInterface(portName, -1, &pPvt->drvUser, &drvUserPrev);
    if (status != asynSuccess) {
        errlogPrintf("A3200 ERROR: calling interpose interface.\n");
        return -1;
    }
    pPvt->drvUserPrev = drvUserPrev->pinterface;
    pPvt->drvUserPrevPvt = drvUserPrev->drvPvt;

    return(asynSuccess);
}

static asynStatus drvUserCreate(void *drvPvt, asynUser *pasynUser,
                                const char *drvInfo,
                                const char **pptypeName, size_t *psize)
{
	A3200Pvt *pPvt = (A3200Pvt *) drvPvt;
    int i;
    char *pstring;
    int ncommands = sizeof(A3200Commands)/sizeof(A3200Commands[0]);

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "A3200::drvUserCreate, drvInfo=%s, pptypeName=%p, psize=%p, pasynUser=%p\n",
               drvInfo, pptypeName, psize, pasynUser);

    for (i=0; i < ncommands; i++) {
        pstring = A3200Commands[i].commandString;
        if (epicsStrCaseCmp(drvInfo, pstring) == 0) {
            break;
        }
    }
    if (i < ncommands) {
        pasynUser->reason = A3200Commands[i].command;
        if (pptypeName) {
            *pptypeName = epicsStrDup(pstring);
        }
        if (psize) {
            *psize = sizeof(A3200Commands[i].command);
        }
        asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "A3200::drvUserCreate, command=%s\n", pstring);
        return(asynSuccess);
    } else {
        /* This command is not recognized, call the previous driver's routine */
        return(pPvt->drvUserPrev->create(pPvt->drvUserPrevPvt, pasynUser, drvInfo, pptypeName, psize));
    }
}

static asynStatus drvUserGetType(void *drvPvt, asynUser *pasynUser,
                                 const char **pptypeName, size_t *psize)
{
	A3200Pvt *pPvt = (A3200Pvt *) drvPvt;
    A3200Command command = pasynUser->reason;

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
              "A3200::drvUserGetType entered");

    if (command == axisFaultStatus)
    {
    	*pptypeName = NULL;
    	*psize = 0;
        if (pptypeName)
            *pptypeName = epicsStrDup(A3200Commands[command-MOTOR_AXIS_NUM_PARAMS].commandString);
        if (psize) *psize = sizeof(command);
        return(asynSuccess);
    }
    else
    {
        return(pPvt->drvUserPrev->getType(pPvt->drvUserPrevPvt, pasynUser,
                                                 pptypeName, psize));
    }
}

static asynStatus drvUserDestroy(void *drvPvt, asynUser *pasynUser)
{
    A3200Pvt *pPvt = (A3200Pvt *) drvPvt;
    return(pPvt->drvUserPrev->destroy(pPvt->drvUserPrevPvt, pasynUser));
}

static const iocshArg A3200InterposeArg0 = {"Port Name", iocshArgString};
static const iocshArg * const A3200InterposeArgs[1] = {&A3200InterposeArg0};
static const iocshFuncDef A3200InterposeDef = {"A3200Interpose", 1, A3200InterposeArgs};

static void A3200InterposeCallFunc(const iocshArgBuf *args)
{
    A3200Interpose(args[0].sval);
}


static void A3200InterposeRegister(void)
{
    iocshRegister(&A3200InterposeDef,  A3200InterposeCallFunc);
}

epicsExportRegistrar(A3200InterposeRegister);



