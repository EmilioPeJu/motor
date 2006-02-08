/* This script was addapted from the Newport Socket.cpp code
   to provide TCP/IP sockets for the XPSC8 motion controller
   using EPICS asynOctetSyncIO.cc
   
   By Jon Kelly July 2005
   
   Modification Log:
   -----------------
   10/11/05 Improve the error checking on the Send&Recieve by counting
   the size of the returned string and matching the requested number of
   numbers with the returned number of commas.
   
*/

/* includes */

#ifdef vxWorks
#else
#define TRUE 1
#define FALSE 0
typedef int BOOL;
#endif

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include "asynDriver.h"
#include "asynOctetSyncIO.h"
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsExport.h>
#include <epicsString.h>


/* defines */

#define MAX_MSG_SIZE       256
#define TIMEOUT            0.2
#define MAX_RETRY          10
#define MAX_INSPEC	   8
#define MAX_RESEND	   5
#define CONNREFUSED        -1
#define CREATESOCKETFAILED -2
#define OPTREFUSED         -3

#define MAX_NB_SOCKETS     50
#define MAX_BUSY_LOOP      100000

#define BUFFERCLEAR	   0	/* Used in SendOnly */

/* global variables */

BOOL    UsedSocket[MAX_NB_SOCKETS] = { FALSE };
double  TimeoutSocket[MAX_NB_SOCKETS];
int     ErrorSocket[MAX_NB_SOCKETS];
BOOL    asynXPSC8InterfaceBusy =  { FALSE };

/* Pointer to the connection info for each socket 
   the asynUser structure is defined in asynDriver.h */
static struct asynUser *pasynUserArray[MAX_NB_SOCKETS];

asynStatus status;
#define DEBUG

#ifdef __GNUG__
    #ifdef	DEBUG
        volatile int asynXPSC8Debug = 0;
	#define Debug(l, f, args...) { if(l<=asynXPSC8Debug) {\
			printf("asynOctetSocket - "); \
			printf(f,## args); }}
	epicsExportAddress(int, asynXPSC8Debug);
    #else
	#define Debug(l, f, args...)
    #endif
#else
    #define Debug()
#endif


/* The returned error values correspond to the xps values */

/***************************************************************************************/
/* The drvXPSC8.cc must assign the aysn port string to the Ip field 
   and replace the xps port int with the asyn addr int		*/
int ConnectToServer(char *Ip_Address, int Ip_Port, double TimeOut)
{
/*	printf("Socket.cpp ConnectToServer: Top/n");*/

    int SocketIndex = 0;
    const char *asynPort = (const char *)Ip_Address;
    int asynAddr;
    asynAddr = Ip_Port;


    /* Select a free socket */
    while ((UsedSocket[SocketIndex] == TRUE) && (SocketIndex < MAX_NB_SOCKETS))
	{
		SocketIndex++;
	}

    if (SocketIndex == MAX_NB_SOCKETS)
        return -1;

    ErrorSocket[SocketIndex] = 0;


    /* These asyn functions are defined in asynOctetSyncIO.c */
    status = pasynOctetSyncIO->connect(asynPort,asynAddr,&pasynUserArray[SocketIndex],NULL);
    
    if (status != asynSuccess )
    {
        ErrorSocket[SocketIndex] = CREATESOCKETFAILED;
	printf("Error Socket connect failed asynStatus=%i  \n",status);
        return -1;
    }

    
    /* Use default timeout macro if one is not given 
    if (TimeOut > 0) TimeoutSocket[SocketIndex] = TimeOut;
    else             TimeoutSocket[SocketIndex] = TIMEOUT;*/
    
    UsedSocket[SocketIndex] = TRUE;
    TimeoutSocket[SocketIndex] = TIMEOUT;
    return SocketIndex;
}

/***************************************************************************************/
void SetTCPTimeout(int SocketIndex, double TimeOut)
{
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
       if (TimeOut > 0) TimeoutSocket[SocketIndex] = TimeOut;
    }
}

/***************************************************************************************/
void SendAndReceive (int SocketIndex, char *buffer, char *valueRtrn)
{
    size_t nbytesOut = 0; 
    size_t nbytesIn = 0;
    int eomReason;
    char *output, *input;
    int writeReadStatus = -99;
    int loop = 0;
    int inspecLoop = 0;
    size_t measureLength; 
    size_t bufferLength;
    int totalStar;
    int totalComma;
    
    /* Valid Socket? */
    bufferLength = strlen(buffer);
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        if (bufferLength > MAX_MSG_SIZE)
        {
	    /* Error string too long */
	    printf("Error buffer>Max SendAndRecieve Out=%s In=%s\n",valueRtrn,buffer);
	    printf("asynStatus=%i  ",status);
	    strcpy(valueRtrn,"-3");
	    return;
	}
    	
	while ((writeReadStatus < 0) && (loop < MAX_RETRY))
	{
	    status = pasynOctetSyncIO->writeRead(pasynUserArray[SocketIndex],
						(char const *)buffer, 
						bufferLength,
						valueRtrn,
						MAX_MSG_SIZE,
						TimeoutSocket[SocketIndex],
						&nbytesOut,
						&nbytesIn,
						&eomReason);
	
	    Debug(12,"buffer %s\n bufferlen %i\n valueRtrn %s\n",buffer,bufferLength,valueRtrn);
	    Debug(12,"nbytesOut %i nbytesIn %i eomReason %i\n",nbytesOut,nbytesIn,eomReason);
	    Debug(12,"loop=%i \n",loop); 
	    output = valueRtrn;
	    if (output != NULL) sscanf (output, "%i", &writeReadStatus);
	    
	    if (writeReadStatus < 0){
	        Debug(10,"Address %x\n",pasynUserArray[SocketIndex]);
	        Debug(1,"Error WriteReadStatus = %i Retry ",writeReadStatus);
		Debug(2,"Sock=%i Tout=%g In=%s Out=%s \n  Command sent again! loop %i\n",SocketIndex,
					TimeoutSocket[SocketIndex],valueRtrn,buffer,loop);
	    } else {
	        output = valueRtrn;
		input = buffer;		
	
		/* If the relpy is longer than it should be retry & don't bother to check
		   the number of ,s = *s. */
		measureLength = strlen (output);

		Debug(12,"measureLength=%i nbytesIn=%i\n",measureLength,nbytesIn);
		
		if (measureLength != nbytesIn) {
		    Debug(2,"Reply too long in=%s out=%s\n",valueRtrn,buffer);
		    Debug(2,"measureLength=%i nbytesIn=%i\n",measureLength,nbytesIn);
		    writeReadStatus = -98;
		    goto after_error_check; /* jump further error checking */
		}
		
		/* See if the number of ,s -1 Out = *s In i.e. the correct number of values */
		inspecLoop = 0;
	        while ( (input=strchr (input, '*')) != NULL && inspecLoop < MAX_INSPEC) {
		  inspecLoop++; /* Increment loop counter */
		  input++; 	/* Move ptr past * */ 
		}
		totalStar = inspecLoop;
		
		inspecLoop = 0;
	        while ( (output=strchr (output, ',')) != NULL && inspecLoop < MAX_INSPEC) {
		  inspecLoop++; /* Increment loop counter */
		  output++; 	/* move ptr past , */ 
		}
		totalComma = inspecLoop;
		
		Debug(4,"totalStar=%i totalComma=%i\n",totalStar,totalComma);
		
		if (totalStar != (totalComma -1 )) {
		  Debug(2,"Error Not enough numbers in=%s out=%s\n",valueRtrn,buffer);
		  Debug(2,"totalStar=%i totalComma=%i\n",totalStar,totalComma);
		  writeReadStatus = -98;
		 } 
		 
after_error_check:		
	        if (loop > 0) Debug(4,"loop=%i\n",loop);
	    }
	    loop++;
	
	} /* End of while loop */
	
	if (loop >= MAX_RETRY) Debug(1,"writeread debug failed loop=%i max=%i\n",loop,MAX_RETRY); 
	
	if ( status != asynSuccess ) 
    	{
	    /* Error no data returned */
	    /*strcpy(valueRtrn,"-71");*/
	    Debug(5,"Error buffer %s\n bufferlen %i\n valueRtrn %s\n",buffer,bufferLength,valueRtrn);
	    Debug(5,"Error nbytesOut %i nbytesIn %i eomReason %i\n",nbytesOut,nbytesIn,eomReason); 
	    
	    printf("Error SendAndRecieve read In=%s Out=%s nbytesOut=%i",valueRtrn,buffer,nbytesOut);
	    printf(" asynStatus=%i \n",status);
	    asynXPSC8InterfaceBusy = FALSE;
	    return;
	}
	
    } else {
        /* Not allowed action */
	printf("Error Socket Invalid SendAndRecieve In=%s Out=%s\n",valueRtrn,buffer);
	printf("asynStatus=%i  ",status);
	strcpy(valueRtrn,"-22");
        asynXPSC8InterfaceBusy = FALSE;
	return;
    }
    asynXPSC8InterfaceBusy = FALSE;
    return;
}
/***************************************************************************************/
void SendOnly (int SocketIndex, char *buffer, char *valueRtrn)
{
    char dummyReturn[MAX_MSG_SIZE];
    size_t nbytesOut = 0; 
    size_t nbytesIn = 0;
    int eomReason;
    char const dummyBuffer[40] = "FirmwareVersionGet (char *)";
    char *output;
    int writeReadStatus = -99;
    int loop = 0;
    size_t bufferLength = 0;
    
    /* Valid Socket? */
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        if (strlen(buffer) > MAX_MSG_SIZE)
        {
	    /* Error string too long */
	    strcpy(valueRtrn,"-3");
	    return;
	}

Above_Write:
	bufferLength = (size_t) strlen(buffer);;
	status = pasynOctetSyncIO->write(pasynUserArray[SocketIndex],(char const *)buffer, 
				 bufferLength,TimeoutSocket[SocketIndex], &nbytesIn);

	if ((status != asynSuccess) || (nbytesIn <= 0))
    	{
	    printf("****Error SendOnly status=%i",status);
	    /* Error during write message */
	    strcpy(valueRtrn,"-72");
	    asynXPSC8InterfaceBusy = FALSE;
	    return;
	}

	if (BUFFERCLEAR == 0) {
	    epicsThreadSleep(0.1); /* Wait for the XPS to digest the command */
	    goto SendOnly_End; /* Don't  try to clear the buffer */
	    }

	/* Send a command to the XPS until it replys with the correct answer */	
Above_Dummy_writeRead:	 
	 status = pasynOctetSyncIO->writeRead(pasynUserArray[SocketIndex],
						(char const *)dummyBuffer, 
						strlen(dummyBuffer),
						dummyReturn,
						MAX_MSG_SIZE,
						TimeoutSocket[SocketIndex],
						&nbytesOut,
						&nbytesIn,
						&eomReason);

	 output = dummyReturn;
	 if (output != NULL) sscanf (output, "%i", &writeReadStatus);
	 
	 else Debug(5,"SendOnly Dummywriteread Output Null\n");
	    
	 if (writeReadStatus < 0 && output != NULL){
	        Debug(2,"SendOnly DumyWriteReadStatus = %i Retry ",writeReadStatus);
		Debug(3,"Sock=%i Timeout=%g DumyReturn=%s \n DummyIn=%s OrigCommand=%s\n"\
		" Dummy sent again\n",SocketIndex,TimeoutSocket[SocketIndex],
							dummyReturn,dummyBuffer,buffer);
	    } 
	 loop++;
	 if (writeReadStatus == -3 && loop < MAX_RETRY) /* Command Not executed by XPS so retry */
	     goto Above_Write;
	 if (writeReadStatus < 0 && loop < MAX_RESEND) /* The rubish has not been cleared!*/
	     goto Above_Dummy_writeRead;
	 if (loop >= MAX_RESEND) Debug(1,"Send debug failed loop=%i max=%i\n",loop,MAX_RESEND);  
    } else {
        /* Not allowed action */
	strcpy(valueRtrn,"-22");
	asynXPSC8InterfaceBusy = FALSE;
        return;
    }
SendOnly_End:
    strcpy(valueRtrn,"0");
    asynXPSC8InterfaceBusy = FALSE;
    return;
}

/***************************************************************************************/
void CloseSocket(int SocketIndex)
{
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        status = pasynOctetSyncIO->disconnect(pasynUserArray[SocketIndex]);
	
	TimeoutSocket[SocketIndex] = TIMEOUT;
	ErrorSocket[SocketIndex] = 0;
	UsedSocket[SocketIndex] = FALSE;
    }
}

/***************************************************************************************/
void CloseAllSockets(void)
{
	int i;
    for (i = 0; i < MAX_NB_SOCKETS; i++)
    {
	if (UsedSocket[i] == TRUE)
	{
            status = pasynOctetSyncIO->disconnect(pasynUserArray[i]);
		    
	    TimeoutSocket[i] = TIMEOUT;
	    ErrorSocket[i] = 0;
	    UsedSocket[i] = FALSE;
	}
    }
}

/***************************************************************************************/
void ResetAllSockets(void)
{
	int i;
    for (i = 0; i < MAX_NB_SOCKETS; i++)
    {
		if (UsedSocket[i] == TRUE)
		    UsedSocket[i] = FALSE;
    }
}


/***************************************************************************************/
char * GetError(int SocketIndex)
{
    if ((SocketIndex >= 0) && (SocketIndex < MAX_NB_SOCKETS) && (UsedSocket[SocketIndex] == TRUE))
    {
        switch (ErrorSocket[SocketIndex])
        {
        case CONNREFUSED:         return("The attempt to connect was rejected.");
        case CREATESOCKETFAILED:  return("Create Socket failed.");
        case OPTREFUSED:          return("SetSockOption() Refused.");
        }
    }
    return("");
}
