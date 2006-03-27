/******************************************************************************
**
** Program:
**    devSimPmac.c
**
** Description:
**    Functions to process commands from the EPICS PMAC device driver software
**    to drive motor simulation code.
**
** Author:
**    Martin Norbury (1/3/05)
**
** $Id: devSimPmac.c,v 1.2 2005/04/13 15:58:13 mn Exp $
**
** $Name:  $
** 
** $Log: devSimPmac.c,v $
** Revision 1.2  2005/04/13 15:58:13  mn
** Modifications to incorporate a simulation layer
**
** Revision 1.1  2005/03/07 09:21:52  mn
** First import to CVS
**
**
******************************************************************************/
#define DEVSIMPMAC_C

#include "devSimPmac.h"

/*----------------debugging-----------------*/
#ifdef	DEBUG
volatile int devSimPmacdebug = 9;
#define Debug(l, f, args...) { if(l<=devSimPmacdebug) printf(f,## args); }
#else
#define Debug(l, f, args...)
#endif

/* Global variables */
simmotor simmot[MAX_CARDS][MAX_AXIS];
int num_simmotors=0;


/******************************************************************************
**
** Function:
**   MotorConfig()
**
** Description:
**   Function to configure a simulated motor. Called from st.cmd file
**
** Arguments:
**   int card     (in)
**   int axis     (in)
**
** Return:
**    void
** 
******************************************************************************/
int MotorConfig( int card, int axis)
{
   
  Debug(2,"Adding simulated motor card %d axis %d\n",card,axis);
  
  if( card >= MAX_CARDS )
  {
    errlogPrintf("%s(%d): card > MAX_CARDS\n",__FILE__,__LINE__);
    return ERROR;
  }

  if( axis >= MAX_AXIS )
  {
    errlogPrintf("%s(%d): axis > MAX_AXIS\n",__FILE__,__LINE__);
    return ERROR;
  }
  
  /* Set card to active */
  simmot[card][axis].act=1;


  /* Increment number of simulated motors */
  num_simmotors++;

  Debug(2,"Finish MotorConfig\n");
  return OK;

}

/******************************************************************************
**
** Function:
**   simmotor_task()
**
** Description:
**   Function to process commands from the EPICS pmac device driver
**
** Arguments:
**   int a1 ... a10   (in)
**
** Return:
**    void
** 
******************************************************************************/
void simmotor_task( int a1, /* Semaphore ID */
                    int a2, /* DPRAM Pointer */
                    int a3, 
                    int a4, 
                    int a5, 
                    int a6,
                    int a7, 
                    int a8,
                    int a9,
                    int a10
                   )
{
  STATUS sem_ret;
  SEM_ID simulate_sem;
  struct pmac_dpram **pDpram;
  struct pmac_dpram *ptest;
  char *ptoken;
  int numtoken;

  Debug(2,"INSIDE simmotor_task\n");
  Debug(2,"simulate_sem = %d\n",(int) simulate_sem);

  
  /* Check to see if we have any simulated motors */
  if( num_simmotors == 0 )
  {
    errlogPrintf("%s(%d): No simulated motors configured\n",__FILE__,__LINE__);
    return ERROR;
  }
  
  /* Initialise semaphore */
  simulate_sem = (SEM_ID) a1;

  /* Initialise the pointer to DPRAM */
  Debug(2,"sizeof(int)>=sizeof(char *)\n");
  pDpram = (struct pmac_dpram **) a2;
  
  ptest=(struct pmac_dpram *) pDpram[0];

  Debug(2,"pDpram[0] = 0x%x\n",(int) pDpram[0]);

  for(;;)
  {
    /* Wait for semaphore */
    sem_ret = semTake(simulate_sem, -1 );
    if( sem_ret == ERROR )
      errlogPrintf("%s(%d): Error in semTake\n",__FILE__,__LINE__);
    Debug(2,"Semaphore taken\n");

    /* Parse the DPRAM command buffer */
    Debug(2,"test %s\n",&ptest->cmndbuff[0]);
    pmacmessage(ptest);

    /* Take action according to command */
    
    

  }/* End of for(;;) */

  printf("FINISH simmotor_task\n");

}
/******************************************************************************
**
** Function:
**   pmacmessage
**
** Description:
**   Function to process split incoming messages into pmac commands
**
** Arguments:
**   struct pmac_dpram *pdram     (in)   Pointer to DPRAM ASCII buffer
**
** Return:
**    void
** 
******************************************************************************/
void pmacmessage( struct pmac_dpram *pdpram )
{
  char space[]=" ";  /* Space delimeter */
  int  equals=61;    /* Equals */
  int  stop='/';
  char *pchar;
  char *ptest;
  
  Debug(2,"Parsing pmac message\n");

  Debug(2,"MESSAGE = %s\n",&pdpram->cmndbuff[0]);

  pchar = (char *) &pdpram->cmndbuff[0];

  switch( pchar[0] )
  {
  case 'M': /* Get M-Variable */
    Debug(2,"Getting M-Variable\n");
    getMvariable(pdpram);
    break;
    
  case 'I': /* Get I-Variable */
    Debug(2,"Getting I-Variable\n");

    ptest = strchr( pchar,equals);
    Debug(2,"ptest = 0x%x\n",(int) ptest);

    if( strchr( pchar,equals) != NULL )
    {
      Debug(2,"Set I-variable\n");
      setIvariable(pdpram);
    }
    else
    {
      Debug(2,"Get I-variable %s\n",pchar);
      getIvariable(pdpram);
    }
    break;

  case '#': /* Get axis info, start axis moving or stop axis */
    Debug(2,"Axis command\n");

    if( strchr( pchar, equals) !=  NULL  )
    {
      Debug(2,"Set axis\n");
      setAxis(pdpram);     
    }
    else if (strchr( pchar, stop) != NULL )
    {
      Debug(2,"STOP AXIS\n");
      stopAxis(pdpram);
    }
    else
    {
      Debug(2,"Get axis status\n");
      getStatus(pdpram);
    }

    break;
    
  case 'T': /* Get PMAC type */
    Debug(2,"Get PMAC type\n");
    getType(pdpram);
    break;
    
  case 'V': /* Get PMAC version */
    Debug(2,"Get PMAC version\n");
    getVersion(pdpram);
    break;
    
  default:
    Debug(2,"Command \"%s\" not recognised\n",pchar);
    break;
    
  }
	
}

void stopAxis( struct pmac_dpram *pdpram )
{
  int axis;
  char *pchar;

  pchar = (char *) &pdpram->cmndbuff[0];

  sscanf(pchar,"#%dJ/",&axis);

  Debug(1,"Axis %d stopping\n",axis);

  simmot[0][axis].dempos=simmot[0][axis].actpos;
  simmot[0][axis].state=STOPPED;

  /* Send an ACK */
  pdpram->reply_status.Bits.cntrl_char=ACK;
  pdpram->out_cntrl_wd = 0;

}

void setAxis( struct pmac_dpram *pdpram )
{
  int axis;
  int na1,na2;
  int pos;
  float vel,acc;
  char *pchar;
  
  pchar = (char *) &pdpram->cmndbuff[0];

  sscanf(pchar,"#%d I%d=%f I%d=%f J=%d",&axis,&na1,&vel,&na2,&acc,&pos);

  Debug(1,"%s: %d %d %f %d %f %d\n",__FILE__,axis,na1,vel,na2,acc,pos);

  pos *= 96; /* Multiply by positional scale factor Ix08) */
  pos *= 32; /* Multiply by fractional data component */

  simmot[0][axis].dempos=pos;
  simmot[0][axis].vel=vel;
  simmot[0][axis].acc=acc;
  simmot[0][axis].time=(float) tickGet()/ (float) sysClkRateGet();

  /* Set to moving state if this is a new position */
  if( simmot[0][axis].dempos != simmot[0][axis].actpos )
    simmot[0][axis].state=MOVING;

  /* Send an ACK */
  pdpram->reply_status.Bits.cntrl_char=ACK;
  pdpram->out_cntrl_wd = 0;
}

void getMvariable( struct pmac_dpram *pdpram )
{
  int axis;
  int mvar;
  static int i=0;
  char *pchar;
  char *pnewpos;

  Debug(2,"Inside getMvariable\n");

  pchar = (char *) &pdpram->cmndbuff[0];

  sscanf(pchar,"M%2d%2d",&axis,&mvar);

  switch( mvar )
  {
  case MVAR_COMMANDPOS:
    /* Send a response */
    
    pnewpos = calcnewpos(axis);
    Debug(2,"pnewpos = %s\n",pnewpos);
    strcpy((char *) pdpram->response,pnewpos);
    pdpram->reply_status.Bits.cntrl_char=CR;
    pdpram->out_cntrl_wd = 0;
    break;

  case MVAR_ACTUALPOS:
     /* Send a response */

    strcpy((char *) pdpram->response,"100000");
    Debug(2,"Sending %s\n",pdpram->response);
    pdpram->reply_status.Bits.cntrl_char=CR;
    pdpram->out_cntrl_wd = 0;
    break;

  default:
    Debug(2,"Unknown M-Variable\n");
    break;

  }

}

void getStatus( struct pmac_dpram *pdpram )
{
  int axis;
  char *pchar;
  char reply[256];
  int pmacstate;
  int state;

  Debug(2,"Inside getStatus\n");

  pchar = (char *) &pdpram->cmndbuff[0]; 

  sscanf(pchar,"#%d?",&axis);

  state = simmot[0][axis].state;

  if( state == MOVING )
  {
    pmacstate = 0;
  }
  else
  {
    pmacstate = 0x1;
  }
 
  sprintf(reply,"%4hx%4hx%4hx",0,0,pmacstate);

  /* Send a response */
  strcpy((char *) pdpram->response,reply);

  pdpram->reply_status.Bits.cntrl_char=CR;
  pdpram->out_cntrl_wd = 0;
  

  Debug(2,"Getting status for axis %d\n",axis);

}

void setIvariable( struct pmac_dpram *pdpram )
{
  char *pchar;

  pchar = (char *) &pdpram->cmndbuff[0];

  Debug(2,"Inside setIvariable\n");

  /* Send an ACK */
  pdpram->reply_status.Bits.cntrl_char=ACK;
  pdpram->out_cntrl_wd = 0;
  

}

void getIvariable( struct pmac_dpram *pdpram )
{
  int axis;
  int ivar;
  char *pchar;

  pchar = (char *) &pdpram->cmndbuff[0];

  Debug(2,"Inside getIvariable\n");

  sscanf(pchar,"I%2d%2d",&axis,&ivar);
  switch( ivar )
  {
  case IVAR_ACTIVATION:
    
    /* Check if axis is active */
    if( simmot[0][axis].act == 1)
      strcpy((char *) pdpram->response,"1");
    else
      strcpy((char *) pdpram->response,"0");
    pdpram->reply_status.Bits.cntrl_char=CR;
    pdpram->out_cntrl_wd = 0;
    break;

  case IVAR_JOGACCTIME:
    strcpy((char *) pdpram->response,"1");
    pdpram->reply_status.Bits.cntrl_char=CR;
    pdpram->out_cntrl_wd = 0;
    break;
  case IVAR_JOGSCRVTIME:
    strcpy((char *) pdpram->response,"1");
    pdpram->reply_status.Bits.cntrl_char=CR;
    pdpram->out_cntrl_wd = 0;
    break;

  default:
    Debug(1,"Unknown I-variable command\n");
    break;

  }

  Debug(1,"Axis %d Variable %d\n",axis,ivar);



}

void getVersion( struct pmac_dpram *pdpram )
{
  char *pchar;

  pchar = (char *) &pdpram->cmndbuff[0];

  strcpy((char *) pdpram->response,VERSIONREPLY);
  pdpram->reply_status.Bits.cntrl_char=CR;
  pdpram->out_cntrl_wd = 0;
  
}

void getType( struct pmac_dpram *pdpram )
{
  char *pchar;

  pchar = (char *) &pdpram->cmndbuff[0];

  strcpy((char *) pdpram->response,TYPEREPLY);
  pdpram->reply_status.Bits.cntrl_char=CR;
  pdpram->out_cntrl_wd = 0;
    
}

char * calcnewpos( int axis )
{
  char reply[256];
  float newtime;
  float timeincr;
  int posincr;
  float vel;
  int currentpos;
  int dempos;
  int actpos;
  int newpos;
  int state;

  state = simmot[0][axis].state;

  if( state == MOVING )
  {
    
    /* Get the time increment since last update */
    newtime = (float) tickGet() / (float) sysClkRateGet ();
    timeincr = newtime - simmot[0][axis].time;
        
    /* Velocity ( counts/msec) */
    vel = simmot[0][axis].vel;
    
    /* Check for zero velocity */
    /* assert( vel != 0 );*/

    posincr = (int) ( vel * timeincr * (float) MSEC );
    posincr *= 96; /* Multiply by positional scale factor Ix08) */
    posincr *= 32; /* Multiply by fractional data component */

    /* Make sure posincr is not zero */
    /*assert( posincr != 0 );*/

    /* Make sure posincr is at least 1 */
    if( posincr < 1 )
      posincr = 1;

    dempos = simmot[0][axis].dempos;
    actpos = simmot[0][axis].actpos;

    /* Calculate new position */

    Debug(3,"posincr = %d, vel = %f, timeincr = %f\n",posincr, vel, timeincr);
    Debug(3,"Calculate new pos: dempos %d actpos %d\n",dempos,actpos);

    /* Check to see if dempos-actpos > posincr */
    if( abs( actpos - dempos ) > posincr )
    {
      Debug(3,"fabs(actpos - dempos) > posincr\n");
      if( dempos > actpos )
      {
	simmot[0][axis].actpos += posincr;
      }
      else
      {
	simmot[0][axis].actpos -= posincr;
      }
    }
    else
    {
      Debug(3,"fabs(actpos - dempos) < posincr\n");
      simmot[0][axis].actpos = dempos;
      simmot[0][axis].state = STOPPED;
    }
    
    Debug(2,"%s(%d): posincr = %d %f %f\n"__FILE__,__LINE__,posincr,vel,timeincr);
  

    sprintf(reply,"%d",simmot[0][axis].actpos);

    /* Tidy up save timestamp */
    simmot[0][axis].time=newtime;


  }
  else
  {
    sprintf(reply,"%d",simmot[0][axis].actpos);
  }
  

  return reply;


}

#undef DEVSIMPMAC_C
