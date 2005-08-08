/* File: drvMicos.h             */


/* Device Driver Support definitions for motor */
/*
 *      Original Author: Kurt Goetze
 *      Current Author: Kurt Goetze
 *      Date: 11/24/2003
 *
 * Modification Log:
 * -----------------
 * .00  11/24/2003  kag  initialized from drvMCB4B.h
 */

#ifndef	INCdrvMicosh
#define	INCdrvMicosh 1

#include "motordrvCom.h"

/* Micos default profile. */

#define MICOS_NUM_CARDS   16
#define MICOS_NUM_AXIS    16
#define CTLA               1
#define OUTPUT_TERMINATOR "\r"
#define INPUT_TERMINATOR   3

struct MicosController
{
    void *serialInfo;       /* For RS-232 */
    int serial_card;        /* Card on which Hideos/MPF is running */
    char serial_task[20];   /* Hideos/MPF task/server name for serial port */
};

#endif	/* INCdrvMicosh */
