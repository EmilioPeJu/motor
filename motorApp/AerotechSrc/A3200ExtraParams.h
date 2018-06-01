/*
 * This file contains extra parameters to be stored in A3200 axis params
 */

#include <asynDriver.h>
#include <asynDrvUser.h>
# include "motor_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

// Number of extra parameters
#define A3200_NUM_PARAMS 1

// Extra A3200 parameters
typedef enum {
    axisFaultStatus = MOTOR_AXIS_NUM_PARAMS
} A3200Command;

// A3200 command struct
typedef struct {
	A3200Command command;
    char *commandString;
} A3200CommandStruct;

// A3200 parameter names
static A3200CommandStruct A3200Commands[A3200_NUM_PARAMS] = {
    {axisFaultStatus, "AXIS_FAULT_STATUS"}
};

// Asyn driver user
typedef struct {
    char *portName;
    asynInterface drvUser;
    asynDrvUser *drvUserPrev;
    void *drvUserPrevPvt;
} A3200Pvt;

// Asyn driver setup
int A3200Interpose(const char *portName);

#ifdef __cplusplus
}
#endif
