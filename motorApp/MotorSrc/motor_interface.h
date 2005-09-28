/**\page Overview Analysis of current motor record interface to device and driver support.

At present, the motor record does not have a clean interface between
generic record support code and specific implementations for a given
motor controller because of the 

\section control Analysis of commands that control hardware behaviour

All of these commands go through an interface with three routines
implementing a transaction style interface: start_trans, build_trans
and end_trans. The implication is that any build_trans commands issued
between a start_trans and an end_trans happen atomically. Build_trans
takes a three parameters, the transaction type (an enumerated value) a
pointer to a double parameter, and a pointer to the motor record
itself.

\subsection sec1 build_trans commands that are only executed alone.

All of the commands in this section are only ever called alone, bracketed by a start_trans and an end_trans call. Hence, they could be re-implemented in a non-transaction based interface.

\subsubsection ss1 Commands that could be removed
\li \b PRIMITIVE  Called line 316 motordevCom.cc. This command is used just to pass initialisation strings to the controller from the motor OUT link. It could be done as part of controller initialisation.

\subsubsection ss2 Commands for which the return status is consulted.

These return an error if there is a problem with the demand value. On return, I think the demand value should be
replaced by the permitted value (but I don't see this being used).

\li \b SET_DGAIN Called line 341 motordevCom.cc. Called line 2571 motorRecord.cc
\li \b SET_HIGH_LIMIT Not called in motordevCom.cc. Called line 2346, 2387, 3466 motorRecord.cc
\li \b SET_IGAIN Called line 334 motordevCom.cc. Called line 2571 motorRecord.cc
\li \b SET_LOW_LIMIT Not called in motordevCom.cc. Called line 2346, 2387, 3501 motorRecord.cc
\li \b SET_PGAIN Called line 326 motordevCom.cc. Called line 2571 motorRecord.cc

\subsubsection ss3 Other build_trans commands called singly

The return status is never checked when these transactions are build.

\li \b DISABL_TORQUE Not called in motordevCom.cc. Called line 2593 motorRecord.cc
\li \b ENABLE_TORQUE Not called in motordevCom.cc. Called line 2591 motorRecord.cc. These two \b _TORQUE commands are crying out to be implemented as a single command because the parameter is currently ignored. Note that they don't really enable or disable torque, they actually open or close the PID loop.

\li \b GET_INFO Called line 348 motordevCom.cc. Called line 1117, 1149, 1645, 1908, 2111, 3270 motorRecord.cc. Note that this ultimately leads to the motor status information being updated, and then this is accessed by a call to motor_update_values in motordevCom.
\li \b LOAD_POS Called line 309 motordevCom.cc. Called line 3267 motorRecord.cc. Note that the position loaded is not a demand position - this is just a command to set the current position to a given absolute value.
\li \b SET_ENC_RATIO Called line 300 motordevCom.cc. Called line 1638 motorRecord.cc.
\li \b STOP_AXIS Not called in motordevCom.cc. Called line 1094, 1533, 1566, 1701, 1760, 1798 motorRecord.cc. It is not clear exactly what acceleration should be used in stopping - but I assume that the one that initiated the last move is probably OK.

\subsection sec2 build_trans commands that are executed in groups

These commands are always executed in as part of a group of
transactions intended to initiate motion of some kind. The consist of:
\b SET_VEL_BASE, \b SET_VELOCITY, \b SET_ACCEL, \b JOG_VELOCITY, \b
HOME_FOR, \b HOME_REV, \b MOVE_REL, \b MOVE_ABS, \b GO and \b JOG. They implement three different kinds of move:

\li \b Home - Starts a home search.
\li \b Move - Starts a relative or absolute move of a fixed distance.
\li \b Jog - Starts moving at a given velocity and keeps moving until stopped.

The latter is an unfortunate name since I think the work Jog has
multiple meanings in motion control context, but here we are dictated
by the OMS definition where a jog is a constant velocity move. There
is also two JOG commands, JOG and JOG_VELOCITY, the only difference in
the OMS driver being that JOG clears the controller DONE flag for this
axis before starting.

In the API I have implemented these three collections of commands as
three separate routines, not as multiple single parameter routines
with a start/end buffering command. I have done this for simplicity to
highlight the minimum functionality that needs to be provided to
support the motor record, but this may be the wrong
decision. Buffering could get complicated if multiple people wrote it
in different ways. Note that if we did go down the buffering route, I
see no need for both a GO command or a JOG and JOG_VELOCITY
command. The former is implicit in the end_trans, and the latter
should be amalgamated into a single command.

\subsubsection ss4 Homing sequence.

\li \b SET_VEL_BASE
\li \b SET_VELOCITY 
\li \b HOME_FOR or \b HOME_REV
\li \b GO.

These five transactions are called as a set in lines 659-662 and also
in lines 1719-1726 in motorRecord.cc. Only one of HOME_FOR or HOME_REV
is called, dependent on the setting of the record homf or homr
fields. (What happens if you don't find the home switch - can't the
controller work out which way to attempt to home?). Note that no 
acceleration is specified.

\subsubsection ss5 Move sequence.
\li ( \b SET_VEL_BASE )
\li \b SET_VELOCITY
\li \b SET_ACCEL
\li \b MOVE_REL or MOVE_ABS
\li \b GO

These six transactions are called as a set three times in
motorRecord.cc (between lines 704-746, lines 778-798 and lines
2042-2102). Only one of MOVE_REL and MOVE_ABS is called depending on
whether it is a relative move or an absolute move, and the parameter
passed in this transaction is the actual move distance. SET_VEL_BASE
is in parentheses because it is not called every time - but the reason
for this is pretty unclear to me (I suspect it could be).

\li In the first invocation (lines  704-746), SET_VEL_BASE is only included if it is a MIP_JOG_STOP.
\li It is \b not called in the second invocation (lines 778-798).
\li It is  included in the third invocation (lines 2042-2102).

\subsubsection ss6 Jog sequences

There are two jog sequences:

\li \b SET_ACCEL followed by \b JOG. Called between lines 1782-1785 motorRecord.cc.
\li \b SET_ACCEL followed by \b JOG_VELOCITY. Called between lines 2622-2625 motorRecord.cc.

I don't really understand the real difference between these two set -
as far as I can see the only difference in the OMS driver is that the
JOG transaction executes a CA to clear the axis done bit before
running, whilst JOG_VELOCITY doesn't. I suspect that the two could be
replaced by a single command in a new interface.

\section Status Analysis of commands that provide status information

As mentioned above, motor status is handled by issuing GET_INFO commands on a regular basis, and then calling pdset->update_values (which is actually motor_update_values in motordevCom.cc). This routine updates the:

\li Motor step position (motor record rmp field)
\li Encoder position (motor record rep field), if available.
\li Motor velocity (motor record vel field) and
\li Status bits (motor record msta field). The mst field is a bit field of motor status bits, some reflecting the status of the current move, and others representing motor capabilities.

There is also a certain amount of other updates done in device support, but I suspct these could possibly all be incorporated into a generic piece of code, and 

\section Analysis How could this be documented as an acceptable interface.

There are a number of problems with the motor record as it stands.

\li There is no defined (i.e. written down) interface anywhere between the motor record and driver code that implements the specific control of a particular motor controller.
\li This is further complicated by there being multiple levels of shared code - the motor record itself, motordevCom and motodrvCom. These each have multiple interfaces between themselves and shared code.
\li The arrangement of shared code makes it difficult to extend the motor controller interface in a generic way to support records other than the motor record.

As a result of this, I feel the first thing to do is to define an API
that can be implemented by person developing a motor controller, with
the knowledge that if they provide this API, then they will have motor
record support. We can then extend the API to provide support for
other records, or the suppportor of a particular motor controller can
do it on his own. We will then write device support that links the API
and the motor record, using the API as, effectively, a driver support
entry table.

*/

/** Forwadr declaration of AXIS_ID, which is a pointer to an internal driver-dependent handle */
typedef struct motorAxis_str * AXIS_ID;

#define MOTOR_AXIS_OK (0)
#define MOTOR_AXIS_ERROR (-1)


/**\union motorAxisStatusBits_union

    This contains all the status bits required by the motor record
*/

typedef union motorAxisStatusBits_union
{
    unsigned long All;
    struct
    {
#ifdef MSB_First
	unsigned int na		    :18;
	unsigned int RA_MINUS_LS    :1;
	unsigned int CNTRL_COMM_ERR :1;
	unsigned int GAIN_SUPPORT   :1;
	unsigned int RA_MOVING      :1;
	unsigned int RA_PROBLEM     :1;
	unsigned int EA_PRESENT     :1;
	unsigned int EA_HOME        :1;
	unsigned int EA_SLIP_STALL  :1;
	unsigned int EA_POSITION    :1;
	unsigned int EA_SLIP        :1;
	unsigned int RA_HOME        :1;
	unsigned int RA_PLUS_LS     :1;
	unsigned int RA_DONE        :1;
	unsigned int RA_DIRECTION   :1;
#else
	unsigned int RA_DIRECTION   :1;	/**< last motion direction 0=Negative, 1=Positive */
	unsigned int RA_DONE        :1;	/**< a motion is complete */
	unsigned int RA_PLUS_LS     :1; /**< plus limit switch has been hit */
	unsigned int RA_HOME        :1; /**< The home signal is on */
	unsigned int EA_SLIP        :1; /**< encoder slip enabled (optional - not currently used in software?) */
	unsigned int EA_POSITION    :1; /**< position maintenence enabled */
	unsigned int EA_SLIP_STALL  :1; /**< slip/stall detected (optional - not currently used in software?) */
	unsigned int EA_HOME        :1; /**< encoder home signal on */
	unsigned int EA_PRESENT     :1; /**< encoder is present */
	unsigned int RA_PROBLEM     :1; /**< driver stopped polling */
	unsigned int RA_MOVING      :1;	/**< non-zero velocity present (optional - not currently used in software?) */
	unsigned int GAIN_SUPPORT   :1;	/**< Motor supports closed-loop position control. */
	unsigned int CNTRL_COMM_ERR :1;	/**< Controller communication error. */
	unsigned int RA_MINUS_LS    :1;	/**< minus limit switch has been hit */
	unsigned int na		    :18;/**< N/A bits  */
#endif
    } Bits;                                
} motorAxisStatusBits_t;

/**\struct motorAxisStatus_str

    This structure is returned by motorAxisGetStatus and contains all the current information
    required by the motor record to indicate current motor status

*/

typedef struct motorAxisStatus_str 
{
    long position;                   /**< Current motor position in motor steps (if not servoing) or demand position (if servoing) */
    long encoder_position;           /**< Current motor position in encoder units (only available if a servo system). */
    motorAxisStatusBits_t status;    /**< bit field of errors and other binary information */
} motorAxisStatus_t;


/**\defgroup EPICS EPICS driver support interface routines
@{
*/

typedef void (*motorAxisReportFunc)( int level );
/** Print the status of the motor controller to stdout

    This optional routine is intended to provide a debugging log about the motor controller
    and will typically be called by the EPICS dbior function. The level indicates
    the level of detail - typically level 0 prints a one line summary and higher
    levels provide increasing order of detail.

    \param level   [in] Report level
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
void motorAxisReport( int level );
#endif

typedef int (*motorAxisInitFunc)( void );

/** EPICS driver support entry function initialisation routine.

    This optional routine is provided purely so the motor driver support entry table 
    (motorAxisDrvSET_t) compatible with an EPICS driver support entry table.
    Typically it won't be provided and the driver support entry table
    initialised with a null pointer. However, if provided, it may be called
    at iocInit.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisInit( void );
#endif

typedef enum {motorAxisErrInfo,motorAxisErrMinor,motorAxisErrMajor,motorAxisErrFatal} motorAxisSev_t;

typedef int (*motorAxisLogFunc)( const motorAxisSev_t severity,const char *pFormat, ...);
typedef int (*motorAxisSetLogFunc)( motorAxisLogFunc logFunc );

/** Provide an external logging routine.

    This is an optional function which allows external software to hook
    the driver log routine into an external logging system. The
    external log function is a standard printf style routine with the
    exception that it has a first parameter which is a message
    severity indicator. This can have one of four values -
    infomational, minor, major or fatal.

    \param logFunc [in] Pointer to function of motorAxisLogFunc type.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetLog( motorAxisLogFunc logFunc );
#endif

/**@}*/

/**\defgroup Access Routines to open and close a connection to a motion axis.
@{
*/

typedef AXIS_ID (*motorAxisOpenFunc)( char * device, int axis );

/** Initialise connection to a motor axis.

    This routine should open a connection to an motor controller axis previously
    and return a driver dependent handle that can be uses for subsequent calls to
    other axis routines supported by the motor controller. The driver should support
    multiple opens on a single axis and process all calls from separate threads on a
    fifo basis.

    \param card  [in] Number representing the motor controller.
    \param axis  [in] Axis number - some devices may have conventions on virtual axes.
    \param param [in] Arbitrary, driver defined, parameter string.

    \return AXIS_ID - pointer to handle for using to future calls to motorAxis routines
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
AXIS_ID motorAxisOpen( int card, int axis, char * param );
#endif

typedef int (*motorAxisCloseFunc)( AXIS_ID pAxis );

/** Close a connection to a motor axis.

    This routine should close the connection to an motor controller previously
    opened with motorOpen, and clean up all space specifically allocated to this 
    axis handle.

    \param pAxis  [in]   Pointer to axis handle returned by motorAxisOpen.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisClose( AXIS_ID pAxis );
#endif

/**@} - end of Access group*/

/**\defgroup Status Routines to get axis status information
@{
*/

typedef int (*motorAxisGetStatusFunc)( AXIS_ID pAxis, motorAxisStatus_t * status );

/** Get status information about a motor axis.

    This routine returns infomation about the current axis stattus in a motorAxisStatus_t
    structure. Positions are in raw (step or encoder) units. If there is no encoder, 
    then the encoder position is undefined. If it is a stepper motor controlled by a PID 
    algorithm, both positions have to be in the same units. Velocity should be in steps/second.

    \param pAxis  [in]   Pointer to axis handle returned by motorAxisOpen.
    \param status [out]  Structure of motorAxisStatus_t giving current motor status

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisGetStatus( AXIS_ID pAxis, motorAxisStatus_t status );
#endif

typedef void (*motorAxisCallbackFunc)( void *param );
typedef int (*motorAxisSetCallbackFunc)( AXIS_ID pAxis, motorAxisCallbackFunc callback, void * param, int period );

/** Set a callback function to be called when motor axis information changes

    This routine sets a function to be called by the driver if the motor status
    changes. The interest parameter indicates a minimum update period, ranging 
    from 0 (for all updates that the driver detects) to some large number (possibly
    in clock ticks) indicating a lack of interest. Normally 0 should not flood
    a system.

    Only one callback function is allowed per AXIS_ID, and so subsequent calls to
    this function using the original axis identifier will replace the original
    callback. Setting the callback function to a NULL pointer will delete the
    callback hook.

    \param pAxis    [in]   Pointer to axis handle returned by motorAxisOpen.
    \param callback [in]   Pointer to a callback function taking a void parameter.
    \param param    [in]   Void pointer to parameter that should be used when calling the callback

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetCallback( AXIS_ID pAxis, motorAxisCallbackFunc callback, void * param );
#endif

/**@} - end of Status group*/

/**\defgroup parameters Routines that set axis control parameters
 @{
*/

typedef int (*motorAxisSetDoubleFunc)( AXIS_ID pAxis, double );
typedef int (*motorAxisSetIntegerFunc)( AXIS_ID pAxis, int );

/** Set the current motor position

    Sets the current position of the axis to be a given number in motor (i.e.
    not encoder) units.

    \param pAxis    [in]   Pointer to axis handle returned by motorAxisOpen.
    \param position [in]   Double value indicating motors current position.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetPosition( AXIS_ID pAxis, double position );
#endif

typedef int (*motorAxisSetEncoderRatioFunc)( AXIS_ID pAxis, double enc_counts, double mot_counts );

/** Set the encoder ratio

    The encoder ratio is expressed as two double precision numbers, both of which represent
    the same physical dimension (i.e. angle, distance etc) but in two different units, motor
    units (i.e. stepper motor steps) and encoder units. For stepper motors without encoders
    and DC servo motors without a concept of motor steps, both elements should be unity.

    \param pAxis      [in]   Pointer to axis handle returned by motorAxisOpen.
    \param enc_counts [in]   Number of encoder counts equivalent to mot_counts motor counts.
    \param mot_counts [in]   Number of motor counts equivalent to enc_counts encoder counts.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetEncoderRatio( AXIS_ID pAxis, double enc_counts, double mot_counts );
#endif

/** Set the axis low limit

    Sets the minimum drive position for the axis in motor units. If the value
    is out of range an error occurs and the limit isn't changed.

    \param pAxis     [in]   Pointer to axis handle returned by motorAxisOpen.
    \param low_limit [in]   Double value indicating the low limit.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetLowLimit( AXIS_ID pAxis, double low_limit );
#endif


/** Set the axis high limit

    Sets the maximum drive position for the axis in motor units. If the value
    is out of range an error occurs and the limit isn't changed.

    \param pAxis      [in]   Pointer to axis handle returned by motorAxisOpen.
    \param high_limit [in]   Double value indicating the high limit.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/


#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetHighLimit( AXIS_ID pAxis, double high_limit );
#endif

/** Enable or disable closed loop control.

    This call implements the misleadingly names ENABLE_TORQUE and DISABL_TORQUE transactions 
    which actually open and close PID loop control or any form of stepper closed loop.
    If the loop is off the PID parameters should be set to zero and any form of stepper 
    motor closed loop turned off. However, there may still be holding torque due to the 
    stepper motor power (or gearbox stiction), it is just not supplied by the control loop.

    \param pAxis   [in]   Pointer to axis handle returned by motorAxisOpen.
    \param loop_on [in]   Integer - zero for loop off, non-zero for loop on.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetClosedLoop( AXIS_ID pAxis, int loop_on );
#endif

/** Set the proportional gain.

    Sets the proportional gain term for PID control. PID control is enabled
    by the motorAxisSetClosedLoop function. The units are controller dependent,
    but might be limited to between 0.0 and 1.0 by the motor record. (Which
    might be debatable).

    \param pAxis [in]   Pointer to axis handle returned by motorAxisOpen.
    \param pgain [in]   Double value containing loop proportional gain.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetPGain( AXIS_ID pAxis, double pgain );
#endif

/** Set the integral gain.

    Sets the integral gain term for PID control. PID control is enabled
    by the motorAxisSetClosedLoop function. The units are controller dependent,
    but might be limited to between 0.0 and 1.0 by the motor record. (Which
    might be debatable).

    \param pAxis [in]   Pointer to axis handle returned by motorAxisOpen.
    \param igain [in]   Double value containing loop integral gain.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetIGain( AXIS_ID pAxis, double igain );
#endif

/** Set the differential gain.

    Sets the differential gain term for PID control. PID control is enabled
    by the motorAxisSetClosedLoop function. The units are controller dependent,
    but might be limited to between 0.0 and 1.0 by the motor record. (Which
    might be debatable).

    \param pAxis [in]   Pointer to axis handle returned by motorAxisOpen.
    \param dgain [in]   Double value containing loop differential gain.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisSetDGain( AXIS_ID pAxis, double dgain );
#endif
/**@} - end parameters group*/

/**\defgroup Motion Routines that initiate and stop motion
 @{
*/

typedef int (*motorAxisMoveFunc)( AXIS_ID pAxis, double position, int relative, double min_velocity, double max_velocity, double acceleraton );
/** Moves the axis to a given demand position and then stops.

    This is a normal move command. Moves consist of an acceleration phase, a coast phase and a decelleration phase.
    If min_velocity is greater than zero and the controller supports this function, the system will not demand 
    a non-zero velocity between + and - min_velocity. Move-on-move (i.e. a move command being issue before another
    is completed should not generate an error - the motor should immediately start moving to the latest demanded position.
    The function should return immediately the move has been started successfully.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param position      [in]   Position to move to in motor units.
    \param relative      [in]   If zero position is an absolute position, otherwise it is relative
                                to the current position.
    \param min_velocity  [in]   Minimum startup velocity in motor units/second. If negative, it will be ignored.
    \param max_velocity  [in]   Maximum velocity during move in motor units/second.
    \param acceleration  [in]   Maximum acceleration (or decelleration) during velocity ramp in 
                                motor units/second squared. Should be positive.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisMove( AXIS_ID pAxis, double position, int relative, double min_velocity, double max_velocity, double acceleration );
#endif

typedef int (*motorAxisHomeFunc)( AXIS_ID pAxis, double min_velocity, double max_velocity, int forwards );
/** Homes the axis, starting in a particular direction.

    This initiates a homing operation. If the controller supports it the home procedure can be initially in either
    a forward or a reverse direction. min_velocity and max_velocity are the same as for the motorAxisMove function.
    I don't know why there isn't an acceleration parameter, but I suppose the controller can choose any reasonable value.
    The routine returns as soon as the home has started successfully.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param min_velocity  [in]   Minimum startup velocity in motor units/second. If negative, it will be ignored.
    \param max_velocity  [in]   Maximum velocity during move in motor units/second.
    \param forwards      [in]   If zero, intitial move is in negative direction, otherwise it is positive (possibly ignored).

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisHome( AXIS_ID pAxis, double min_velocity, double max_velocity, int forwards );
#endif

typedef int (*motorAxisVelocityMoveFunc)( AXIS_ID pAxis, double velocity, int acceleration );
/** Starts the axis moving at a constant velocity

    This initiates a constant velocity move (JOG in OMS parlance). homing operation. The axis is moved at the velocity
    supplied after ramping up at the rate specified by the acceleration parameter. The motion will only stop if a limit is
    hit or a motorAxisStop command is issued.

    \param pAxis         [in]   Pointer to axis handle returned by motorAxisOpen.
    \param velocity      [in]   Velocity to ramp up to and coast at expressed in motor units/second.
    \param acceleration  [in]   Maximum acceleration (or decelleration) during velocity ramp in motor units/second squared.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/


#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisVelocityMove(  AXIS_ID pAxis, double velocity, double acceleration );
#endif

typedef int (*motorAxisStopFunc)( AXIS_ID pAxis );
/** Stops the axis from moving.

    This aborts any current motion and brings the axis to a halt at
    the current position. It takes no parameters, so it is not clear
    what deacceleration to use, but presumably it should chose a
    reasonable one - such as the one specified in the last move, home
    or jog command. The command completes as soon as the stop is initiated.

    \return Integer indicating 0 (MOTOR_AXIS_OK) for success or non-zero for failure. 
*/

#ifdef DEFINE_MOTOR_PROTOTYPES
int motorAxisStop( AXIS_ID pAxis );
#endif

/**@} end motion group*/

/** The driver support entry table */

typedef struct motorAxisDrvSET_str
{
    int number;
    motorAxisReportFunc          report;            /**< Standard EPICS driver report function (optional) */
    motorAxisInitFunc            init;              /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLogFunc          setLog;            /**< Defines an external logging function (optional) */
    motorAxisOpenFunc            open;              /**< Driver open function */
    motorAxisCloseFunc           close;             /**< Driver close function */
    motorAxisGetStatusFunc       getStatus;         /**< Returns motor status */
    motorAxisSetCallbackFunc     setCallback;       /**< Provides a callback function the driver can call when the status updates */
    motorAxisSetDoubleFunc       setPosition;       /**< Defines the current motor position to be a given value */
    motorAxisSetEncoderRatioFunc setEncoderRatio;   /**< Sets the encoder ratio */
    motorAxisSetDoubleFunc       setLowLimit;       /**< */
    motorAxisSetDoubleFunc       setHighLimit;      /**< */
    motorAxisSetIntegerFunc      setClosedLoop;     /**< */
    motorAxisSetDoubleFunc       setPGain;          /**< */
    motorAxisSetDoubleFunc       setIGain;          /**< */
    motorAxisSetDoubleFunc       setDGain;          /**< */
    motorAxisMoveFunc            move;              /**< */
    motorAxisVelocityMoveFunc    velocityMove;      /**< */
    motorAxisStopFunc            stop;              /**< */
} motorAxisDrvSET_t;
