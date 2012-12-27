
/* ======== Variables for EngGDriver.c ======== */

/* --- Timers ---*/
volatile int32_t timer20 = 0;           // timer in 1/20th of a millisecond
volatile uint8_t timerPrescaler = 0;    // used to increment timer
volatile int32_t timer = 0;             // timer in milliseconds
/* --------------*/

/* --- Speed and Acceleration --- */
volatile int32_t ticks0 = 0;            // encoder values for motor0 (i.e. tachometer)
volatile int32_t ticks1 = 0;            // encoder values for motor1 (i.e. tachometer)
    // Timestamps of previous encoder interrupts:
volatile int32_t int0Time0 = 0;         // latest timestamp from encoder 0 interrupt
volatile int32_t int0Time1 = 0;         // previous timestamp from encoder 0 interrupt
volatile int32_t int1Time0 = 0;         // latest timestamp from encoder 1 interrupt
volatile int32_t int1Time1 = 0;         // previous timestamp from encoder 1 interrupt
int32_t int0TimeCS = 0, int1TimeCS = 0; // encoder timestamps when last called calculateSpeeds();
int32_t ticks0CS = 0, ticks1CS = 0;     // ticks# value when int#TimeCS was recorded
volatile int8_t int0dir = FORWARD;      // latest recorded direction of rotation
volatile int8_t int1dir = FORWARD;      // latest recorded direction of rotation
    // Note: Speed is in ticks / sec, where 1 tick = 0.110 mm
int16_t speed0 = 0, speed1 = 0;         // speed of motor in ticks / sec
int16_t accel0 = 0, accel1 = 0;         // acceleration of motor in ticks / sec^2
int16_t lastSpeed0[8], lastSpeed1[8];   // values used by calculateSpeeds() for acceleration
int16_t lastSpeedInd = 0;               // index for the current value in lastSpeed buffer
/* --------------*/

/* --- Odometry and position correction --- */
    // Legend: L - Left motor, R - Right motor
int32_t p_Lfull = 0, p_Rfull = 0;       // position, full precision (mm / 1024) 
int16_t p_L = 0, p_R = 0;               // position, in mm
int32_t p_Lticks = 0, p_Rticks = 0;     // tacho count on last odometer run
int8_t p_Ltrans = 0, p_Rtrans = 0;      // Which transition region we are located in
int16_t p_Lrel = 0, p_Rrel = 0;         // position, relative to section
    // List of absolute positions for each track sensor transition
int16_t p_transLlist[TRANSITIONS] = TRANS_L_LIST;
int16_t p_transRlist[TRANSITIONS] = TRANS_R_LIST;
    // position correction
uint8_t p_LsensVal = 0, p_RsensVal = 0; // last track sensor value (0 | 1)
//int16_t p_LsensPos = 0, p_RsensPos = 0; // ticks at last sensor poll
int16_t p_Lerr = 0, p_Rerr = 0;          // last track sensor position correction error
int8_t posCorrectionOn = 0;             // toggle position correction
uint16_t posCorrLeftFailed = 0;         // failed position correction left counter
uint16_t posCorrRightFailed = 0;        // failed position correction right counter
uint8_t posCorrLready = 0;              // determines if left sensor will trigger on next plank
uint8_t posCorrRready = 0;              // determines if right sensor will trigger on next plank
/* --------------*/

/* --- PID parameters --- */
uint8_t kP = 128;                        // P Constant
uint8_t kI = 32;                        // I Constant
uint8_t kD = 32;                        // D Constant
uint8_t kX = 0;                         // Cross dependency between both motor displacements
int16_t errIMax = 1600;                  // Max Integer value
int16_t errIMin = -1600;         
int16_t adjustMax = 30;                 // Max power adjustment factor
int16_t adjustMin = -20;
    // legacy code, used for cross-adjustment:
int16_t adjustXMax = 50;                // Max power adjustment from kX
int16_t adjustXMin = -50;           
uint8_t xCalibration = 128;             // calibrates motor1 to motor0 using ticks. 128 for no calibration
/* --------------*/

/* --- Motor Control and PID variables --- */
uint8_t pidOn = 1;                      // turns PID on or off
uint8_t ldir = FORWARD;                 // direction in which left Motor is set to rotate
uint8_t rdir = FORWARD;                 // direction in which left Motor is set to rotate
uint16_t targetSpeed0 = 0;              // target speed of motor 0
uint16_t targetSpeed1 = 0;              // target speed of motor 1
int16_t errI0 = 0, errI1 = 0;           // Accumulator for the PID I part
uint16_t power0 = 0, power1 = 0;        // power (0 to 100 * 256) applied to each motor
uint8_t moving0 = 0, moving1 = 0;       // presence of power on each motor (used for arrow indicators)
uint8_t arrowsAuto = 0;                 // define the behaviour of arrows (manual = 0 | auto = 1);
    // legacy code, used for cross-adjustment:
uint8_t adjXOn = 0;                     // enable or disable cross adjustment
int16_t adjustX = 0;                    // current cross adjustment
/* --------------*/

/* --- Navigation variables --- */
typedef enum {
    NAV_NONE = 0,
    NAV_DEST,
    NAV_FREE,
} NavCom;                               // enum for possible navigation states
NavCom navCom = NAV_NONE;               // navigator state
    // NAV_DEST mode variables:
int16_t n_targetLpos = 0;               // target L destination
int16_t n_targetRpos = 0;               // target R destination
uint8_t n_Ldone, n_Rdone;               // set to 1 when L/R side reached destination
/* --------------*/

/* --- Debug --- */
uint8_t debug1 = 0;
uint8_t debug2 = 0;
uint8_t debug3 = 0;
uint8_t debug4 = 0;
uint8_t debug5 = 0;
uint8_t debug6 = 0;
uint8_t debug7 = 0;
uint8_t debugPID = 0;
uint16_t debugPeriod = 500;
/* --------------*/

/* This statement allows printf to work with serial com
 * for every character sent to stream uart_stdout, uart_put is executed
 * later in code, stdout is set to uart_stdout, so that
 * printf writes to the stream uart_stdout automatically 
 */
static FILE uart_stdout = FDEV_SETUP_STREAM(uart_put, NULL, _FDEV_SETUP_WRITE);

/* ======================== */
