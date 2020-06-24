/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.20
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
* 
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). You'll need to use the MAX31855 
* library for Arduino if you are having a shield of v1.60 & above which can be 
* downloaded from our GitHub repository. Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller shield. 
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x   
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
*     |         x     |                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                                                Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Required Libraries
* ==================
* - Arduino PID Library: 
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above): 
*   >> https://github.com/rocketscream/MAX31855
* - MAX6675 Library (for board v1.50 & below):
*   >> https://github.com/adafruit/MAX6675-library
*
* Revision  Description
* ========  ===========
* 1.20      Adds supports for v1.60 (and above) of Reflow Oven Controller 
*           Shield:
*           - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*             to be used for user application).
*           - Uses analog based switch (allowing D2 & D3 to be used for user 
*             application). 
*           Adds waiting state when temperature too hot to start reflow process.
*           Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10      Arduino IDE 1.0 compatible.
* 1.00      Initial public release.
*******************************************************************************/
// Comment either one the following #define to select your board revision
// Newer board version starts from v1.60 using MAX31855KASA+ chip 
#define  USE_MAX31855
// Older board version below version v1.60 using MAX6675ISA+ chip
//#define USE_MAX6675

// ***** INCLUDES *****
#include <MAX31855.h>
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1, 
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

// ***** CONSTANTS *****    Change this constant depending on the temperature profile you are looking. Values can be up to 10 degrees above set parameter. Play with the PID parameters constants to get a more accurate value
#define PU_OFFSET 20

#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 90
#define TEMPERATURE_SOAK_MAX 138
#define TEMPERATURE_REFLOW_MAX 150
#define TEMPERATURE_COOL_MIN 90
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 4 
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 100

// ***** PID PARAMETERS *****   Change this constants for the PID multiplication
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100      // Present error constant
#define PID_KI_PREHEAT 0.0001   // Integral (past) error constant
#define PID_KD_PREHEAT 5000     // Derivative (future) constant 
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 400 
#define PID_KI_SOAK 0.01 
#define PID_KD_SOAK 8000 
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 400 
#define PID_KI_REFLOW 0.0001
#define PID_KD_REFLOW 8000 
#define PID_SAMPLE_TIME 1000

// ***** PIN ASSIGNMENT *****
int ssrPin = 5;               /* Controls relay*/
int thermocoupleSOPin = 12;
int thermocoupleCSPin = 10;
int thermocoupleCLKPin = 13;
int ledRedPin = 2;
int ledGreenPin = 4;
int switchPin = A0;


// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Specify MAX6675 thermocouple interface
MAX31855 thermocouple(thermocoupleSOPin, thermocoupleCSPin, 
                      thermocoupleCLKPin);

void setup()
{
  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  pinMode(switchPin, INPUT_PULLUP);
  
  // LED pins initialization and turn on upon start-up (active low)
  digitalWrite(ledRedPin, HIGH);
  pinMode(ledRedPin, OUTPUT);
  
  digitalWrite(ledGreenPin, HIGH);
  pinMode(ledGreenPin, OUTPUT);

  // Serial communication at 57600 bps
  Serial.begin(57600);

  // Turn off GREEN LED
  digitalWrite(ledGreenPin, LOW);
  
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();

//  Serial.println("Reflow Oven time. Press button to start cycle");
  Serial.println("Setpoint,Temp,State");

}

void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    
    // Read current temperature
    input = thermocouple.readThermocouple(CELSIUS);
        
    // If thermocouple problem detected
    if (isnan(input)) {
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;

      Serial.println(" ************** ERROR ******************** ");
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += 1000;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle green LED as system heart beat
      digitalWrite(ledGreenPin, !(digitalRead(ledGreenPin)));
      // Turn off red LED as reflow is active
      digitalWrite(ledRedPin, LOW);
      
      // Increase seconds timer for reflow curve analysis
      timerSeconds++;
      // Send temperature and time stamp to serial 
      Serial.print(setpoint);
      Serial.print("\t");
      Serial.print(input);
      Serial.print("\t");
      
      switch(reflowState){
        case REFLOW_STATE_IDLE: 
          Serial.println("IDLE");
          break;
        case REFLOW_STATE_PREHEAT: 
          Serial.println("PREHEAT");
          break;
        case REFLOW_STATE_SOAK: 
          Serial.println("SOAK");
          break; 
        case REFLOW_STATE_REFLOW: 
          Serial.println("REFLOW");
          break;
        case REFLOW_STATE_COOL: 
          Serial.println("COOL");
          break;
        case REFLOW_STATE_COMPLETE: 
          Serial.println("COMPLETE");
          break; 
        case REFLOW_STATE_ERROR: 
          Serial.println("ERROR");
          break; 
        case REFLOW_STATE_TOO_HOT:
          Serial.println("TOO HOT");
          break; 
      }
    }
    else
    {
      // Turn on red LED
      digitalWrite(ledRedPin, HIGH);
      // Turn off Green LED
      digitalWrite(ledGreenPin, LOW);

    }

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // No thermocouple wire connected
      Serial.println("TC Error!");
      digitalWrite(ledGreenPin, LOW);
      digitalWrite(ledRedPin, !(digitalRead(ledRedPin)));
    }
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:
    // If oven temperature is still above room temperature
    if (input >= TEMPERATURE_ROOM)
    {
      reflowState = REFLOW_STATE_TOO_HOT;
    }
    else
    {
      // If switch is pressed to start reflow process
      if (switchStatus == SWITCH_1)
      {
        // Send header for CSV file
//        Serial.println("Reflow process started!");
        // Intialize seconds timer for serial debug information
        timerSeconds = 0;
        // Initialize PID control window starting time
        windowStartTime = millis();
        // Ramp up to minimum soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN;
        // Tell the PID to range between 0 and the full window size
        reflowOvenPID.SetOutputLimits(0, windowSize);
        reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
        // Turn the PID on
        reflowOvenPID.SetMode(AUTOMATIC);
        // Proceed to preheat stage
        reflowState = REFLOW_STATE_PREHEAT;
      }
    }
    break;

  case REFLOW_STATE_PREHEAT:
    reflowStatus = REFLOW_STATUS_ON;
    // If minimum soak temperature is achieve       
    if (input >= TEMPERATURE_SOAK_MIN)
    {
      // Chop soaking period into smaller sub-period
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Set less agressive PID parameters for soaking ramp
      reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
      // Ramp up to first section of soaking temperature
      setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;   
      // Proceed to soaking state
      reflowState = REFLOW_STATE_SOAK; 
    }
    break;

  case REFLOW_STATE_SOAK:      /* Note: Most logic in time depends on this */
    // If micro soak temperature is achieved       
    if (millis() > timerSoak)
    {
      timerSoak = millis() + SOAK_MICRO_PERIOD;
      // Increment micro setpoint
      setpoint += SOAK_TEMPERATURE_STEP;
      if (setpoint > TEMPERATURE_SOAK_MAX)
      {
        // Set agressive PID parameters for reflow ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_REFLOW_MAX;   
        // Proceed to reflowing state
        reflowState = REFLOW_STATE_REFLOW; 
      }
    }
    break; 

  case REFLOW_STATE_REFLOW:
    // We need to avoid hovering at peak temperature for too long
    // Crude method that works like a charm and safe for the components
    if (input >= (TEMPERATURE_REFLOW_MAX - 5))
    {
      // Set PID parameters for cooling ramp
      reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
      // Ramp down to minimum cooling temperature
      setpoint = TEMPERATURE_COOL_MIN;   
      // Proceed to cooling state
      reflowState = REFLOW_STATE_COOL; 
    }
    break;   

  case REFLOW_STATE_COOL:
    // If minimum cool temperature is achieve       
    if (input <= TEMPERATURE_COOL_MIN)
    {
      // Retrieve current time for buzzer usage
      buzzerPeriod = millis() + 1000;
      
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;                
      // Proceed to reflow Completion state
      reflowState = REFLOW_STATE_COMPLETE; 
    }         
    break;    

  case REFLOW_STATE_COMPLETE:
    if (millis() > buzzerPeriod)
    {
      digitalWrite(ledGreenPin, HIGH);
      // Reflow process ended
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;
  
  case REFLOW_STATE_TOO_HOT:
    // If oven temperature drops below room temperature
    if (input < TEMPERATURE_ROOM)
    {
      // Ready to reflow
      reflowState = REFLOW_STATE_IDLE;
    }
    break;
    
  case REFLOW_STATE_ERROR:
    // If thermocouple problem is still present
    if (isnan(input)){
      // Wait until thermocouple wire is connected
      reflowState = REFLOW_STATE_ERROR; 
    }
    else
    {
      // Clear to perform reflow process
      reflowState = REFLOW_STATE_IDLE; 
    }
    break;    
  }    

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;

//      Serial.println("Reflow process canceled. Wait for cooling.");
    }
  } 

  // Simple switch debounce state machine (for switch #1 (both analog & digital
  // switch supported))
  //Serial.println(analogRead(switchPin));
  switch (debounceState)
  {
  case DEBOUNCE_STATE_IDLE:
    // No valid switch press
    switchStatus = SWITCH_NONE;
    // If switch #1 is pressed

    if (analogRead(switchPin) - PU_OFFSET <= 0){
      // Intialize debounce counter
      lastDebounceTime = millis();
      // Proceed to check validity of button press
      debounceState = DEBOUNCE_STATE_CHECK;
    } 
      break;
  
    case DEBOUNCE_STATE_CHECK:
      if (analogRead(switchPin) - PU_OFFSET<= 0) {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
        // False trigger
        else
        {
          // Reinitialize button debounce state machine
          debounceState = DEBOUNCE_STATE_IDLE; 
        }
      break;
  
    case DEBOUNCE_STATE_RELEASE:
      
      if (analogRead(switchPin) > 0){
        // Valid switch 1 press
        switchStatus = SWITCH_1;
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE; 
      }
      break;

    default:
      break;
    }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if((now - windowStartTime) > windowSize)
    { 
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if(output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);   
  }
  // Reflow oven process is off, ensure oven is off
  else 
  {
    digitalWrite(ssrPin, LOW);
  }
}
