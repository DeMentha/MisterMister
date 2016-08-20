#include <hd44780_I2Cexp.h>
#include <MistLCD.h>

hd44780_I2Cexp lcd;

// LCD geometry
const int LCD_ROWS = 4;
const int LCD_COLS = 20;

// Digital Pin Allocations
byte BAT_PIN = 0;
byte PRIME_VALVE_PIN = 0;     // D0 - Controls water flow for priming phase
byte MIST_VALVE_PIN = 1;      // D1 - Controls water flow for mist phase

byte PUMP_FLOW_PIN = 2;       // D2 - Flow sensor for pump (counts pulses)
                              // must be 2 or 3 for interrupt service pin. This
                              // will interrupt main service thread allowing us
                              // to count pulses.

byte MIST_FLOW_PIN = 3;       // D3 - Mist sensor must be 2 or 3 for reasons
                              // stated above.

byte PRESSURE_SWITCH_PIN = 4; // D4 - Switch goes off at a given pressure value

byte MIST_MODE_PIN = 5;       // D5 - Simple on/off switch for controlling mist.
byte PUMP_RELAY_PIN = 6;      // D6 - Enables or disables the pump
byte SPARE_RELAY_PIN = 7;     // D7 - Spare Pin
byte PUMP_SWITCH_PIN = 8;     // D8 - ?


// Analog Pin Allocations
byte I2C_SCL_PIN = 5;         // A5
byte I2C_SDA_PIN = 4;         // A4
byte BAT_VOLTAGE_PIN = 3;     // A3

byte STATUS_LED = 13;

// Battery related constants
const float BAT_SCALING = 1.05 * 5 * ((5.1+15)/5.1) / 1024;
const float BAT_LIMIT_HIGH = 15; // Volts
const float BAT_LIMIT_LOW = 11;  // Volts

// Flow Sensor constants
const int PUMP_FLOW_PULSE = 174; // Pulses per Gallon
const int MIST_FLOW_PULSE = 380; // Pulses per Gallon

// Timing constants
const int STEP_DELAY = 5000; // The time to wait in between each step (5 secs)

// Phase State
enum phase {
  one,
  two,
  three
};
phase currentPhase = one;

// Result Constants returned by phase functions indicating status
int RESULT_OK = 0;
int RESULT_FAIL = 1;
int RESULT_WAIT = 2;

// Mist and Pump Flow Sensors data
volatile int mistPulseCount;
volatile int pumpPulseCount;

float pumpFlowRate; // Pump flow rate (Pulses / Second)
float pumpVolumeRate; // Pump volume (Gallons / Minute)
long pumpFlowLastTimeChecked = millis();

/**
 * Class that stores all phase one related variables and functions.
 */
class PhaseOne
{
  boolean phaseOneSetupComplete = false;
  long phaseOneStartTime;
  const int PHASE_ONE_PUMP_MIN_FLOW_RATE = 200;
  const int PHASE_ONE_PUMP_MAX_FLOW_RATE = 400;

  // Wait one minute before checking flow rate.
  const int PHASE_ONE_WAIT_TIME = 2 * 1 * 1000;

  void (*primingValveOpen)();
  void (*mistingValveClose)();
  void (*pumpEnable)();
  void (*updatePumpFlowRate)();
public:
  PhaseOne(void (*primingValveOpen)(), void (*mistingValveClose)(),
    void (*pumpEnable)(), void (*updatePumpFlowRate)())
  {
    this->primingValveOpen = primingValveOpen;
    this->mistingValveClose = mistingValveClose;
    this->pumpEnable = pumpEnable;
    this->updatePumpFlowRate = updatePumpFlowRate;
  }

  void setup();
  int check();
};

/**
 * Gets the system ready for phase one.
 */
void PhaseOne::setup()
{
  if (phaseOneSetupComplete) {
    return;
  }

  primingValveOpen();
  mistingValveClose();
  pumpEnable();
  phaseOneStartTime = millis();

  updatePumpFlowRate();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Flow Rate: ");
  lcd.setCursor(0,1);
  lcd.print(pumpFlowRate);

  phaseOneSetupComplete = true;
}

/**
 * Checks if phase one has completed succesfully. Returns OK, FAIL or WAIT.
 * WAIT means we're still waiting for the result.
 */
int PhaseOne::check() {
  if (millis() - phaseOneStartTime > PHASE_ONE_WAIT_TIME) {
    updatePumpFlowRate();
    if (pumpFlowRate > PHASE_ONE_PUMP_MIN_FLOW_RATE
        && pumpFlowRate < PHASE_ONE_PUMP_MAX_FLOW_RATE) {
      return RESULT_OK;
    } else {
      return RESULT_FAIL;
    }
  }
  return RESULT_WAIT;
}

PhaseOne phaseOne(&primingValveOpen, &mistingValveClose, &pumpEnable,
    updatePumpFlowRate);

/*
* This is run once by the arduino board.
*/
void setup() {
  // initialize LCD with number of columns and rows:
	if (lcd.begin(LCD_COLS, LCD_ROWS))
	{
		// begin() failed so blink the onboard LED if possible
    #ifdef LED_BUILTIN
    		pinMode(LED_BUILTIN, OUTPUT);
    		while(1)
    		{
    			digitalWrite(LED_BUILTIN, HIGH);
    			delay(500);
    			digitalWrite(LED_BUILTIN, LOW);
    			delay(500);
    		}
    #else
    		while(1){} // spin and do nothing
    #endif
  }

  // Print welcome message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DEMENTHA MISTING 2.0 - MISTER MISTER");

  // Setup pins for all our switches (prime valve, mist valve and pump enable)
  pinMode(PRIME_VALVE_PIN, OUTPUT);
  primingValveOpen();
  pinMode(MIST_VALVE_PIN, OUTPUT);
  mistingValveClose();
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pumpDisable();
  pinMode(SPARE_RELAY_PIN, OUTPUT);
  digitalWrite(SPARE_RELAY_PIN,HIGH);

  // pumpFlowLastTimeChecked = millis();

  // Attach interrupts to our two sensors: pump flow and mist flow sensors.
  pinMode(PUMP_FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUMP_FLOW_PIN), pumpPulseCounterISR,
      RISING);

  pinMode(MIST_FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MIST_FLOW_PIN), mistPulseCounterISR,
      RISING);

  // Setup the status LED line as an output
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);  // We have an active-low LED attached

  // Run preconditionsCheck to make sure system is ready
  preconditionsCheck();
}

void loop() {
  continuousHaltCheck();
  if (currentPhase == one) {
    phaseOne.setup();

    int check = phaseOne.check();
    if (check == RESULT_OK) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("This bitch is primed!");
      // Update currentPhase and move onto phase 2.
      delay(1000);
    } else if (check == RESULT_FAIL) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Somebody fucked up...");
      delay(1000);
    }
  } else if (currentPhase == two) {

  } else if (currentPhase == three) {

  }
}

// Functions defining major steps in our state machine.

/**
 * Checks if the system is ready to start.
 */
void preconditionsCheck() {
  checkBatteryVoltage(true);
  delay(1000);
}

// Methods to execute specific tasks

/**
 * Checks any general halt conditions.
 */
void continuousHaltCheck() {
  // Battery Voltage Check
  checkBatteryVoltage(false);
}

// Flow rate related methods

/*
 * Updates the Pump Flow Rate (pulses / second) and volume.
 */
void updatePumpFlowRate() {
  long count = pumpPulseCount;
  float timePassed = (millis() - pumpFlowLastTimeChecked) / 1000;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(count);
  lcd.setCursor(0,1);
  lcd.print(timePassed);
  pumpPulseCount = 0;
  pumpFlowLastTimeChecked = millis();

  pumpFlowRate = (float) count / (float) timePassed;
  pumpVolumeRate = (float) count / (float) PUMP_FLOW_PULSE;
}

/**
 * Simply reads the battery voltage and returns the float value.
 */
float readBatteryVoltage()
{
  return analogRead(BAT_VOLTAGE_PIN)*BAT_SCALING;
}

void checkBatteryVoltage(boolean displayVoltage)
{
  char strBuffer[21];
  char tempBuffer[6];

  float temp = readBatteryVoltage();

  // Check battery voltage
  dtostrf(readBatteryVoltage(), 4, 2, tempBuffer);
  if (displayVoltage) {
    sprintf(strBuffer,"BATT VOLTAGE: %sV", tempBuffer);
    lcd.setCursor(0, 1);
    lcd.print(strBuffer);
  }

  if (temp < BAT_LIMIT_LOW)
  {
     char strBuffer[21];
     char tempBuffer[6];
     lcd.setCursor(0, 2);
     dtostrf(BAT_LIMIT_LOW, 4, 2, tempBuffer);
     sprintf(strBuffer,"LOW LIMIT:    %sV", tempBuffer);
     //lcd.print(strBuffer);
     lcd.print(strBuffer);
     lcd.setCursor(0, 3);
     lcd.print("ERROR: HALT SYSTEM!");
     while(1);
  } else if (temp > BAT_LIMIT_HIGH)
  {
     char strBuffer[21];
     char tempBuffer[6];
     lcd.setCursor(0, 2);
     dtostrf(BAT_LIMIT_HIGH, 4, 2, tempBuffer);
     sprintf(strBuffer,"HIGHT LIMIT:   %sV", tempBuffer);
     //lcd.print(strBuffer);
     lcd.print(strBuffer);
     lcd.setCursor(0, 3);
     lcd.print("ERROR: HALT SYSTEM!");
     while(1);
  }
  if (displayVoltage) {
    lcd.setCursor(0,2);
    lcd.print("BATTERY VOLTAGE OK.");
  }
}

//
// Interrupt Service Routines
//

/*
 * Interrupt Service Routine for Mist Flow Sensor
 */
void mistPulseCounterISR()
{
  // Increment the mist pulse counter
  mistPulseCount++;
}

/*
 * Interrupt Service Routine for Pump Flow Sensor
 */
void pumpPulseCounterISR()
{
  // Increment the pump pulse counter
  pumpPulseCount++;
}


// Simple on/off methods for each switch


void pumpEnable()
{
  digitalWrite(PUMP_RELAY_PIN,LOW);
}

void pumpDisable()
{
  digitalWrite(PUMP_RELAY_PIN,HIGH);
}

void mistingValveOpen()
{
  digitalWrite(MIST_VALVE_PIN,LOW);
}

void mistingValveClose()
{
  digitalWrite(MIST_VALVE_PIN,HIGH);
}

void primingValveOpen()
{
  digitalWrite(PRIME_VALVE_PIN,LOW);
}

void primingValveClose()
{
  digitalWrite(PRIME_VALVE_PIN,HIGH);
}
