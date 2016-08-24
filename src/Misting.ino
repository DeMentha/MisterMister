#include <hd44780_I2Cexp.h>
#include <DeviceConstants.h>

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


byte MIST_MODE_PIN = 4;       // D4 - Simple on/off switch for controlling mist.
byte PRESSURE_SWITCH_PIN = 5; // D5 - Switch goes off at a given pressure value

byte PUMP_RELAY_PIN = 6;      // D6 - Enables or disables the pump
byte SPARE_RELAY_PIN = 7;     // D7 - Spare Pin
byte PUMP_SWITCH_PIN = 8;     // D8 - Pump Pressure switch


// Analog Pin Allocations
byte I2C_SCL_PIN = 5;         // A5
byte I2C_SDA_PIN = 4;         // A4
byte BAT_VOLTAGE_PIN = 3;     // A3

byte STATUS_LED = 13;

// // Battery related constants
// const float BAT_SCALING = 1.05 * 5 * ((5.1+15)/5.1) / 1024;
// const float BAT_LIMIT_HIGH = 25; // Volts
// const float BAT_LIMIT_LOW = 8;  // Volts
//
// // Flow Sensor constants
// const float PUMP_FLOW_PULSE = 2255.86; // Pulses per Gallon
// const float MIST_FLOW_PULSE = 5223.3; // Pulses per Gallon

// Timing constants
const int STEP_DELAY = 5000; // The time to wait in between each step (5 secs)

// Phase State
enum phase {
  halt,
  one,
  two,
  three
};
phase currentPhase = one;

// Result Constants returned by phase functions indicating status
int RESULT_OK = 0;
int RESULT_FAIL = 1;
int RESULT_WAIT = 2;
int currentResult = RESULT_WAIT;

int MAX_PULSES_STORED = 5; // The number of timestamps we store.

// Mist and Pump Flow Sensors data
volatile int mistPulseCount;
volatile int lastMistPulseCount;
volatile int totalMistPulseCount;

float mistFlowRate;
float totalMistVolume;
long mistFlowLastTimeChecked = millis();

volatile int pumpPulseCount;
volatile int lastPulseCount;
volatile int totalPumpPulseCount;

float pumpFlowRate; // Pump flow rate (Pulses / Second)
float pumpVolumeRate; // Pump volume (Gallons / Minute)
float totalPumpVolume; // Total Volume Pumped
long pumpFlowLastTimeChecked = millis();

int pumpDutyTime; // The number of seconds pump runs during mist phase.
int totalMistTime; // Total time in mist phase.
float pumpDutyCycle; // Duty cycle for the pump (i.e. 25%);
int printCount;

char haltReason[21];

long timeRunner;

/**
 * Class that stores all phase one related variables and functions.
 */
class PhaseOne
{
  boolean phaseOneSetupComplete = false;
  long timeRunner;
  long phaseOneInitTime;

  // Constants
  static const float PHASE_ONE_PUMP_MIN_FLOW_RATE;
  static const float PHASE_ONE_PUMP_MAX_FLOW_RATE;
  // Wait one minute before checking flow rate.
  static const double PHASE_ONE_WAIT_TIME; // Currently 60 seconds

  void (*primingValveOpen)();
  void (*mistingValveClose)();
  void (*pumpEnable)();
  void (*pumpDisable)();
  void (*updatePumpFlowRate)();
  void (*printStatus)(float);
  // void (*timeLeft)();
public:
  PhaseOne(void (*primingValveOpen)(), void (*mistingValveClose)(),
    void (*pumpEnable)(), void (*updatePumpFlowRate)(), void (*pumpDisable)(),
    void (*printStatus)(float) )
  {
    this->primingValveOpen = primingValveOpen;
    this->mistingValveClose = mistingValveClose;
    this->pumpEnable = pumpEnable;
    this->pumpDisable = pumpDisable;
    this->updatePumpFlowRate = updatePumpFlowRate;
    this->printStatus = printStatus;
  }

  void setup();
  int check();
  void tearDown();
};

const float PhaseOne::PHASE_ONE_PUMP_MAX_FLOW_RATE = 1.8;
const float PhaseOne::PHASE_ONE_PUMP_MIN_FLOW_RATE = 0.8;
const double PhaseOne::PHASE_ONE_WAIT_TIME = 60 * 1 * 1000; // Currently 60 seconds

/**
 * Gets the system ready for phase one.
 */
void PhaseOne::setup()
{
  if (phaseOneSetupComplete) {
    return;
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Phase 1: Priming");
  lcd.setCursor(0,2);
  lcd.print("Drink and Spit!");
  lcd.setCursor(0,3);
  lcd.print("Drink and Spit!");
  mistingValveClose();
  primingValveOpen();
  delay(3000);
  pumpEnable();
  timeRunner = millis();
  phaseOneInitTime = millis();
  currentResult = RESULT_WAIT;
  updatePumpFlowRate();

  phaseOneSetupComplete = true;
}

/**
 * Checks if phase one has completed succesfully. Returns OK, FAIL or WAIT.
 * WAIT means we're still waiting for the result.
 */
int PhaseOne::check() {
  if (millis() - timeRunner > 1000) { // Update values every two seconds
    updatePumpFlowRate();
    timeRunner = millis();
    if (millis() - phaseOneInitTime > 60000) {
      if (pumpFlowRate >= 0.8 && pumpFlowRate <= 1.8) {
        tearDown();
        return RESULT_OK;
      } else {
        if (pumpFlowRate < 0.8) {
          sprintf(haltReason, "1: HALT PFlow Low");
        } else {
          sprintf(haltReason, "1: HALT PFlow High");
        }
        tearDown();
        return RESULT_FAIL;
      }
    }
    float timeLeft = ( (float) phaseOneInitTime
        + 60000 - (float)millis() ) / (float) 1000;
    printStatus(timeLeft);
  }
  return RESULT_WAIT;
}

void PhaseOne::tearDown() {
  pumpDisable();
}

PhaseOne phaseOne(&primingValveOpen, &mistingValveClose, &pumpEnable,
    updatePumpFlowRate, &pumpDisable, &printStatus);

/**
 * Class that stores all phase one related variables and functions.
 */
class PhaseTwo
{
  boolean phaseTwoSetupComplete = false;
  long timeRunner;
  long phaseTwoInitTime;

  // Constants
  static const float PHASE_TWO_MIN_VOLUME;
  static const float PHASE_TWO_MAX_VOLUME;

  // Wait one minute before checking flow rate.
  static const long PHASE_TWO_WAIT_TIME;

  void (*mistingValveClose)();
  void (*primingValveClose)();
  void (*pumpEnable)();
  void (*pumpDisable)();
  void (*updatePumpFlowRate)();
  bool (*readPressureSwitch)();
  void (*printStatus)(float);
public:
  PhaseTwo(void (*mistingValveClose)(), void (*primingValveClose)(),
    void (*pumpEnable)(), void (*updatePumpFlowRate)(), void (*pumpDisable)(),
    bool (*readPressureSwitch)(), void (*printStatus)(float))
  {
    this->mistingValveClose = mistingValveClose;
    this->primingValveClose = primingValveClose;
    this->pumpEnable = pumpEnable;
    this->pumpDisable = pumpDisable;
    this->updatePumpFlowRate = updatePumpFlowRate;
    this->readPressureSwitch = readPressureSwitch;
    this->printStatus = printStatus;
  }

  void setup();
  int check();
  void tearDown();
};

const float PhaseTwo::PHASE_TWO_MIN_VOLUME = 1.5;
const float PhaseTwo::PHASE_TWO_MAX_VOLUME = 2.5;
const long PhaseTwo::PHASE_TWO_WAIT_TIME = 60 * 1 * 1000; // Currently 60 seconds

void PhaseTwo::setup() {
  if (phaseTwoSetupComplete) {
    return;
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Phase 2: Accumulate");
  lcd.setCursor(0,2);
  lcd.print("Mister Mister is");
  lcd.setCursor(0,3);
  lcd.print("fucking thirsty!");

  totalPumpPulseCount = 0;
  mistingValveClose();
  primingValveClose();
  delay(3000);
  pumpEnable();
  timeRunner = millis();
  phaseTwoInitTime = millis();
  currentResult = RESULT_WAIT;
  updatePumpFlowRate();

  phaseTwoSetupComplete = true;
}

int PhaseTwo::check() {
  if (millis() - timeRunner > 1000) {
    updatePumpFlowRate();
    timeRunner = millis();
    if (millis() - phaseTwoInitTime > 60000) {
      if (totalPumpVolume >= 0.25 && totalPumpVolume <= 1.0
          && readPressureSwitch() && !readPumpSwitch()) {
        tearDown();
        return RESULT_OK;
      } else {
        if (totalPumpVolume < 0.25) {
          sprintf(haltReason, "2: HALT Vol Low");
        } else if (totalPumpVolume > 1.0) {
          sprintf(haltReason, "2: HALT Vol High");
        } else if (!readPressureSwitch()) {
          sprintf(haltReason, "2: HALT PressSwitch 0");
        } else if (readPumpSwitch()) {
          sprintf(haltReason, "2: HALT Pump On");
        }
        tearDown();
        return RESULT_FAIL;
      }
    }
    float timeLeft = ( (float) phaseTwoInitTime
        + 60000 - (float)millis() ) / (float) 1000;
    printStatus(timeLeft);
  }
  return RESULT_WAIT;
}

void PhaseTwo::tearDown() {
  pumpDisable();
}

PhaseTwo phaseTwo(&mistingValveClose, &primingValveClose, &pumpEnable,
    updatePumpFlowRate, &pumpDisable, &readPressureSwitch, &printStatus);

/**
 * Class that stores all phase one related variables and functions.
 */
class PhaseThree
{
  boolean phaseThreeSetupComplete = false;
  long timeRunner;
  long phaseThreeInitTime;
  int errorCount;
  bool mistOn;

  // Constants
  // static const float PHASE_TWO_MIN_VOLUME;
  // static const float PHASE_TWO_MAX_VOLUME;

  // Wait one minute before checking flow rate.
  // static const long PHASE_TWO_WAIT_TIME;

  void (*mistingValveOpen)();
  void (*mistingValveClose)();
  void (*primingValveClose)();
  void (*pumpEnable)();
  void (*pumpDisable)();
  void (*updatePumpFlowRate)();
  void (*updateMistFlowRate)();
  void (*printStatus)(float);
public:
  PhaseThree(void (*mistingValveOpen)(), void (*mistingValveClose)(),
    void (*primingValveClose)(), void (*pumpEnable)(),
    void (*updatePumpFlowRate)(), void (*pumpDisable)(),
    void (*updateMistFlowRate)(), void (*printStatus)(float))
  {
    this->mistingValveOpen = mistingValveOpen;
    this->mistingValveClose = mistingValveClose;
    this->primingValveClose = primingValveClose;
    this->pumpEnable = pumpEnable;
    this->pumpDisable = pumpDisable;
    this->updatePumpFlowRate = updatePumpFlowRate;
    this->updateMistFlowRate = updateMistFlowRate;
    this->printStatus = printStatus;
  }

  void setup();
  int check();
  void tearDown();
};

// const float PhaseTwo::PHASE_TWO_MIN_VOLUME = 1.5;
// const float PhaseTwo::PHASE_TWO_MAX_VOLUME = 2.5;
// const long PhaseTwo::PHASE_TWO_WAIT_TIME = 60 * 1 * 1000; // Currently 60 seconds

void PhaseThree::setup() {
  if (phaseThreeSetupComplete) {
    return;
  }

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Phase 3: Mist");
  lcd.setCursor(0,2);
  lcd.print("People 'bout to get");
  lcd.setCursor(0,3);
  lcd.print("fucking wet!");
  primingValveClose();
  mistingValveClose(); // check() method will read mist mode switch and open.
  delay(3000);
  pumpEnable();
  timeRunner = millis();
  phaseThreeInitTime = millis();
  pumpDutyTime = 0;
  currentResult = RESULT_WAIT;
  updateMistFlowRate();

  phaseThreeSetupComplete = true;
}

int PhaseThree::check() {
  if (millis() - timeRunner > 1000) {
    bool mistSwitch = readMistSwitch();
    timeRunner = millis();
    if (!mistSwitch) {
      // Mist turned off. Close valve.
      mistOn = false;
      mistingValveClose();
    } else {
      if (!mistOn) {
        // Mist just turned on, so reset init time.
        mistOn = true;
        phaseThreeInitTime = millis();
      }
      mistingValveOpen();
      updatePumpFlowRate();
      updateMistFlowRate();
      updatePumpDutyCycle();
      totalMistTime++;
      if (millis() - phaseThreeInitTime > 40000) {
        // Continuously check error cases.
        bool mistFlowGood = mistFlowRate >= 2.5 && mistFlowRate <= 9.5;
        bool pumpSwitch = readPumpSwitch();
        bool pumpFlowGood = !pumpSwitch
            || (pumpSwitch && pumpFlowRate >= 0.8 && pumpFlowRate <= 1.8);
        bool dutyCycleGood = pumpDutyCycle >= 0 && pumpDutyCycle <= 20;
        if (mistFlowGood && pumpFlowGood && dutyCycleGood) {
          errorCount = 0;
        } else {
          if (errorCount > 5) {
            if (!pumpFlowGood) {
              sprintf(haltReason, "3: HALT PFlow Bad");
            } else if (!mistFlowGood) {
              sprintf(haltReason, "3: HALT MFlow Bad");
            } else if (!dutyCycleGood) {
              sprintf(haltReason, "3: HALT DCycle Bad");
            }
            tearDown();
            return RESULT_FAIL;
          } else {
            errorCount++;
          }
        }
      }
    }

    float timeLeft = ( (float) phaseThreeInitTime
        + 40000 - (float)millis() ) / (float) 1000;
    printStatus(mistOn ? timeLeft : -1);
  }
  return RESULT_WAIT;
}

void PhaseThree::tearDown() {
  mistingValveClose();
  primingValveClose();
  pumpDisable();
}


PhaseThree phaseThree(&mistingValveOpen, &mistingValveClose, &primingValveClose,
    &pumpEnable, updatePumpFlowRate, &pumpDisable, &updateMistFlowRate,
    &printStatus);


/*
* This is run once by the arduino board.
*/
void setup() {
  // Logger::start();
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
  lcd.print("DEMENTHA MISTING 2.0");

  // Setup pins for all our switches (prime valve, mist valve and pump enable)
  pinMode(MIST_MODE_PIN, INPUT_PULLUP);
  pinMode(PRESSURE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(PUMP_SWITCH_PIN, INPUT);

  pinMode(PRIME_VALVE_PIN, OUTPUT);
  primingValveClose();
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

  timeRunner = millis();

  // Run preconditionsCheck to make sure system is ready
  preconditionsCheck();
}

void loop() {
  if (currentPhase == one) {
    phaseOne.setup();
    currentResult = phaseOne.check();
    if (currentResult == RESULT_OK) {
      printStatus(0);
      delay(2000);
      currentPhase = two;
    } else if (currentResult == RESULT_FAIL) {
      printStatus(0);
      delay(2000);
      currentPhase = halt;
    }
  } else if (currentPhase == two) {
    phaseTwo.setup();
    currentResult = phaseTwo.check();
    if (currentResult == RESULT_OK) {
      printStatus(0);
      delay(2000);
      currentPhase = three;
    } else if (currentResult == RESULT_FAIL) {
      printStatus(0);
      delay(2000);
      currentPhase = halt;
    }
  } else if (currentPhase == three) {
    phaseThree.setup();
    currentResult = phaseThree.check();
    if (currentResult == RESULT_WAIT) {
      // Misting is going well.
    } else if (currentResult == RESULT_FAIL) {
      // Something messed up.
      printStatus(0);
      delay(2000);
      currentPhase = halt;
    }
  } else if (currentPhase == halt) {
      // DO Nothing.
      if (millis() - timeRunner > 1000) {
        printStatus(0);
        timeRunner = millis();
      }
  }
  //
  // } else if (currentPhase == three) {
  //
}

// Functions defining major steps in our state machine.

/**
 * Checks if the system is ready to start.
 */
void preconditionsCheck() {
  checkBatteryVoltage(true);
  delay(5000);
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

// long getPumpFlowRate() {
//   long avgTimePassed = pumpPulseTimestamps.back() - pumpPulseTimestamps.front();
//   return (long) pumpPulseTimestamps.size() / avgTimePassed;
// }

/*
 * Updates the Mist Flow Rate (pulses / second) and volume.
 */
void updateMistFlowRate() {
  long count = mistPulseCount - lastMistPulseCount;
  float timePassed = (millis() - mistFlowLastTimeChecked) / 1000;

  lastMistPulseCount = mistPulseCount;
  mistFlowLastTimeChecked = millis();

  mistFlowRate = ((float) count / (float) timePassed / (float) MIST_FLOW_PULSE) * 60 * 60;
  totalMistVolume = totalMistPulseCount / (float) MIST_FLOW_PULSE;
}

/*
 * Updates the Pump Flow Rate (pulses / second) and volume.
 */
void updatePumpFlowRate() {
  long count = pumpPulseCount - lastPulseCount;
  float timePassed = (millis() - pumpFlowLastTimeChecked) / 1000;

  lastPulseCount = pumpPulseCount;
  pumpFlowLastTimeChecked = millis();

  pumpFlowRate = ((float) count / (float) timePassed / (float) PUMP_FLOW_PULSE) * 60;
  pumpVolumeRate = pumpPulseCount / (float) PUMP_FLOW_PULSE;
  totalPumpVolume = totalPumpPulseCount / (float) PUMP_FLOW_PULSE;
}

/**
 * Update the pump duty cycle.
 */
void updatePumpDutyCycle() {
  if (totalMistTime == 0 || currentPhase != three) {
    pumpDutyTime = 0;
    pumpDutyCycle = 0;
  } else {
    pumpDutyTime += readPumpSwitch() ? 1 : 0;
    pumpDutyCycle = ( (float) pumpDutyTime / (float) totalMistTime )
        * 100.0f;
  }
}

// Phase 1: Priming
// PF:1.2G/s V:0.55G   |
// MF:0.0G/h V:0.42    |
// PS:Off Pu:Off M:Off
/**
 * Prints current status of all metrics.
 */
void printStatus(float timeLeft) {
  char tempBuffer[6];
  char strBuffer[21];
  lcd.clear();
  lcd.setCursor(0,0);
  dtostrf(timeLeft, 3, 0, tempBuffer);
  if (currentPhase == one) {
    if (currentResult == RESULT_WAIT) {
      sprintf(strBuffer,"1: Priming %ss", tempBuffer);
    } else if (currentResult == RESULT_OK) {
      sprintf(strBuffer,"1: Primed :) %ss", tempBuffer);
    } else if (currentResult == RESULT_FAIL) {
      sprintf(strBuffer,"1: HALT! %ss", tempBuffer);
    }
    lcd.print(strBuffer);
  } else if (currentPhase == two) {
    if (currentResult == RESULT_WAIT) {
      sprintf(strBuffer,"2: Accumulate %ss", tempBuffer);
    } else if (currentResult == RESULT_OK) {
      sprintf(strBuffer,"2: Loaded :) %ss", tempBuffer);
    } else if (currentResult == RESULT_FAIL) {
      sprintf(strBuffer,"2: HALT! %ss", tempBuffer);
    }
    lcd.print(strBuffer);
  } else if (currentPhase == three) {
    if (currentResult == RESULT_WAIT) {
      sprintf(strBuffer, "3: %s", readMistSwitch() ? "Misting" : "Mist OFF");
      if (timeLeft >= 0) {
        sprintf(strBuffer, "%s %ss", strBuffer, tempBuffer);
      }

      lcd.print(strBuffer);
    } else if (currentResult == RESULT_FAIL) {
      lcd.print("3: Misting HALT!");
    }
  } else if (currentPhase == halt) {
    lcd.print(haltReason);
  }

  // Pump Flow Metrics
  dtostrf(pumpFlowRate, 4, 2, tempBuffer);
  sprintf(strBuffer,"PF:%sG/m", tempBuffer);
  dtostrf(totalPumpVolume, 4, 2, tempBuffer);
  sprintf(strBuffer,"%s V:%sG", strBuffer, tempBuffer);
  lcd.setCursor(0,1);
  lcd.print(strBuffer);

  // Mist Flow Metrics
  dtostrf(mistFlowRate, 4, 2, tempBuffer);
  sprintf(strBuffer,"MF:%sG/h", tempBuffer);
  dtostrf(totalMistVolume, 4, 2, tempBuffer);
  sprintf(strBuffer,"%s V:%sG", strBuffer, tempBuffer);
  lcd.setCursor(0,2);
  lcd.print(strBuffer);

  if (printCount < 5) {
    // Switches
    bool pressureSwitch = readPressureSwitch();
    bool pumpPressureSwitch = readPumpSwitch();
    bool mistSwitch = readMistSwitch();
    dtostrf(pumpDutyCycle, 2, 0, tempBuffer);
    sprintf(strBuffer, "P:%s Pu:%s D:%s%% M:%s", pressureSwitch ? "1" : "0",
        pumpPressureSwitch ? "1" : "0", tempBuffer,
        mistSwitch ? "1" : "0");
    lcd.setCursor(0,4);
    lcd.print(strBuffer);
  } else {
    dtostrf(readBatteryVoltage(), 4, 2, tempBuffer);
    sprintf(strBuffer,"BATT VOLTAGE: %sV", tempBuffer);
    lcd.setCursor(0, 4);
    lcd.print(strBuffer);
  }

  if (printCount >= 9) {
    printCount = 0;
  } else {
    printCount++;
  }
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

  // TODO: Only cancel if below limite for x seconds.
  if (temp < BAT_LIMIT_LOW)
  {
    long startTime = millis();
    while(1) {
      if (startTime + 4000 > millis()) {
       startTime = millis();
       char strBuffer[21];
       char tempBuffer[6];
       lcd.setCursor(0, 2);
       dtostrf(BAT_LIMIT_LOW, 4, 2, tempBuffer);
       sprintf(strBuffer,"LOW LIMIT:    %sV", tempBuffer);
       //lcd.print(strBuffer);
       lcd.print(strBuffer);
       lcd.setCursor(0, 3);
       lcd.print("ERROR: HALT SYSTEM!");
     }
   }
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
  totalMistPulseCount++;
}

/*
 * Interrupt Service Routine for Pump Flow Sensor
 */
void pumpPulseCounterISR()
{
  pumpPulseCount++;
  totalPumpPulseCount++;
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

bool readMistSwitch()
{
  return digitalRead(MIST_MODE_PIN);
}

bool readPressureSwitch()
{
  return !digitalRead(PRESSURE_SWITCH_PIN);
}

bool readPumpSwitch() {
  return digitalRead(PUMP_SWITCH_PIN);
}
