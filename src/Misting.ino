#include <hd44780_I2Cexp.h>
#include <Logger.h>
#include <StandardCplusplus.h>
// #include <queue>

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
int RXLED = 17;

// Battery related constants
const float BAT_SCALING = 1.05 * 5 * ((5.1+15)/5.1) / 1024;
const float BAT_LIMIT_HIGH = 25; // Volts
const float BAT_LIMIT_LOW = 8;  // Volts

// Flow Sensor constants
const float PUMP_FLOW_PULSE = 2255.86; // Pulses per Gallon
const float MIST_FLOW_PULSE = 5223.3; // Pulses per Gallon

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

// queue<long> mistPulseTimestamps;
// std::queue<long> pumpPulseTimestamps;

int MAX_PULSES_STORED = 5; // The number of timestamps we store.

float pumpFlowRate; // Pump flow rate (Pulses / Second)
float pumpVolumeRate; // Pump volume (Gallons / Minute)
float totalPumpVolume; // Total Volume Pumped
long pumpFlowLastTimeChecked = millis();



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
  // void (*timeLeft)();
public:
  PhaseOne(void (*primingValveOpen)(), void (*mistingValveClose)(),
    void (*pumpEnable)(), void (*updatePumpFlowRate)(), void (*pumpDisable)())
  {
    this->primingValveOpen = primingValveOpen;
    this->mistingValveClose = mistingValveClose;
    this->pumpEnable = pumpEnable;
    this->pumpDisable = pumpDisable;
    this->updatePumpFlowRate = updatePumpFlowRate;
    // this->timeLeft = timeLeft;
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

  mistingValveClose();
  primingValveOpen();
  delay(4000);
  pumpEnable();
  timeRunner = millis();
  phaseOneInitTime = millis();
  updatePumpFlowRate();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Phase 1: Priming");
  lcd.setCursor(0,1);
  lcd.print("Let's Move Water");

  phaseOneSetupComplete = true;
}

/**
 * Checks if phase one has completed succesfully. Returns OK, FAIL or WAIT.
 * WAIT means we're still waiting for the result.
 */
int PhaseOne::check() {
  if (millis() - timeRunner > 2000) { // Update values every two seconds
    updatePumpFlowRate();
    timeRunner = millis();
    if (millis() - phaseOneInitTime > 20000) {
      if (pumpFlowRate >= 0.8
          && pumpFlowRate <= 1.8) {
        tearDown();
        return RESULT_OK;
      } else {
        tearDown();
        return RESULT_FAIL;
      }
    } else {
      lcd.setCursor(0,3);
      float timeLeft = ( (float) phaseOneInitTime
          + 20000 - (float)millis() ) / (float) 1000;
      // timeLeft(timeLeft);
      char strBuffer[21];
      char tempBuffer[6];
      dtostrf(timeLeft, 4, 1, tempBuffer);
      sprintf(strBuffer,"Countdown: %ss", tempBuffer);
      lcd.print(strBuffer);
    }
  }
  return RESULT_WAIT;
}

void PhaseOne::tearDown() {
  pumpDisable();
}

PhaseOne phaseOne(&primingValveOpen, &mistingValveClose, &pumpEnable,
    updatePumpFlowRate, &pumpDisable);

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
  // void (*timeLeft)();
public:
  PhaseTwo(void (*mistingValveClose)(), void (*primingValveClose)(),
    void (*pumpEnable)(), void (*updatePumpFlowRate)(), void (*pumpDisable)(),
    bool (*readPressureSwitch)())
  {
    this->mistingValveClose = mistingValveClose;
    this->primingValveClose = primingValveClose;
    this->pumpEnable = pumpEnable;
    this->pumpDisable = pumpDisable;
    this->updatePumpFlowRate = updatePumpFlowRate;
    this->readPressureSwitch = readPressureSwitch;
    // this->timeLeft = timeLeft;
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

  totalPumpPulseCount = 0;
  mistingValveClose();
  primingValveClose();
  delay(4000);
  pumpEnable();
  timeRunner = millis();
  phaseTwoInitTime = millis();
  updatePumpFlowRate();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Phase 2: Accumulate");
  lcd.setCursor(0,1);
  lcd.print("Water Ballon coming");
  lcd.setCursor(0,2);
  lcd.print("right up!");

  phaseTwoSetupComplete = true;
}

int PhaseTwo::check() {
  if (millis() - timeRunner > 2000) {
    updatePumpFlowRate();
    timeRunner = millis();
    if (millis() - phaseTwoInitTime > 60000) {
      if (totalPumpVolume >= 0.01 && totalPumpVolume <= 3.0
          && readPressureSwitch() /* TODO: re-add this && readPumpPressureSwitch() */) {
        tearDown();
        return RESULT_OK;
      } else {
        tearDown();
        return RESULT_FAIL;
      }
    } else {
      lcd.setCursor(0,3);
      float timeLeft = ( (float) phaseTwoInitTime
          + 60000 - (float)millis() ) / (float) 1000;
      char strBuffer[21];
      char tempBuffer[6];
      dtostrf(timeLeft, 4, 1, tempBuffer);
      sprintf(strBuffer,"Countdown: %ss", tempBuffer);
      lcd.print(strBuffer);
    }
  }
  return RESULT_WAIT;
}

void PhaseTwo::tearDown() {
  pumpDisable();
}

PhaseTwo phaseTwo(&mistingValveClose, &primingValveClose, &pumpEnable,
    updatePumpFlowRate, &pumpDisable, &readPressureSwitch);

/**
 * Class that stores all phase one related variables and functions.
 */
class PhaseThree
{
  boolean phaseThreeSetupComplete = false;
  long timeRunner;
  long phaseThreeInitTime;
  int errorCount;

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
public:
  PhaseThree(void (*mistingValveOpen)(), void (*mistingValveClose)(),
    void (*primingValveClose)(), void (*pumpEnable)(),
    void (*updatePumpFlowRate)(), void (*pumpDisable)(),
    void (*updateMistFlowRate)())
  {
    this->mistingValveOpen = mistingValveOpen;
    this->mistingValveClose = mistingValveClose;
    this->primingValveClose = primingValveClose;
    this->pumpEnable = pumpEnable;
    this->pumpDisable = pumpDisable;
    this->updatePumpFlowRate = updatePumpFlowRate;
    this->updateMistFlowRate = updateMistFlowRate;
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

  primingValveClose();
  mistingValveOpen();
  delay(4000);
  pumpEnable();
  timeRunner = millis();
  phaseThreeInitTime = millis();
  updateMistFlowRate();

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Phase 3: Mist");
  lcd.setCursor(0,1);
  lcd.print("Let's get some");
  lcd.setCursor(0,2);
  lcd.print("people wet!");

  phaseThreeSetupComplete = true;
}

int PhaseThree::check() {
  if (millis() - timeRunner > 2000) {
    updatePumpFlowRate();
    updateMistFlowRate();
    timeRunner = millis();
    if (millis() - phaseThreeInitTime > 40000) {
      // Continuously check error cases.
      bool mistFlowGood = mistFlowRate >= 4.5 && pumpVolumeRate <= 6.5;
      bool pumpPressureSwitch = readPumpPressureSwitch();
      lcd.setCursor(0,2);
      lcd.print(pumpPressureSwitch);
      if (pumpPressureSwitch) {
        primingValveOpen();
      }
      bool pumpFlowGood = !pumpPressureSwitch
          || (pumpPressureSwitch && pumpFlowRate >= 0.6 && pumpFlowRate <= 1.8);
      if (mistFlowGood && pumpFlowGood) {
        errorCount = 0;
        lcd.setCursor(0,3);
        lcd.print("Getting My Mist On");
      } else {
        if (errorCount > 5) {
          tearDown();
          return RESULT_FAIL;
        } else {
          errorCount++;
        }
      }
    } else {
      float timeLeft = ( (float) phaseThreeInitTime
          + 40000 - (float)millis() ) / (float) 1000;
      char strBuffer[21];
      char tempBuffer[6];
      dtostrf(timeLeft, 4, 1, tempBuffer);
      sprintf(strBuffer,"Countdown: %ss", tempBuffer);
      lcd.setCursor(0,3);
      lcd.print(strBuffer);
    }
  }
  return RESULT_WAIT;
}

void PhaseThree::tearDown() {
  mistingValveClose();
  primingValveClose();
  pumpDisable();
}


PhaseThree phaseThree(&mistingValveOpen, &mistingValveClose, &primingValveClose,
    &pumpEnable, updatePumpFlowRate, &pumpDisable, &updateMistFlowRate);


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

  // Run preconditionsCheck to make sure system is ready
  preconditionsCheck();
}

void loop() {
  if (currentPhase == one) {
    phaseOne.setup();
    int check = phaseOne.check();
    if (check == RESULT_OK) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Mister Mister Primed");
      lcd.setCursor(0,1);
      lcd.print(pumpFlowRate);
      // Update currentPhase and move onto phase 2.
      delay(2000);
      currentPhase = two;
    } else if (check == RESULT_FAIL) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Bad Flow Rate - HALT!");
      lcd.setCursor(0,1);
      lcd.print(pumpFlowRate);
      currentPhase = halt;
      delay(1000);
    }
  } else if (currentPhase == two) {
    phaseTwo.setup();
    int check = phaseTwo.check();
    bool ps = readPressureSwitch();
    bool pps = readPumpPressureSwitch();
    if (check == RESULT_OK) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Mister Mister Loaded");
      lcd.setCursor(0,1);
      lcd.print(totalPumpVolume);
      lcd.setCursor(0,2);
      lcd.print(ps);
      lcd.setCursor(0,3);
      lcd.print(pps);
      // Update currentPhase and move onto phase 2.
      delay(2000);
      currentPhase = three;
    } else if (check == RESULT_FAIL) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("No water - HALT!");
      lcd.setCursor(0,1);
      lcd.print(totalPumpVolume);
      lcd.setCursor(0,2);
      lcd.print(ps);
      lcd.setCursor(0,3);
      lcd.print(pps);
      delay(1000);
      currentPhase = halt;
    }
  } else if (currentPhase == three) {
    phaseThree.setup();
    int check = phaseThree.check();
    if (check == RESULT_WAIT) {
      // Misting is going well.
    } else if (check == RESULT_FAIL) {
      // Something messed up.
      lcd.setCursor(0,3);
      lcd.print("HALT!");
      delay(1000);
      currentPhase = halt;
    }
  } else if (currentPhase == halt) {
      // DO Nothing.
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
 * Updates the Pump Flow Rate (pulses / second) and volume.
 */
void updateMistFlowRate() {
  long count = mistPulseCount - lastMistPulseCount;
  float timePassed = (millis() - mistFlowLastTimeChecked) / 1000;

  lastMistPulseCount = mistPulseCount;
  mistFlowLastTimeChecked = millis();

  mistFlowRate = ((float) count / (float) timePassed / (float) MIST_FLOW_PULSE) * 60 * 60;
  // pumpVolumeRate = pumpPulseCount / (float) PUMP_FLOW_PULSE;
  totalMistVolume = totalMistPulseCount / (float) MIST_FLOW_PULSE;

  char tempBuffer[6];
  char strBuffer[21];
  lcd.clear();
  lcd.setCursor(0,0);
  dtostrf(mistFlowRate, 4, 2, tempBuffer);
  sprintf(strBuffer,"Flow: %sG/h", tempBuffer);
  lcd.print(strBuffer);
  lcd.setCursor(0,1);
  dtostrf(totalMistVolume, 4, 2, tempBuffer);
  sprintf(strBuffer,"Volume: %sG", tempBuffer);
  lcd.print(strBuffer);
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

  char tempBuffer[6];
  char strBuffer[21];
  lcd.clear();
  lcd.setCursor(0,0);
  dtostrf(pumpFlowRate, 4, 2, tempBuffer);
  sprintf(strBuffer,"Flow: %sG/m", tempBuffer);
  lcd.print(strBuffer);
  lcd.setCursor(0,1);
  dtostrf(totalPumpVolume, 4, 2, tempBuffer);
  sprintf(strBuffer,"Volume: %sG", tempBuffer);
  lcd.print(strBuffer);
}

void printTimeLeft(float timeLeft) {
  char strBuffer[21];
  char tempBuffer[6];
  dtostrf(timeLeft, 4, 1, tempBuffer);
  sprintf(strBuffer,"Countdown: %ss", tempBuffer);
  lcd.print(strBuffer);
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
  // if (mistPulseTimestamps.size() >= MAX_PULSES_STORED) {
  //   mistPulseTimestamps.pop();
  // }
  // mistPulseTimestamps.push(millis());
}

/*
 * Interrupt Service Routine for Pump Flow Sensor
 */
void pumpPulseCounterISR()
{
  // Increment the pump pulse counter
  pumpPulseCount++;
  totalPumpPulseCount++;
  // if (pumpPulseTimestamps.size() >= MAX_PULSES_STORED) {
  //   pumpPulseTimestamps.pop();
  // }
  // pumpPulseTimestamps.push(millis());
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
  return !digitalRead(MIST_MODE_PIN);
}

bool readPressureSwitch()
{
  return !digitalRead(PRESSURE_SWITCH_PIN);
}

bool readPumpPressureSwitch() {
  return digitalRead(PUMP_SWITCH_PIN);
}
