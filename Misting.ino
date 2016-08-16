// vi:ts=4
// ----------------------------------------------------------------------------
// HelloWorld - simple demonstration of lcd
// Created by Bill Perry 2016-07-02
// bperrybap@opensource.billsworld.billandterrie.com
//
// This example code is unlicensed and is released into the public domain
// ----------------------------------------------------------------------------
// 
// Sketch will print "Hello, World!" on top row of lcd
// and will print the amount of time since the Arduino has been reset
// on the second row.
//
// If initialization of the LCD fails and the arduino supports a built in LED,
// the sketch will simply blink the built in LED.

#include <hd44780_I2Cexp.h>

hd44780_I2Cexp lcd; // declare lcd object: auto locate & config exapander chip

// LCD geometry
const int LCD_ROWS = 4;
const int LCD_COLS = 20;

// Digital Pin Allocations
byte BAT_PIN = 0;
byte PRIME_VALVE_PIN = 0;     // D0
byte MIST_VALVE_PIN = 1;      // D1
byte PUMP_FLOW_PIN = 2;       // D2  must be 2 or 3
byte MIST_FLOW_PIN = 3;       // D3  must be 2 or 3
byte PRESSURE_SWITCH_PIN = 4; // D4
byte MIST_MODE_PIN = 5;       // D5
byte PUMP_RELAY_PIN = 6;      // D6
byte SPARE_RELAY_PIN = 7;     // D7
byte PUMP_SWITCH_PIN = 8;     // D8


// Analog Pin Allocations
byte I2C_SCL_PIN = 5;         // A5
byte I2C_SDA_PIN = 4;         // A4
byte BAT_VOLTAGE_PIN = 3;     // A3

byte PUMP_FLOW_INTERRUPT = 0; // D2
byte MIST_FLOW_INTERRUPT = 1; // D3

const float BAT_SCALING = 1.05 * 5 * ((5.1+15)/5.1) / 1024; 
const float BAT_LIMIT_HIGH = 15; // Volts
const float BAT_LIMIT_LOW = 11;  // Volts

byte statusLed = 13;

byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin       = 2;

// The hall-effect flow sensor outputs approximately 4.5 pulses per second per
// litre/minute of flow.
float calibrationFactor = 4.5;

volatile byte mistPulseCount;  
volatile byte pumpPulseCount;  

float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

unsigned long oldTime;


void setup()
{

  pinMode(PRIME_VALVE_PIN, OUTPUT);
  primingValveOpen();
  pinMode(MIST_VALVE_PIN, OUTPUT);
  mistingValveClose();
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pumpDisable();
  pinMode(SPARE_RELAY_PIN, OUTPUT);
  digitalWrite(SPARE_RELAY_PIN,HIGH);

  pinMode(PUMP_FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PUMP_FLOW_PIN), pumpPulseCounterISR, RISING);

  pinMode(MIST_FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MIST_FLOW_PIN), mistPulseCounterISR, RISING);
  
	// initialize LCD with number of columns and rows: 
	if( lcd.begin(LCD_COLS, LCD_ROWS))
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

   // Initialize a serial connection for reporting values to the host
  //Serial.begin(38400);
   
  // Set up the status LED line as an output
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, HIGH);  // We have an active-low LED attached
  

  mistPulseCount    = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;
	
	// Print a message to the LCD
  lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("DEMENTHA MISTING 2.0");
  delay(5000);
    
  while(1) 
  {
    initialize();
    prime();
  }

}

void loop()
{
}

// PrintUpTime(outdev, secs) - print uptime in HH:MM:SS format
// outdev - the device to send output
//   secs - the total number of seconds uptime
void PrintUpTime(Print &outdev, unsigned long secs)
{
uint8_t hr, mins, sec;

	// convert total seconds to hours, mins, seconds
	mins =  secs / 60;	// how many total minutes
	hr = mins / 60;		// how many total hours
	mins = mins % 60;	// how many minutes within the hour
	sec = secs % 60;	// how many seconds within the minute
		

	// print uptime in HH:MM:SS format
	// Print class does not support fixed width formatting
	// so insert a zero if number smaller than 10
	if(hr < 10)
		outdev.write('0');
	outdev.print((int)hr);
	outdev.write(':');
	if(mins < 10)
		outdev.write('0');
	outdev.print((int)mins);
	outdev.write(':');
	if(sec < 10)
		outdev.write('0');
	outdev.print((int)sec);
}

void initialize()
{
  lcd.clear();  
  checkBatteryVoltage();
  delay(5000);

  // Open priming valve
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("OPENING PRIMING");
  lcd.setCursor(0,1);
  lcd.print("VALVE...");
  primingValveOpen();
  delay(5000);

  // Close misting valve
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CLOSING MISTING");
  lcd.setCursor(0,1);
  lcd.print("VALVE...");
  mistingValveClose();
  delay(5000);

  // Check motor temperature
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CHECKING PUMP");
  lcd.setCursor(0,1);
  lcd.print("TEMPERATURE...");
  delay(5000);

  // Check misting flow sensor, flow should be very close to zero
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CHECKING MIST");
  lcd.setCursor(0,1);
  lcd.print("FLOW SENSOR...");
  delay(5000);

  // Check pump flow sensor, flow should be very close to zero
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CHECKING PUMP FLOW");
  lcd.setCursor(0,1);
  lcd.print("SENSOR...");
  delay(5000);

  // Check pressure switch, should be low
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("CHECKING PRESSURE");
  lcd.setCursor(0,1);
  lcd.print("SWITCH...");
  delay(5000);

  // Check pump switch, should be off
}

void prime()
{
static unsigned long lastsecs = -1; // pre-initialize with non zero value
unsigned long secs;
unsigned long primeDelay = 30;

lcd.clear();

lcd.setCursor(0, 3);


  for (unsigned long primeDelay = 30;primeDelay>0;primeDelay--)
  {

      char buffer[3];
      if (primeDelay < 10) sprintf(buffer, "PRIME IN %d SECONDS ", primeDelay);
      else sprintf(buffer, "PRIME IN %d SECONDS", primeDelay);

      lcd.setCursor(0, 3);

      // print uptime on lcd device: (time since last reset)
      
      lcd.print(buffer);
      delay(1000);
    
  }
  lcd.clear();
}

void mist()
{
  
}

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

bool readPressureSwitch()
{
  return digitalRead(PRESSURE_SWITCH_PIN);
}

bool readPumpSwitch()
{
  return digitalRead(PUMP_SWITCH_PIN);
}

float readBatteryVoltage()
{
  return analogRead(BAT_VOLTAGE_PIN)*BAT_SCALING;
}

void checkBatteryVoltage()
{
  
  char strBuffer[21];
  char tempBuffer[6];
  
  float temp = readBatteryVoltage();

  
  // Check battery voltage
  dtostrf(readBatteryVoltage(), 4, 2, tempBuffer);
  sprintf(strBuffer,"BATT VOLTAGE: %sV", tempBuffer);
      
  lcd.setCursor(0, 1);
  lcd.print(strBuffer);
  
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
  lcd.setCursor(0,2);
  lcd.print("BATTERY VOLTAGE OK.");
}

/*
Liquid flow rate sensor -DIYhacking.com Arvind Sanjeev

Measure the liquid/water flow rate using this code. 
Connect Vcc and Gnd of sensor to arduino, and the 
signal line to arduino digital pin 2.
 
 */


/**
 * Main program loop
 */
void test()
{
   
   if((millis() - oldTime) > 1000)    // Only process counters once per second
  { 
    // Disable the interrupt while calculating flow rate and sending the value to
    // the host
    detachInterrupt(PUMP_FLOW_INTERRUPT);
    detachInterrupt(MIST_FLOW_INTERRUPT);
        
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - oldTime)) * mistPulseCount) / calibrationFactor;
    
    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;
    
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
      
    unsigned int frac;
    
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));  // Print the integer part of the variable
    Serial.print(".");             // Print the decimal point
    // Determine the fractional part. The 10 multiplier gives us 1 decimal place.
    frac = (flowRate - int(flowRate)) * 10;
    Serial.print(frac, DEC) ;      // Print the fractional part of the variable
    Serial.print("L/min");
    // Print the number of litres flowed in this second
    Serial.print("  Current Liquid Flowing: ");             // Output separator
    Serial.print(flowMilliLitres);
    Serial.print("mL/Sec");

    // Print the cumulative total of litres flowed since starting
    Serial.print("  Output Liquid Quantity: ");             // Output separator
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 

    // Reset the pulse counter so we can start incrementing again
    mistPulseCount = 0;
    
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(digitalPinToInterrupt(PUMP_FLOW_PIN), pumpPulseCounterISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MIST_FLOW_PIN), mistPulseCounterISR, RISING);
  }
}

/*
Insterrupt Service Routines
 */
void mistPulseCounterISR()
{
  // Increment the pulse counter
  mistPulseCount++;
}

void pumpPulseCounterISR()
{
  // Increment the pulse counter
  pumpPulseCount++;
}




