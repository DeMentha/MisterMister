// Battery related constants
const float BAT_SCALING = 1.05 * 5 * ((5.1+15)/5.1) / 1024;
const float BAT_LIMIT_HIGH = 25; // Volts
const float BAT_LIMIT_LOW = 8;  // Volts

// Flow Sensor constants
const float PUMP_FLOW_PULSE = 2255.86; // Pulses per Gallon
const float MIST_FLOW_PULSE = 5223.3; // Pulses per Gallon
