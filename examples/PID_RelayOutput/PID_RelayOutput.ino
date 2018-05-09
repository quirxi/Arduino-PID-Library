/********************************************************
   PID RelaypidOutput Example
   Same as basic example, except that this time, the output
   is going to a digital pin which (we presume) is controlling
   a relay.  the pid is designed to pidOutput an analog value,
   but the relay can only be On/Off.

   To connect them together we use "time proportioning
   control"  it's essentially a really slow version of PWM.
   first we decide on a window size (5000mS say.) we then
   set the pid to adjust its output between 0 and that window
   size.  lastly, we add some logic that translates the PID
   output into "Relay On Time" with the remainder of the
   window being "Relay Off Time"
 ********************************************************/

#include <PID_v1.h>


// set the pins for input, output and sample frequency here:
const unsigned short PID_INPUT_PIN = 0; // pin from which sensor input is read
const unsigned short PID_RELAY_PIN = 6; // pin that is used to control relay
const unsigned int PID_CYCLE = 5000;    // a fixed period of time that determines a full PID on/off cycle (ms)

// specify the PID constants here:
double cProportional = 2;               // the proportional constant
double cIntegral = 5;                   // the integral constant
double cDerivative = 1;                 // the derivative constant

double pidSetpoint;                     // the desired target setpoint
double pidInput;                        // the input to the PID controller as read from the sensor (=PID_INPUT_PIN)
double pidOutput;                       // specifies how long the relay is switched on within a fixed period of time (=PID_CYCLE)

// initialize the PID controller
PID myPID(&pidInput, &pidOutput, &pidSetpoint, cProportional, cIntegral, cDerivative, DIRECT);

unsigned long pidStart;                 // variable that marks the start of each PID_CYCLE

//////////////////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode(PID_RELAY_PIN, OUTPUT);       // set the relay pin to output
  digitalWrite(PID_RELAY_PIN, LOW);     // initialize relay pin to low
  pidSetpoint = 70;                    // set the desired target value
  myPID.SetSampleTime(PID_CYCLE);       // tell the PID controller how frequently we will read a sample and calculate output
  myPID.SetOutputLimits(0, PID_CYCLE);  // tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);             // turn the PID on
  pidStart = millis();                  // here our first pid cycle (=PID_CYCLE) starts
  pidInput = analogRead(PID_INPUT_PIN); // read sensor input
  myPID.Compute();                      // compute pidOutput for the first time
}

//////////////////////////////////////////////////////////////////////////////////
// turns relay on/off
// ATTENTION: it is assumed that pulling your PID_RELAY_PIN to
//            high is switching the relay on (=normally open relays).
//////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // compute pidOutput each PID_CYCLE
  if (millis() - pidStart >= PID_CYCLE)
  {
    pidStart = millis();                    // reset pidStart to begin of new PID_CYCLE
    pidInput = analogRead(PID_INPUT_PIN);   // read sensor input
    myPID.Compute();                        // compute pidOutput
    if (pidOutput > 0)
    {
      digitalWrite(PID_RELAY_PIN, HIGH);
      if (pidOutput < PID_CYCLE)            // only switch off relay when necessary, otherwise leave it on till next PID_CYCLE starts
      {
        delay(pidOutput);
        digitalWrite(PID_RELAY_PIN, LOW);
      }
    }
    else digitalWrite(PID_RELAY_PIN, LOW);  // set relay pin low when pidOutput = 0
  }
}

