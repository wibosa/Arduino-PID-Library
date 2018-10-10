/********************************************************
 * PID timed Interrupt Basic Example ( Mega 2560 < 3kHz )
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/
const int ledPin = LED_BUILTIN;  // the pin with a LED

#include <TimerThree.h>

volatile int ledState = LOW;
#include <PID_v1.h>
#define PIN_INPUT 0
#define PIN_OUTPUT 3
//Define Variables we'll be connecting to
//Specify the links and initial tuning parameters
volatile double Setpoint, Input, Output;
double Kp=12, Ki=0.001, Kd=0;
volatile long run; // algo calcs

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void PIDcompute(void)  /// ISR
{
  //noInterrupts();           // disable all interrupts
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  //ledState= ledState ^ 1;
  //digitalWrite(ledPin, ledState);
  run++;
  //interrupts();             // enable all interrupts
}
unsigned long time;
unsigned long starttime;
void setup()
{ 
  Serial.begin(115200);  
  pinMode(ledPin, OUTPUT);
  //Serial.print("Time: ");
  
  //Serial.println(time); 
  
  
 
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 512;
  Output = analogRead(PIN_OUTPUT);
  run = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  noInterrupts();           // disable all interrupts
  Timer3.initialize(3000);  // millisecs max FOR PID ISR about 3.3kHz better run 0.33kHz
  Timer3.attachInterrupt(PIDcompute);
  interrupts();             // enable all interrupts
}

void loop()
{ run=0;
  //interrupts(); 
  delay(995);   // display update time msecs
  //delayMicroseconds(10000);
  //noInterrupts(); 
  // Serial comm 115200 takes some msecs so must be interruptable for high bandwidth
  //Serial.print("double Setpoint, Input, Output; ");
  time = millis();
  Serial.print(time-starttime); 
  starttime=time;
  Serial.print(" Set ");
  Serial.print(Setpoint, DEC);
  Serial.print(" In ");
  Serial.print(Input, DEC);
  
  Serial.print(" Out: ");
  Serial.print(Output, DEC);
  Serial.print(" run: ");
  Serial.println(run, DEC);
  
  
}
