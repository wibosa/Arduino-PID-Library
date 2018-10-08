/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3
#define ledPin 13
//Define Variables we'll be connecting to
volatile double Setpoint, Input, Output;
int run; // algo calcs
//Specify the links and initial tuning parameters
double Kp=12, Ki=0.001, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/* 
Example Timer1 Interrupt
Flash LED every second
*/


int timer1_counter;

void setup()
{ 
  Serial.begin(38400);
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 512;
  run = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
   pinMode(ledPin, OUTPUT);

 
  

    // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 31250/2;            // compare match register 16MHz/256/20Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // 1 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}


ISR(TIMER1_COMPA_vect) { //change the 0 to 1 for timer1 and 2 for timer2

   //noInterrupts();           // disable all interrupts
  Input = analogRead(PIN_INPUT);
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
  run++;
  //interrupts();             // enable all interrupts
}




void loop()
{
  //Serial.print("double Setpoint, Input, Output; ");
  Serial.print(" Set ");
  Serial.print(Setpoint, DEC);
  Serial.print(" In ");
  Serial.print(Input, DEC);
  
  Serial.print(" Out: ");
  Serial.print(Output, DEC);
  Serial.print(" run: ");
  Serial.println(run, DEC);
  run=0;
  
  delay(999);
  // your program here...
}
