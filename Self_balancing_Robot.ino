#include "Wire.h"
#include "math.h"

#include "PID.h"
#include "ADXL345.h"


/*------define Left Motor Pins ----------*/

#define LeftMotorPWMPin    3
#define LeftMotorDirPinA   9 
#define LeftMotorDirPinB   2

/*------define Right Motor Pins ----------*/

#define RightMotorPWMPin   6
#define RightMotorDirPinA  7
#define RightMotorDirPinB  5

/*------ define Gyroscope Integration Periode Value ----------*/

#define SampleTime  0.005
/*------ define Robot Target  Angle Value ----------*/
double  TargetAngle  =-2.5 ;
 
/*------ define PID Controller Parametres ----------*/
#define Kp  8.8
#define Kd   0.78 // 2.78
#define Ki  4.5   

int ADXL345 = 0x53 ;
float X_out, Y_out, Z_out;  


volatile double MotorPower ;
volatile double  CurrentAngle ;

PID Pid(& CurrentAngle, & MotorPower, & TargetAngle , ( double ) Kp, ( double ) Ki, ( double ) Kd, DIRECT);
/*----------define functions prototypes -------------------*/
void InitSamplingTimer();

void SetMotors(int LeftMotorSpeed, int RightMotorSpeed );


void setup() {

  
  // set the motor control and PWM pins to output mode
  pinMode(LeftMotorPWMPin, OUTPUT);
  pinMode(LeftMotorDirPinA, OUTPUT);
  pinMode(LeftMotorDirPinB, OUTPUT);
  
  pinMode(RightMotorPWMPin, OUTPUT);
  pinMode(RightMotorDirPinA, OUTPUT);
  pinMode(RightMotorDirPinB, OUTPUT);
  
  // set  PID 
    Pid.SetMode( AUTOMATIC) ;
    Pid.SetSampleTime( 5 );
    Pid.SetOutputLimits(-255.0, 255.0);

    InitSamplingTimer();
    
    pinMode(10 ,OUTPUT);
    pinMode(11 ,OUTPUT);

    digitalWrite(11 , HIGH );
    digitalWrite(10 , LOW );
  /*-----------Initialize Accelerometer I2C  Communication---------  */
  Wire.begin(); 
  // Set ADXL345 in measuring mode
  Wire.beginTransmission(ADXL345); 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable 
  Wire.endTransmission();
  delay(10);

  Serial.begin(9600);
}

void loop() {
  
 
 
/*-----Plot The robot current Angle ----------- */
Serial.println(CurrentAngle) ;


/*--------- Read acceleromter data ------------*/
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  X_out = ( Wire.read()| Wire.read() << 8); // X-axis value
  X_out = X_out/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  Y_out = ( Wire.read()| Wire.read() << 8); // Y-axis value
  Y_out = Y_out/256;
  Z_out = ( Wire.read()| Wire.read() << 8); // Z-axis value
  Z_out = Z_out/256;

/*------------- set motor power after ----------------*/
  MotorPower = constrain( (int ) MotorPower , -255 , 255);
  SetMotors( (int) MotorPower , (int) MotorPower );

  
}


// The ISR : Interrupt Service Routine  :  will be called every 5 milliseconds : 
ISR( TIMER1_COMPA_vect )
 {

    CurrentAngle =  atan2(-1 * X_out ,sqrt(pow(Y_out, 2) + pow(Z_out, 2))) * 180 / PI;
   Pid.Compute();

 }
 
void SetMotors(int LeftMotorSpeed, int RightMotorSpeed ) 
{
  if( LeftMotorSpeed >= 0 )
  {
    analogWrite (  LeftMotorPWMPin   , LeftMotorSpeed );
    digitalWrite( LeftMotorDirPinA , LOW  );
    digitalWrite( LeftMotorDirPinB, HIGH );
  }
  else 
  {
    analogWrite ( LeftMotorPWMPin, -LeftMotorSpeed );
    digitalWrite( LeftMotorDirPinA , HIGH  );
    digitalWrite( LeftMotorDirPinB , LOW   );    
  }


  if( RightMotorSpeed >= 0 )
  {
    analogWrite(  RightMotorPWMPin   , RightMotorSpeed );
    digitalWrite( RightMotorDirPinA , LOW  );
    digitalWrite( RightMotorDirPinB , HIGH );
  }
  else 
  {
    analogWrite ( RightMotorPWMPin, -RightMotorSpeed );
    digitalWrite( RightMotorDirPinA , HIGH  );
    digitalWrite( RightMotorDirPinB , LOW   );    
  }

  

}


void InitSamplingTimer() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}
