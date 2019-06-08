/* Fish control in the main tube
 *  Complete on 23/05/2018 Version 3.6


Changelogs: 

Fixed encoder angle, because the angle is relative to the starting point, we need to compensate for the 0 error.
Fixed turning code to be relative to the duty cycle as opposed to a fix turning ratio. 
This is the code with the HEAVE functionality as well, also known as the heave function. 

Remember to set the encoder angle everytime we change the mechanism. The code assumes that the encoder angle is 
0 at the neutral position
the motor rotates anti-clockwise.

 Currently Using Serial3. 
 Serial would be used for debugging purposes. 

 Turning Code Disabled to Test the HEAVE CODE. 

 Next step, ask arduino to read paramters from a text file!

 

 Changelog 19/3/2019
 Activated the turning function, and checked the entire gait control code function of the fish. 
 Prior unrecorded updates, the pin numbers have been changed to suit the new controller pins. 
 New function addded: Able to control items variables on the fly before testing. We now have two operating modes, pre-test mode and testing mode. Soon we'll add a results page. 

 Future Work: 

 1) Add initialization values on the tail part
 2) Fix the screen switching code
 3) Add Current Controller


New Additions for the turnign code: added a filter to rid of garbage values for the turning code. 

Adding Ultransound range finder to the equation. 

 */
#include <SD.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>

// Function declaration
void sdCard();
float currentSensor();
float gaitControl(byte x);
float fishTurning(float y);

// Features 

// Debugging Features 
#define debug 0


//SD Card variables
int fileCounter = 1;
int sdState = 0;
File myFile;
String filename;
const int csPin = 53;
long long sdTimer;
const int samplingTime = 10000;  // Microseconds

//Sensor Variables
// Current Sensor Variables
float stepI = 0.12 ; // The ACS 723 has it at ~0.012, 10 times better. 
float zeroI = 512;
float current = 0;
const int     cur_sensorPin  = A0;

unsigned const int encoderInitial          = 427;

//const int enablePin = 4;                   //AMP enable pin
//volatile unsigned long encoder0Pos = 0;    //Count plus  

//Items for receiving Signals
char          inComingbyte[12];
char          sendData[12];
char          speedC[4];
char          dutyC[4];
char          minP[4];
char          heaveA[4];
char          receivedTurn;                                //received turning signal
bool          tubeState      = 1;

//Pin Definitions
const int     pwmPin         = 10; // Pin for PwmOutput to the motor
const int     inc_encoderPin = 2; // Incremental Encoder


//Configuration Variables
unsigned int  pwmLimit            = 0;                  //received dutycycle limit
unsigned int  receivedDutyCycle   = 0;                  //received velocity(dutycycle if pid is not used 
unsigned long rotation            = 0;                  //
unsigned long lastReceiveTime     = 0;                  //
unsigned int  pwmOutput           = 0;                  //

float         lastAngle      = 0.0;                //last motor shaft position
float         velocity       = 0.0;                //motor rotational velocity(rps)
float         turnDifferential = 0.2;               //5% lower on one side, and 5% higher on the other side.
unsigned int  deltaT         = 10;                 // time constant
unsigned long t              = 0;
unsigned long tMax           = 0;
unsigned int  relEncTime     = 0;

volatile unsigned int  encoderValueRaw        = 0;                  //decimal reading from absolute encoder
volatile float         angle          = 0.0;                //current motor shaft position
// Gait Control Variables

const float maxStallPwm = 0.15;      // This is as a percentage of the assigned duty cycle
int minPwm = 0;               // This is the bare minimum pwm signal to get the motor moving under the weight of water and tail PWM range [0,255]
float contPwmOutput = 0;
int heaveAngle = 0; // This is the compensate amount where the minimum PWM will be in effect. Having it as 30 means that the low PWM range is 60 degrees. Should be less than 45 degrees. 
int minPwmLimit = 38;  
int heaveAngleLimit = 40;  
unsigned int receivedminPwm;
unsigned int receivedheaveAngle;


void setup() {
  TCCR2B = TCCR2B & 0b11111000 | 0x01;
  TCCR1B = TCCR1B & 0b11111000 | 0x01; // Sets pin 3,9,10,11 to 31500kHz
  Serial.begin(115200);
  Serial3.begin(19200,SERIAL_8O1); // Sets the Serial to 8bit, odd parity with one end stop
  SD.begin(csPin);                // For the SD Card
  for(int encoderPin = 29;encoderPin <= 47;encoderPin = encoderPin + 2 ){   //absolute encoder pin setup
  pinMode(encoderPin,INPUT_PULLUP) ; 
  }
  pinMode(pwmPin, OUTPUT);                  //motor control pwm signal pin

  zeroI = analogRead(cur_sensorPin);
  delay(1);
 

  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
  
  // For the SD card
  String filename = "TestRun";
  filename += fileCounter;
  filename += ".txt";
  //Creates the file object and opens it to be written too. Assume the SD card will be empty. 
  myFile = SD.open(filename, FILE_WRITE);
  myFile.println("DutyCycle\tTime\tAngle\tCurrent");
    
  while(!Serial3.available()) ;        //wait until the first incoming signal 
    

}

void loop(){

  t= micros();
  updateData();
  current = currentSensor();
  absoluteEncoder();
  //Serial.println(angle);
  //Serial.println(contPwmOutput);

  pwmOutput = map(receivedDutyCycle, 0, 99, 0, 255);  // very important to do. Probably want to get rid of it.
  heaveAngle = receivedheaveAngle;
  minPwm = receivedminPwm; 
  contPwmOutput = gaitControl(pwmOutput);   // contPwmOutput stands for the controlled PWM OUTPUT, that is after it has undergone, feedback, gait,or any turning requirements.
  
  analogWrite(pwmPin,contPwmOutput );
  

  if(myFile) {
    sdCard();
  }
  
  //speedFromAbsoluteEncoder();
  deltat = (micros()-t);

  #if debug 
  if (deltat > tMax) tMax = deltat;
  //Serial.print(deltat);
  //Serial.print("\t");
  //Serial.println(angle);
  //Serial.print("\t");
  //Serial.println(tMax);
  //Serial.println(tMax);
  //Serial.print(contPwmOutput);

  //Serial.print("\t");
  Serial.print(encoderValueRaw);
  Serial.print("\t");
  Serial.print(current);
  Serial.print("\t");
  Serial.print(angle);
  Serial.print("\t");
  Serial.print(contPwmOutput);
  Serial.print("\t");
  Serial.print(receivedTurn);
  

  Serial.print("\n");
  //delay(50);
  #endif
  
}
/*
int updatePid(int outputValue, float targetValue, float currentValue,int timeConstant){
  float pidTerm = 0;
  float integral = 0;
  float derivative = 0;
  int error = 0;
  static int lastError = 0;
  error = targetValue - currentValue;
  derivative = (error - lastError)*1000/timeConstant;
  integral = integral + error*timeConstant/1000;
  pidTerm = (Kp*error) + (Ki*integral) + (Kd*derivative);
  lastError = error;
  return constrain(outputValue + int(pidTerm), 0, 255);
}
*/
void updateData(){                  //data update from controller
  if(tubeState == 1){
    if(Serial3.available()>=11){
      Serial3.readBytesUntil('e',inComingbyte,11);
      if(inComingbyte[0] == 'c'){
        tubeState = 0;
        for(int i = 0;i<2;i++){
        dutyC[i]  = inComingbyte[i+1];
        speedC[i] = inComingbyte[i+3];
        minP[i] = inComingbyte[i+5];
        heaveA[i]  = inComingbyte[i+7];
        }
        pwmLimit     = atoi(dutyC);
        receivedDutyCycle = atoi(speedC);
        receivedminPwm = atoi(minP);
        receivedheaveAngle = atoi(heaveA);


     //to protect the turn signal from noise
     if (inComingbyte[9] == 'r' || inComingbyte[9] == 'l' || inComingbyte[9] == '0')
        receivedTurn        = inComingbyte[9];
        
        if(receivedDutyCycle > pwmLimit){
          receivedDutyCycle = pwmLimit;
        }
        if(receivedminPwm > minPwmLimit){
          receivedminPwm = minPwmLimit;
        }
        if(receivedheaveAngle > heaveAngleLimit){
          receivedheaveAngle = heaveAngleLimit;
        }
        lastReceiveTime = millis();
      }
     //while(Serial3.available()>6) Serial3.read();
    }
    

    if(millis() - lastReceiveTime > 3000){
      receivedDutyCycle = 0;
      pwmOutput = 0;
      receivedDutyCycle = 0;
      
    }
  }
  else if(tubeState == 0){
    sprintf(sendData,"f%02u%02u%02u%02u%ce",pwmLimit,receivedDutyCycle,receivedminPwm,receivedheaveAngle,receivedTurn);
    Serial3.print(sendData);
    tubeState = 1;
  }
}

void absoluteEncoder(){               //get angle from absolute encoder
  volatile int a[10];
  volatile int b[10];
  for(int n = 0;n<10;n++){
    a[n] = !digitalRead(2*n+29);
  }
  b[9] = a[9];
  for(int i = 1;i<10;i++){
    b[9-i] = b[9-i+1]^a[9-i];
  }
  encoderValueRaw = 512.0*b[9]+256.0*b[8]+128.0*b[7]+64.0*b[6]+32.0*b[5]+16.0*b[4]+8.0*b[3]+4.0*b[2]+2.0*b[1]+b[0];
  //Serial.println(encoderValueRaw);

  // here's the biggest fix, it's the return angle of the encoder wasn't right before.
  if(encoderValueRaw >= encoderInitial) angle  = (encoderValueRaw-encoderInitial)*360.0/1024.0;
  else angle = (encoderValueRaw+1023-encoderInitial)*360.0/1024.0;
}

void speedFromAbsoluteEncoder(){       //calculate velocity from absolute encoder
   float angle_dif = angle - lastAngle;
   if(angle_dif > 300){          //motor rotate clockwise
    angle_dif = angle_dif - 360;
    rotation++;
   }
   else if(angle_dif < -300){    //motor rotate countclockwise
    angle_dif = angle_dif +360;
    rotation++;
   }
   velocity = abs(angle_dif)*1000.0/relEncTime/360.0;
   //Serial.println(velocity);
   lastAngle = angle;
}

float fishTurning(float y){
// Changes the PWM Output for turning. 
/* The turning function simply tells the fish to turn when it receives the signal, and it stops turning
 *  if it does not receive any signal. States from the controller are r,'0',l. 
 * From the flow, the aerodynamics dictate that the fish turns right. 
Another way to see it is the orientation of the head. For power strokes, 
if you paddle on the left of a boat, you will turn right. 

Assumes 0 is eq. point, tail starts going right. 
(i) Right Drag stroke (0-90) = RIGHT DRAG
(ii) Right Power Stroke (90-180) = LEFT POWER
(iii)Left Drag stroke (180-270) = LEFT DRAG
(iv) Left Power Stroke (270-360) = RIGHT POWER
 */
 float pwmHigh = y*(1 + turnDifferential);
 float pwmLow = y*(1 - turnDifferential);
 if(pwmLow <0) pwmLow = 0;
 float pwmOutput = 0;
//This is to ensure that the PWM value's are a constant for each turn, the issue is caused because this function exists in a loop.
 
    if(receivedTurn == 'r'){
      //Power up third and fourth quadrants. 
      //Drag Stroke
      if(angle > 90 && angle <= 270){
        pwmOutput = pwmLow;
        return pwmOutput;
        //pwmOutput = constrain(pwmOutput,0, (receivedDutyCycle + turnDifferential));
      }  
      else{
        pwmOutput = pwmHigh;
        return pwmOutput;
        //pwmOutput = constrain(pwmOutput,(receivedDutyCycle - turnDifferential),99);
      }

    }
    else if(receivedTurn == 'l'){
      if(angle > 90 && angle <= 270){
        pwmOutput = pwmHigh; 
        return pwmOutput;
        //pwmOutput = constrain(pwmOutput,(receivedDutyCycle - turnDifferential),99);  
      }
      else{
        pwmOutput = pwmLow;
        return pwmOutput;
        //pwmOutput = constrain(pwmOutput,(receivedDutyCycle - turnDifferential),99);
      }
    }
    else{
      pwmOutput = y;
      return pwmOutput;
    }
    
}


void sdCard(){
    // Waiting State
  if(sdState == 0){
    if(pwmOutput > 0){
      // Sets it to running state
      sdState = 1;
      // Initializes the timer
      sdTimer = micros();
    }     
  }
  
  // Running State 
  else if(sdState ==  1){
    //Serial.print("running");
    //Sets the sampling rate to be 10ms or 100Hz
    if (micros()- sdTimer > samplingTime){
      myFile.print(pwmOutput);
      myFile.print("\t");
      myFile.print(micros());
      myFile.print("\t");
      myFile.print(angle);
      myFile.print("\t");
      myFile.println(current);
     
     // myFile.flush();  // ?? Might not be necessary
      // Reset timer
      sdTimer = micros(); 
      //Serial.println("writing"); 
    }
    // Checks if the fish has stopped
    if(pwmOutput == 0){
      // Saves all the data and closes the file
      myFile.close();
      // Opens a new file with a different name
      fileCounter++;
      String filename = "TestRun";
      filename += fileCounter;
      filename += ".txt";
      myFile = SD.open(filename,FILE_WRITE);
      myFile.println("Time\tAngle\tCurrent");
      // Set the SD into waiting mode
      sdState = 0;
      
    }
  }
}

float avgCurrent = 0;
int counterA = 0;
float curA[5] = {0};
float currentSensor(){

  int x = analogRead(A0);
  delay(1);
  float current = (x-zeroI)*stepI;

  //moving average filter for the current
  curA[counterA] = current;
  counterA++;
  if (counterA > 5) counterA = 0; // reset the counter and replace the old data with new ones
  
  for (int i = 0; i < 5; i++)
  {
    //sum of the array current
    avgCurrent += curA[i];
    }
  avgCurrent = avgCurrent/5; 

  return avgCurrent; //  filter mode
}

// Interrupt Service Routine( for the Sensors)
SIGNAL(TIMER0_COMPA_vect) 
{
    //absoluteEncoder();
    //Serial.println(angle);
}

float gaitControl(byte x){
 
  // This code provides a sinosuidal output so that we can control the gait of the fish
  // Rather than having a huge jump, we hope to let the motor ease into it's stall point
  // Appends the pwmOutput of the 
  
  float pwmOutput = x*( (1-maxStallPwm)*abs(cos(angle*PI/180))+ maxStallPwm);

  pwmOutput = fishTurning(pwmOutput);
  if(pwmOutput == 0) 
  {
    pwmOutput = 0;
    return pwmOutput;
  }
  else if ((angle > 90-heaveAngle && angle <90+heaveAngle) || (angle >270-heaveAngle && angle < 270+heaveAngle)) pwmOutput = minPwm;
  else if(pwmOutput < minPwm) pwmOutput = minPwm;
  return pwmOutput;
}
