//Segway control code
//2013-05-19
//by: krsk


//Librarys used
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "pins.h"


//Constants---------------------------------------------------------------
#define BAUD_RATE (long)38400
#define ACC_FILTER_SAMPLES (int)50
#define GYRO_FILTER_SAMPLES (int)10
#define SIGNAL_STOP (float)187.5
#define GRAVIY (float)9.81
#define SIGNAL_MIN (float)128.0
#define SIGNAL_MAX (float)256.0
#define BALANCE_ANGLE (float) -0.23
#define DEBUG (boolean)true



//Variables---------------------------------------------------------------
float sensorAngle = 0.0;       //Angle estimated from accelerometer [radians]
float sensorAngleSpeed = 0.0;
float gyroAngleSpeed = 0.0; //Angle speed estimated from gyro [radians/s]
float prevAngle = 0.0;
float prevAngleSpeed = 0.0;
long goalLoopTime = 5000;  //[us]
long loopTime = 0;
long loopDelay = 0;
long timeStamp = 0;

long startTime = 0;
float integrator = 0.0;
boolean startReady = false;  //Wait until stable angle singal
boolean emergency = false;   //Emergency brake

MPU6050 sensor;
int16_t ax, ay, az;
int16_t gx, gy, gz;


int axAverage;  //Calculated average
int axFirstIn;  //First value in each average calculation
int ayAverage;  //Calculated average
int ayFirstIn;  //First value in each average calculation
int azAverage;  //Calculated average
int azFirstIn;  //First value in each average calculation

int gxAverage;  //Calculated average
int gxFirstIn;  //First value in each average calculation
int gyAverage;  //Calculated average
int gyFirstIn;  //First value in each average calculation
int gzAverage;  //Calculated average
int gzFirstIn;  //First value in each average calculation

//int axHistory[SAMPLE_BUFFER];
int ayHistory[ACC_FILTER_SAMPLES];
int azHistory[ACC_FILTER_SAMPLES];
int gxHistory[GYRO_FILTER_SAMPLES];
//int gyHistory[SAMPLE_BUFFER];
//int gzHistory[SAMPLE_BUFFER];


//Sample counter
int accSampleCC = 0;
int gyroSampleCC = 0;

/*
//Array for logging
float dataLog1[SAMPLE_BUFFER];
float dataLog2[SAMPLE_BUFFER];
float dataLog3[SAMPLE_BUFFER];
*/


void setup(){

  Serial.begin(BAUD_RATE);
  Serial.println("Controller started"); 
  Wire.begin();

  
  pinMode(DIODE_PIN, OUTPUT);
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
  pinMode(EMERGENCY_PIN,INPUT);
  
  
  //Stabilize motors
  analogWrite(MOTOR_PIN_1, SIGNAL_STOP);
  analogWrite(MOTOR_PIN_2, SIGNAL_STOP);
  digitalWrite(DIODE_PIN, LOW);
  
  //Init sensor
  Serial.println("Sensor init");
  sensor.initialize();
  Serial.println("Sensor init ok");
  Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  startTime = micros();
  timeStamp = micros();

}

void loop(){
  
  //Set loop time to fix value by compensating for code execution time
  loopDelay = goalLoopTime - micros() + timeStamp;
  if (loopDelay > 0){
    //delay(loopDelay/1000);
    digitalWrite(DIODE_PIN, LOW);
  }
  else{
    digitalWrite(DIODE_PIN, HIGH);
  }
  

  //loopDelay = max(25000 - (micros() - timeStamp),0);
  //delayMicroseconds(loopDelay);
  
  //Measure the actual time after compensation
  loopTime = micros() - timeStamp;
  timeStamp = micros();

  
  //Compute angle and control motors when ready
  sensorAngle = computeAngle();   //Read and process accelerometer data  

  sensorAngleSpeed = 1000*(sensorAngle - prevAngle)/loopTime;

  controlMotors(sensorAngle,sensorAngleSpeed);                      //Output steering signal
 
  
  prevAngle = sensorAngle;
  
  //Handle heart beat counters
  //accSampleControl();
  //gyroSampleControl();
  //emergencyControl();
  startReady = true;
}

//Generate and put out the controll signal for the motors, based on angleHistory array
void controlMotors(float angle, float angleSpeed){
  //Parameters (declare at top?)

  float k_p = 120.0;     //300 med Serial
  float k_d = 3000.0;    //9500 med Serial
  float k_s = -1.5;
  
  float deadbandF = 0.0;
  float deadbandB = 0.0;
  
  //Steering
  float steering = getSteering();
  steering = 0;  //Disable steering
  float steerAngle = -angle;
  float steerAngleSpeed = -angleSpeed;
  //float angleSpeedFiltered = getAverageAngle();
  //Steering signal, PD-regulator
  //float s = k_p*(angleC-balanceAngle) - k_d*angleSpeedFiltered/dt;  
  
  float e = steerAngle - BALANCE_ANGLE;
  
  float s = k_p*(e) + k_d*steerAngleSpeed;
  
  //Left wheel engine  
  float s_left = s + k_s*steering;
  
  //Right wheel engine
  float s_right = s - k_s*steering;
  
  //Deadbands compensation
  if (s>=0){
    s_left = s_left + deadbandF;
    s_right = s_right + deadbandF;
  }
  else{
    s_left = s_left - deadbandB;
    s_right = s_right - deadbandB;
  }
 
 
  float s_out1 = SIGNAL_STOP;
  float s_out2 = SIGNAL_STOP;
  
  if (startReady && !emergency){
    //s_out1 = min(max(SIGNAL_STOP+s_left,SIGNAL_MIN),SIGNAL_MAX);
    s_out1 = max(min(SIGNAL_STOP-s_left,SIGNAL_MAX),SIGNAL_MIN);
    s_out2 = max(min(SIGNAL_STOP-s_right,SIGNAL_MAX),SIGNAL_MIN);
  }
  
  
  analogWrite(MOTOR_PIN_1,s_out1);  
  analogWrite(MOTOR_PIN_2,s_out2);
  
  //sendValues(e,s_left,s_out1);
}

//Cakculate an angle from filtered gyro and accelerometer data
float computeAngle(){
     
  sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  
  //axHistory[sampleCC] = ax;
  //ayHistory[accSampleCC] = ay;
  //azHistory[accSampleCC] = az; 
  //gxHistory[gyroSampleCC] = gx;
  //gyHistory[sampleCC] = gy;
  //gzHistory[sampleCC] = gz;  
  
  //Average filter
//  float ayAverage = getAverage(ayHistory, ACC_FILTER_SAMPLES);
//  float azAverage = getAverage(azHistory, ACC_FILTER_SAMPLES);
//  float gxAverage = getAverage(gxHistory, GYRO_FILTER_SAMPLES);;

//  float aAngle = accelerationToAngle(ayAverage,azAverage);
//  float gAngle = gyroToAngle(gxAverage,prevAngle);  
    
    float aAngle = accelerationToAngle(ay,az);
    float gAngle = gyroToAngle(gx,prevAngle);  
  
  
  float gBalance = 0.99;    //Between 1 and 0 depending on gyro impac
  float angle = gBalance*gAngle + (1-gBalance)*aAngle;
  //sendValues(aAngle,gAngle,gx);
  return angle;
}

//Get steering output. -10 to 10 for whole span
float getSteering(){
  float sensorInput = analogRead(STEERING_PIN);
  //sendValues(sensorInput,0,0);
  float steeringBalancePoint = 274;
  float steeringValue = min(max((sensorInput-steeringBalancePoint)/7,-10),10);
  return steeringValue;
}

float gyroToAngle(float g, float previousAngle){
  float k = 20e-9;  //Correcion factor for time unit (us) and gyro units (??). Calibrated towards loop 500us loop time
  float gDiff = g+223.0;
  float timeDiff = loopTime/1000;

  float angle = previousAngle + k*gDiff*timeDiff*timeDiff;  //[rad/s]
  
  //sendValues(angle*180/PI,gDiff,timeDiff);
    
  if(startReady){
    return angle;
  }
  else{
    return 0;
  }
}

//Calculate angle from acceleration data. x forward acceleration, y upward acceleration. Returns radians
float accelerationToAngle(float x, float y){
  
  float x0 = x;
  float y0 = y;
  float a = sqrt(sq(x)+sq(y));
  float angle = acos(y/a);
  
 
  if (x<=0){
    angle = -angle;
  }
  
  if(startReady){
    return angle;
  }
  else{
    return 0;
  }
}


float getAverage(int *dataArray, int arraySize){
  float sum = 0.0;

  for (int i=0;i<arraySize;i++){
    sum = sum + dataArray[i];
  }
  sum = sum/arraySize;
  return sum;
}

void sendValues(float a, float b, float c){
    Serial.print(a);
    Serial.print(";");
    Serial.print(b);
    Serial.print(";");
    Serial.println(c);
}

//Keep track of accSampleCC and send logged data
void accSampleControl(){
  if(accSampleCC>=ACC_FILTER_SAMPLES-1){
    accSampleCC=0;
     startReady = true;
  }
  else{
    accSampleCC++;
  }
}

//Check that dead mans switch is still on
void emergencyControl(){
  
   boolean pinLevel = digitalRead(EMERGENCY_PIN);
   if(pinLevel){
      emergency = false;
   } 
   else{
      emergency = true; 
   }
  
}

void gyroSampleControl(){
    if(gyroSampleCC>=GYRO_FILTER_SAMPLES-1){
    gyroSampleCC=0;
     
  }
  else{
    gyroSampleCC++;
  }

}
