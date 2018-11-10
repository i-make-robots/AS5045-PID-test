//------------------------------------------------------------------------------
// PID test
// dan@marginallyclever.com 2018-11-09
// uses
// - a DC motor
// - an AS5045 absolute magnetic encoder, mounted on a PCB by madscientisthut.com
// - an arduino MEGA 2560
// - an Adafruit motor shield v1
//------------------------------------------------------------------------------

#include <AutoPID.h>  // found via arduino > sketch > include library > manage libraries 
#include <AFMotor.h>

//------------------------------------------------------------------------------

// SENSOR PINS
#define PIN_SENSOR_CSEL   23
#define PIN_SENSOR_CLK    25
#define PIN_SENSOR_SD_OUT 27

// SENSOR CONSTANTS
#define SENSOR_PARITY_TEST_ON
#define SENSOR_TOTAL_BITS    (18)  // 18 bits of data
#define SENSOR_ANGLE_BITS    (12)
#define SENSOR_ERROR_BITS    (5)
#define SENSOR_PARITY_BITS   (1)
#define SENSOR_STATUS_BITS   (SENSOR_ERROR_BITS+SENSOR_PARITY_BITS)
#define SENSOR_ANGLE_MASK    (0b111111111111000000)
#define SENSOR_ERROR_MASK    (0b000000000000111110)
#define SENSOR_PARITY_MASK   (0b000000000000000001)
#define SENSOR_ANGLE_PER_BIT (360.0/(float)(1<<SENSOR_ANGLE_BITS))

// PID CONSTANTS
#define PID_ANGLE_MAX (180)  // don't change.
#define PID_ANGLE_MIN (-PID_ANGLE_MAX)  // don't change.
#define PID_TIME_STEP (50)  // ms between PID updates.  too small and the motor will oscillate no matter what PID you choose.

#define DEFAULT_KP 1
#define DEFAULT_KI 0.1
#define DEFAULT_KD 0.01

// motor tuning
#define MOTOR_MAXIMUM_POWER 255  // maximum allowable PWM.  must be <=255


//------------------------------------------------------------------------------

// setup the motor global
AF_DCMotor motor(1,MOTOR12_64KHZ);

// for angle calculations
long sourceAngle;
double angle;
double target;

// PID tuning values
double kp=DEFAULT_KP;
double ki=DEFAULT_KI;
double kd=DEFAULT_KD;

// for output.  wildly changes PID results!
int beVerbose=0;

// for PID library interface
double angleZero=0;  // always zero
double targetAdj;    // adjusted based on current angle
double difference;   // suggested direction and force to move
AutoPID myPID(&angleZero, &targetAdj, &difference, PID_ANGLE_MIN, PID_ANGLE_MAX, kp, ki, kd);

int motorMinimumPower;

//------------------------------------------------------------------------------

void setup() {
  Serial.begin(57600);
  Serial.println("\n** START **");

  // sensor setup
  pinMode(PIN_SENSOR_CLK,OUTPUT);
  pinMode(PIN_SENSOR_CSEL,OUTPUT);
  pinMode(PIN_SENSOR_SD_OUT,INPUT);
  
  findMotorMinimum();

  // pid setup
  myPID.setTimeStep(PID_TIME_STEP);

  Serial.println("\n** READY **");
}


// determine the smallest PID value that makes the motor move.
void findMotorMinimum() {
  motorMinimumPower=0;

  // read the sensor
  sourceAngle = sensor_update(PIN_SENSOR_CSEL,PIN_SENSOR_SD_OUT);
  // TODO: check for error codes here.
  double startAngle = sensor_angle(sourceAngle);
  Serial.print("Start angle=");
  Serial.println(startAngle);
  
  for(motorMinimumPower=0;motorMinimumPower<255;++motorMinimumPower) {
    // try to move the motor
    motor.run(FORWARD);
    motor.setSpeed(motorMinimumPower);
    // wait a bit for results
    delay(20);

    // read the sensor again
    sourceAngle = sensor_update(PIN_SENSOR_CSEL,PIN_SENSOR_SD_OUT);
    // TODO: check for error codes here.
    double angleNow = sensor_angle(sourceAngle);

    // be verbose
    Serial.print("PWM ");
    Serial.print(motorMinimumPower);
    Serial.print(" > ");
    Serial.println(angleNow);
    
    // if the motor has move more than a margin of error then we are good!
    if(abs(angleNow-startAngle)>0.1) break;
  }

  if(motorMinimumPower==255) {
    Serial.println("Nothing has moved at max power.  We have a problem.");
    while(1);
  }
  
  // start the motor not moving!
  motor.run(RELEASE);
  motor.setSpeed(0);
}

void loop() {
  // let the user dynamically adjust some values
  if(Serial.available()) {
    char command = Serial.read();
    // target angle
    if(command=='g'||command=='G') {
      target = Serial.parseFloat();
    }
    // PID values
    if(command=='p'||command=='P') {
      kp = Serial.parseFloat();
      myPID.setGains(kp,ki,kd);
    }
    if(command=='i'||command=='I') {
      ki = Serial.parseFloat();
      myPID.setGains(kp,ki,kd);
    }
    if(command=='d'||command=='D') {
      kd = Serial.parseFloat();
      myPID.setGains(kp,ki,kd);
    }
    // verbose output
    if(command=='v'||command=='V') {
      beVerbose = 1-beVerbose;
    }
  }

  // read the sensor
  sourceAngle = sensor_update(PIN_SENSOR_CSEL,PIN_SENSOR_SD_OUT);
  // TODO: check for error codes here.
  angle = sensor_angle(sourceAngle);

  // magic!  adjust the target to be +/-180 from the current angle.  
  targetAdj = target - angle;
  while(targetAdj<PID_ANGLE_MIN) targetAdj += 360;
  while(targetAdj>PID_ANGLE_MAX) targetAdj -= 360;

  myPID.run();

  // scale the difference to compensate for the motor power requirements.
  double differenceScaled = map(abs(difference),0,PID_ANGLE_MAX,motorMinimumPower,255);

  // show some stats
  if(beVerbose) {
    Serial.print(kp);               Serial.print("\t");
    Serial.print(ki);               Serial.print("\t");
    Serial.print(kd);               Serial.print("\t");
    Serial.print(target);           Serial.print("\t");
    //Serial.print(sourceAngle,BIN);  Serial.print("\t");
    Serial.print(angle);            Serial.print("\t");
    Serial.print(targetAdj);        Serial.print("\t");
    Serial.print(difference);       Serial.print("\n");
  }
  
  // move the motor at the recommended direction and speed.
  if(difference>0) {
    motor.setSpeed(differenceScaled);
    motor.run(BACKWARD);
  } else if(difference<0) {
    motor.setSpeed(differenceScaled);
    motor.run(FORWARD);
  }
}


// from http://www.madscientisthut.com/forum_php/viewtopic.php?f=11&t=7
uint32_t sensor_update(int csel,int sdout) {
  uint32_t data = 0, inputStream;
  int x;
  
  // Sensor sends data when CLK goes high.
  // To choose a board, set the CSEL pin high, then tick the clock.
  digitalWrite(csel, HIGH);
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  // We won't need CSEL again until the next sample, so set it low
  digitalWrite(csel, LOW);
  // Set the clock low.  On the next high sensor will start to deliver data.
  digitalWrite(PIN_SENSOR_CLK, LOW);

  for (x=0; x < SENSOR_TOTAL_BITS; ++x) {
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    // one bit of data is now waiting on sensor pin
    inputStream = digitalRead(sdout);
    data = ((data << 1) + inputStream); // left-shift summing variable, add pin value
    digitalWrite(PIN_SENSOR_CLK, LOW);
  }
  return data;
}


/**
 * @input data the raw sensor reading
 * @return the angle in degrees
 */
float sensor_angle(uint32_t data) {
  uint32_t angle = data >> SENSOR_STATUS_BITS; // shift 18-digit angle right 6 digits to form 12-digit value
  angle &= SENSOR_ANGLE_MASK >> SENSOR_STATUS_BITS;  // mask the 18 bits that form the angle
  return (angle * SENSOR_ANGLE_PER_BIT);
}
