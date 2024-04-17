/*Sparkfun ICM_20948
  Hardware setup: 
  ICM_20948 --------- Arduino
   SCL ---------- SCL (A5)
   SDA ---------- SDA (A4)
   VIN ------------- 5V or 3.3V
   GND ------------- GND
*/

#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h" // http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <inttypes.h>
#include <math.h>
#include <stdint.h>

//////////////////////////
// ICM_20948 Library Init //
//////////////////////////
// default settings for accel and magnetometer

#define WIRE_PORT Wire // desired Wire port.
#define AD0_VAL 1      // value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when
// the ADR jumper is closed the value becomes 0

ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; //250 dps scale sensitivity = 131 dps/LSB
float G_offset[3] = {-60.0, 95.0, -25.0};

//Accel scale: divide by 16604.0 to normalize
float A_B[3]
{   381.65,  -59.38,  437.89};

float A_Ainv[3][3]
{ { 1.03957, -0.00077,  0.00275},
  {-0.00077,  0.99988,  0.01389},
  { 0.00275,  0.01389,  1.00222}
};

//Mag scale divide by 369.4 to normalize
float M_B[3]
{ 8.07,  313.50, 189.24};

float M_Ainv[3][3]
{ { 2.38167, 0.17202,  0.12910},
  { 0.17202, 2.10826, -0.06205},
  { 0.12910,-0.06205,  2.16027}
};

// local magnetic declination in degrees
float declination = -0.0667;

// These are the free parameters in the Mahony filter and fusion scheme
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle outputs
static float acceleration;

int motor1pin1 = 2;
int motor1pin2 = 3;

int motor2pin1 = 4;
int motor2pin2 = 5;

int ledNorth = 6; 
int ledEast = 7;
int ledSouth = 12;
int ledWest = 13;

// Heading control variables
double desiredHeading = 0.0; // Change as needed
double HeadingKp = 2.40; // Proportional gain
double HeadingKi = 0.22; // Integral gain
double HeadingKd = 0.06; // Derivative gain
double error = 0, lastError = 0;
double integral = 0, derivative = 0;


// Speed control variables
bool isInitialRollSet = false;
float initialRoll = 0;
int rollSampleCount = 0;
#define NUM_INIT_SAMPLES 100
double SpeedKp = 3.0; // Tune these based on your system
double SpeedKi = 0.0;
double SpeedKd = 0.3;
double rollError = 0, lastRollError = 0;
double rollIntegral = 0, rollDerivative = 0;
double speedAdjustment;

// State definitions
enum RobotState {INITIALIZING, DRIVE_FORWARDS, DRIVE_BACKWARDS, IDLE};
RobotState currentState = INITIALIZING;

// Timing control
unsigned long StartTime;
const unsigned long driveTime = 4000; // 4 seconds

//Function prototypes
void setMotorSpeed(int leftSpeed, int rightSpeed);
void headingPID(double speedAdjustment);
double speedPID();
void flashLEDs(int times, int duration);
void updateDirectionLEDs(float heading);

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT);

  pinMode(ledNorth, OUTPUT);
  pinMode(ledEast, OUTPUT);
  pinMode(ledSouth, OUTPUT);
  pinMode(ledWest, OUTPUT);

  Serial.begin(115200);
  while (!Serial); //wait for connection
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, AD0_VAL);
  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.println(F("ICM_90248 not detected"));
    while (1);
  }
}

void loop() {
  static int loop_counter = 0; //sample & update loop counter
  static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data


  // Update the sensor values whenever new data is available
  if ( imu.dataReady() ) {

    imu.getAGMT();

    loop_counter++;
    get_scaled_IMU(Gxyz, Axyz, Mxyz);

    // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

    Mxyz[1] = -Mxyz[1]; //reflect Y and Z
    Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    //   Gxyz[0] = Gxyz[1] = Gxyz[2] = 0;
    MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
                           Mxyz[0], Mxyz[1], Mxyz[2], deltat);

    if (millis() - lastPrint > PRINT_SPEED) {

      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.

      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      //conventional nav, yaw increases CW from North

      yaw = -(yaw + declination) - 90.0;
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      acceleration = sqrt(Axyz[0] * Axyz[0] + Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]) * 9.81;

      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.print(roll, 0);
      Serial.print(", ");
      Serial.print(acceleration, 2);
      loop_counter = 0;
      Serial.println();
      lastPrint = millis(); // Update lastPrint time
    }
  }
  //Drive north only (for testing)
  //desiredHeading = 0.0;

  switch (currentState) {
    case INITIALIZING:
      initialRoll += roll;  // Assuming roll is being updated somewhere in loop before this switch
      rollSampleCount++;
      if (rollSampleCount >= NUM_INIT_SAMPLES) {
        initialRoll /= NUM_INIT_SAMPLES; // Calculate average
        isInitialRollSet = true;
        currentState = DRIVE_FORWARDS; // Move to the next state
        StartTime = millis(); // Reset the timer at the start of this state
        Serial.print("Initial Roll Set: ");
        flashLEDs(2,200);
        Serial.println(initialRoll);
      }
      break;
    case DRIVE_FORWARDS:
      desiredHeading = 90.0;
      speedAdjustment = speedPID();
      headingPID(speedAdjustment);
      /*if (millis() - StartTime > driveTime) {
        currentState = DRIVE_BACKWARDS;
        StartTime = millis();
        }*/
      break;
    case DRIVE_BACKWARDS:
      desiredHeading = 180.0;
      speedAdjustment = speedPID();
      headingPID(speedAdjustment);
      if (millis() - StartTime > driveTime) {
        currentState = DRIVE_FORWARDS;
      }
      break;
    case IDLE:
      setMotorSpeed(0, 0); // Stop motors
      break;
  
    default: currentState = IDLE;
      break;
  }

  delay(50);
}

// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];

  Gxyz[0] = Gscale * (imu.agmt.gyr.axes.x - G_offset[0]);
  Gxyz[1] = Gscale * (imu.agmt.gyr.axes.y - G_offset[1]);
  Gxyz[2] = Gscale * (imu.agmt.gyr.axes.z - G_offset[2]);

  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;

  //apply accel offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply mag offsets (bias) and scale factors from Magneto

  for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex;   // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


  gx = gx * (0.5*deltat); // pre-multiply common factors
  gy = gy * (0.5*deltat);
  gz = gz * (0.5*deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}


void setMotorSpeed(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) {
        digitalWrite(motor1pin1, LOW);
        digitalWrite(motor1pin2, HIGH);
        analogWrite(9, leftSpeed);
    } else {
        digitalWrite(motor1pin1, HIGH);
        digitalWrite(motor1pin2, LOW);
        analogWrite(9, -leftSpeed); // Adjust for reverse direction
    }

    if (rightSpeed >= 0) {
        digitalWrite(motor2pin1, LOW);
        digitalWrite(motor2pin2, HIGH);
        analogWrite(10, rightSpeed);
    } else {
        digitalWrite(motor2pin1, HIGH);
        digitalWrite(motor2pin2, LOW);
        analogWrite(10, -rightSpeed); // Adjust for reverse direction
    }
}

double speedPID() {
  rollError = initialRoll - roll; // Assuming 'pitch' is updated in 'updateSensors'

  rollIntegral += rollError;
  rollIntegral = constrain(rollIntegral, -500, 500); // Prevent wind-up
  rollDerivative = rollError - lastRollError;

  lastRollError = rollError;
  return SpeedKp * rollError + SpeedKi * rollIntegral + SpeedKd * rollDerivative;
}

void headingPID(double speedAdjustment) {
  double currentHeading = yaw; // Due to the placement of the IMU on the robot
  double headingError = desiredHeading - currentHeading;

  updateDirectionLEDs(currentHeading);

  // Heading error adjustment for wrap-around conditions
  if (headingError > 180) headingError -= 360;
  else if (headingError < -180) headingError += 360;

  integral += headingError;
  integral = constrain(integral, -500, 500); // Prevent integral windup
  derivative = headingError - lastError;

  int headingCorrection = HeadingKp * headingError + HeadingKi * integral + HeadingKd * derivative;

  // Determine base motor speeds based on heading
  int baseSpeed = 200;
  int leftBaseSpeed = baseSpeed - headingCorrection;
  int rightBaseSpeed = -baseSpeed - headingCorrection;

  // Apply speed adjustment ensuring it does not change the direction of wheel rotation
  int leftSpeed = constrain(abs(leftBaseSpeed) - (int)speedAdjustment, 0, 255) * (leftBaseSpeed >= 0 ? 1 : -1);
  int rightSpeed = constrain(abs(rightBaseSpeed) - (int)speedAdjustment, 0, 255) * (rightBaseSpeed >= 0 ? 1 : -1);

  setMotorSpeed(leftSpeed, rightSpeed);

  lastError = headingError; // Store the last error for derivative calculation in the next cycle
}


void flashLEDs(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(ledNorth, HIGH);
    digitalWrite(ledEast, HIGH);
    digitalWrite(ledSouth, HIGH);
    digitalWrite(ledWest, HIGH);
    delay(duration);
    digitalWrite(ledNorth, LOW);
    digitalWrite(ledEast, LOW);
    digitalWrite(ledSouth, LOW);
    digitalWrite(ledWest, LOW);
    delay(duration);
  }
}

void updateDirectionLEDs(float heading) {
  // Reset all LEDs
  digitalWrite(ledNorth, LOW);
  digitalWrite(ledEast, LOW);
  digitalWrite(ledSouth, LOW);
  digitalWrite(ledWest, LOW);

  // Determine the direction based on the heading angle
  if ((heading >= 337.5) || (heading < 22.5)) {
    digitalWrite(ledNorth, HIGH); // North
  } else if ((heading >= 22.5) && (heading < 67.5)) {
    digitalWrite(ledNorth, HIGH); // North-East
    digitalWrite(ledWest, HIGH);
  } else if ((heading >= 67.5) && (heading < 112.5)) {
    digitalWrite(ledWest, HIGH); // East
  } else if ((heading >= 112.5) && (heading < 157.5)) {
    digitalWrite(ledWest, HIGH); // South-East
    digitalWrite(ledSouth, HIGH);
  } else if ((heading >= 157.5) && (heading < 202.5)) {
    digitalWrite(ledSouth, HIGH); // South
  } else if ((heading >= 202.5) && (heading < 247.5)) {
    digitalWrite(ledSouth, HIGH); // South-West
    digitalWrite(ledEast, HIGH);
  } else if ((heading >= 247.5) && (heading < 292.5)) {
    digitalWrite(ledEast, HIGH); // West
  } else if ((heading >= 292.5) && (heading < 337.5)) {
    digitalWrite(ledEast, HIGH); // North-West
    digitalWrite(ledNorth, HIGH);
  }
}
