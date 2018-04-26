/*
   MPU6050
   ________________________________________
*/
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double oldKalAngleX;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

/*
   Communication
   __________________________________________
*/

/*
   Small letters are actual small letters. Capslock is an int-representation
   Protocol:
      To receive a position:               p ->
      To receive speed:                    s -> SPEED
      To receive new L-values and scaler:  l ->


   The Serial port used for bluetooth is Serial3
*/

/*
   _________________________________________
*/


#define LED 13
#define MOTOR_PIN_F 44
#define MOTOR_PIN_B 45
#define ENCODER_F 18
#define ENCODER_B 19
//#define ENCODER_F 7
//#define ENCODER_B 6

int time = 0;
double h = 20000;
double pos = 0, newPos = 0, posChange = 0;
int speed = 100;
int u = 0;
int counter = 0;

//double L1 = 0, L2 = 400000, L3 = 70, L4 = 2000000;
//double scaler =1.2, L1 = 0.40*scaler, L2 = 500000*scaler, L3 = 70*scaler, L4 = 2200000*scaler; // 10 sekunder
//double scaler =1.2, L1 = 0.20*scaler, L2 = 800000*scaler, L3 = 75*scaler, L4 = 2300000*scaler; // 20 sekunder

double scaler = 1.2, L1 = 0.20 * scaler, L2 = 800000 * scaler, L3 = 45 * scaler, L4 = 2300000 * scaler; // FUNGERAR MED NYA BATTERIER!!

//double scaler =1.25, L1 = 0.50*scaler, L2 = 800000*scaler, L3 = 45*scaler, L4 = 2300000*scaler; // Lite tightare position

double velocity, oldVelocity;
double encoderTimer, oldEncoderTimer;
double angleVelocity, angleOffset = 92.9; //To set zero in equilibrium. Lower towards battery
boolean goingForward = false;


void setup() {
  setupPins();
  setupInterrupts();
  setupMPU6050();
  Serial.begin(115200);
  Serial3.begin(9600);
  Serial.println("Setup done - 1");
  Serial3.println("Setup done - 3");
}

void setupPins() {
  pinMode(LED, OUTPUT);
  pinMode(MOTOR_PIN_F, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(ENCODER_F, INPUT);
  pinMode(ENCODER_B, INPUT);
}

void setupInterrupts() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_F), readEncoder, RISING);
}

void loop() {
  int tempTimer = micros();
  getAngle();
  getAngleVel();
  changePos();
  calcU();
  runMotors();
  checkBT();
  if (abs(kalAngleX + angleOffset) > 30) {
    stopMotor();
  }
  int tempTimer2 = micros();
  //Serial.print(tempTimer2 - tempTimer); Serial.print("\t");
  delayMicroseconds(h - (tempTimer2 - tempTimer));
}

void checkBT() {
  if (Serial3.available() > 0) {
    switch (Serial3.read()) {
      case 'p':
        newPos = readInt();
        break;
      case 's':
        speed = readInt();
        break;
      case 'l':
        break;
      default:
        while (Serial3.available() > 0) {
          Serial3.read();
        }
        break;
    }
  }
}

void changePos() {
  if (abs(posChange - newPos) > 10) {
    if (posChange < newPos) {
      posChange += speed / 100.0;
      pos += speed / 100.0;
    } else if (posChange > newPos) {
      posChange -= speed / 100.0;
      pos -= speed / 100.0;
    }
  }
}

void calcU() {
  if (oldVelocity == velocity) {
    velocity = 0;
  }
  oldVelocity = velocity;
  u = (int)(L1 * pos + L2 * velocity + L3 * (kalAngleX + angleOffset) + L4 * angleVelocity);
  
    Serial.print(L1 * pos, 14); Serial.print('\t');
    Serial.print(L2 * velocity, 14); Serial.print('\t');
    Serial.print(L3 * (kalAngleX + angleOffset), 14); Serial.print('\t');
    Serial.print(L4 * angleVelocity, 14); Serial.print('\t');
    Serial.print(saturate(u)); Serial.println('\t');
  
}

/*
   Read an int, doesn't block and reads even if it is not an int
*/
int readInt() {
  String s = "";
  while (Serial3.available()) {
    s += (char)Serial3.read();
  }
  return s.toInt();
}

void runMotors() {
  if (u < 0) {
    u = saturate(u);
    forward(-u);
  } else {
    u = saturate(u);
    backward(u);
  }
}

int saturate(int i) {
  if (i > 255) {
    i = 255;
  } else if (u < -255) {
    i = -255;
  }
  return i;
}

void forward() {
  digitalWrite(MOTOR_PIN_B, LOW);
  digitalWrite(MOTOR_PIN_F, HIGH);
  digitalWrite(LED, HIGH);
}

void forward(int speed) {
  digitalWrite(MOTOR_PIN_B, LOW);
  analogWrite(MOTOR_PIN_F, speed);
  digitalWrite(LED, HIGH);
}

void backward() {
  digitalWrite(MOTOR_PIN_F, LOW);
  digitalWrite(MOTOR_PIN_B, HIGH);
  digitalWrite(LED, LOW);
}

void backward(int speed) {
  digitalWrite(MOTOR_PIN_F, LOW);
  analogWrite(MOTOR_PIN_B, speed);
  digitalWrite(LED, LOW);
}

void stopMotor() {
  digitalWrite(MOTOR_PIN_F, LOW);
  digitalWrite(MOTOR_PIN_B, LOW);
}

void readEncoder() {
  if ((digitalRead(ENCODER_F) << 1) + digitalRead(ENCODER_B) == 2) {
    oldEncoderTimer = encoderTimer;
    encoderTimer = micros();
    velocity = 1 / (encoderTimer - oldEncoderTimer);
    ++pos;
  } else {
    oldEncoderTimer = encoderTimer;
    encoderTimer = micros();
    velocity = -1 / (encoderTimer - oldEncoderTimer);
    --pos;
  }
}

void getAngleVel() {
  angleVelocity = (kalAngleX - oldKalAngleX) / h;
}

void setupMPU6050() {
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}


void getAngle() {
  /* Save old value*/
  oldKalAngleX = kalAngleX;
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
  /*#if 0 // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");

    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");

    Serial.print("\t");
    #endif

    Serial.print(roll); Serial.print("\t");
    Serial.print(gyroXangle); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");

    Serial.print("\t");

    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");
  */
#if 0 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  //Serial.print("\r\n");
  delay(2);
}




