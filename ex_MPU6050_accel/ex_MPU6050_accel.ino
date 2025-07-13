#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <vector>
#include <numeric>

Adafruit_MPU6050 mpu;

// initial conditions
double accel0X = 0, accel0Y = 0, accel0Z = 0;
double accel1X = 0, accel1Y = 0, accel1Z = 0;
double vel0X = 0, vel0Y = 0, vel0Z = 0;
double vel1X = 0, vel1Y = 0, vel1Z = 0;
double pos0X = 0, pos0Y = 0, pos0Z = 0;
double pos1X = 0, pos1Y = 0, pos1Z = 0;
double vel1 = 0, pos1 = 0;
double accelXOffset, accelYOffset, accelZOffset;
double sampleRate;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    sampleRate = 260;
    Serial.print(sampleRate);
    Serial.println(" Hz");
    break;
  case MPU6050_BAND_184_HZ:
    sampleRate = 184;
    Serial.print(sampleRate);
    Serial.println(" Hz");
    break;
  case MPU6050_BAND_94_HZ:
    sampleRate = 94;
    Serial.print(sampleRate);
    Serial.println(" Hz");
    break;
  case MPU6050_BAND_44_HZ:
    sampleRate = 44;
    Serial.print(sampleRate);
    Serial.println(" Hz");
    break;
  case MPU6050_BAND_21_HZ:
    sampleRate = 21;
    Serial.print(sampleRate);
    Serial.println(" Hz");
    break;
  case MPU6050_BAND_10_HZ:
    sampleRate = 10;
    Serial.print(sampleRate);
    Serial.println(" Hz");
    break;
  case MPU6050_BAND_5_HZ:
    sampleRate = 5;
    Serial.print(sampleRate);
    Serial.println(" Hz");
    break;
  }

  Serial.println("");
  delay(100);

  int calibrationCount = 100;
  std::vector<double> accelXCalibrateVector(calibrationCount);
  std::vector<double> accelYCalibrateVector(calibrationCount);
  std::vector<double> accelZCalibrateVector(calibrationCount);
  for (int calibrationCounter = 0; calibrationCounter < calibrationCount; calibrationCounter++){
    sensors_event_t accelCalibrate, gyroCalibrate, tempCalibrate;
    mpu.getEvent(&accelCalibrate, &gyroCalibrate, &tempCalibrate);
    accelXCalibrateVector[calibrationCounter] = accelCalibrate.acceleration.x;
    accelYCalibrateVector[calibrationCounter] = accelCalibrate.acceleration.y;
    accelZCalibrateVector[calibrationCounter] = accelCalibrate.acceleration.z;
    delay(100);
  }
  accelXOffset = average(accelXCalibrateVector);
  accelYOffset = average(accelYCalibrateVector);
  accelZOffset = average(accelZCalibrateVector);

  Serial.println("Accelerometer X Offset [m/s^2]\tAccelerometer Y Offset [m/s^2]\tAccelerometer Z Offset [m/s^2]");
  Serial.print(accelXOffset);
  Serial.print('\t');
  Serial.print(accelYOffset);
  Serial.print('\t');
  Serial.println(accelZOffset);
  Serial.println("a_X [m/s^2]\ta_Y [m/s^2]\ta_Z [m/s^2]\tv_X [m/s]\tv_Y [m/s]\tv_Z [m/s]\tp_X [m]\tp_Y [m]\tp_Z [m]");
}

void loop() {
  float accel_X, accel_Y, accel_Z;
  float vel_X,   vel_Y,   vel_Z;
  float pos_X,   pos_Y,   pos_Z;
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accel_X = a.acceleration.x - accelXOffset;
  accel_Y = a.acceleration.y - accelYOffset;
  accel_Z = a.acceleration.z - accelZOffset;
  vel_X = calculateVelocity(accel_X, sampleRate, "X");
  vel_Y = calculateVelocity(accel_Y, sampleRate, "Y");
  vel_Z = calculateVelocity(accel_Z, sampleRate, "Z");
  pos_X = calculatePosition(vel_X, sampleRate, "X");
  pos_Y = calculatePosition(vel_Y, sampleRate, "Y");
  pos_Z = calculatePosition(vel_Z, sampleRate, "Z");

  Serial.print(accel_X);
  Serial.print('\t');
  Serial.print(accel_Y);
  Serial.print('\t');
  Serial.print(accel_Z);
  Serial.print('\t');
  // Serial.print(" | ");
  Serial.print(vel_X);
  Serial.print('\t');
  Serial.print(vel_Y);
  Serial.print('\t');
  Serial.print(vel_Z);
  Serial.print('\t');
  // Serial.print(" | ");
  Serial.print(pos_X);
  Serial.print('\t');
  Serial.print(pos_Y);
  Serial.print('\t');
  Serial.println(pos_Z);
}


float calculateVelocity(float accel, float sampleRate, String var){
  if (var == "X"){
    accel0X = accel1X;
    accel1X = accel;
    vel0X = vel1X;
    vel1X = vel0X + (accel0X + accel1X)*(1/sampleRate);
    vel1 = vel1X;
  }else if (var == "Y"){
    accel0Y = accel1Y;
    accel1Y = accel;
    vel0Y = vel1Y;
    vel1Y = vel0Y + (accel0Y + accel1Y)*(1/sampleRate);
    vel1 = vel1Y;
  }else if (var == "Z"){
    accel0Z = accel1Z;
    accel1Z = accel;
    vel0Z = vel1Z;
    vel1Z = vel0Z + (accel0Z + accel1Z)*(1/sampleRate);
    vel1 = vel1Z;
  }else{
    vel1 = 999.99;
  }
  return vel1;
}

float calculatePosition(float vel, float sampleRate, String var){
  if (var == "X"){
    vel0X = vel1X;
    vel1X = vel;
    pos0X = pos1X;
    pos1X = pos0X + (vel0X + vel1X)*(1/sampleRate);
    pos1 = pos1X;
  }else if (var == "Y"){
    vel0Y = vel1Y;
    vel1Y = vel;
    pos0Y = pos1Y;
    pos1Y = pos0Y + (vel0Y + vel1Y)*(1/sampleRate);
    pos1 = pos1Y;
  }else if (var == "Z"){
    vel0Z = vel1Z;
    vel1Z = vel;
    pos0Z = pos1Z;
    pos1Z = pos0Z + (vel0Z + vel1Z)*(1/sampleRate);
    pos1 = pos1Z;
  }else{
    pos1 = 999.99;
  }
  return pos1;
}

float average(std::vector<double> const& v){
    if(v.empty()){
        return 0;
    }

    auto const count = static_cast<double>(v.size());
    return std::reduce(v.begin(), v.end()) / count;
}

