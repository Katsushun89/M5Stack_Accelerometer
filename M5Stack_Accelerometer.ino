#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#define LCD
const static uint32_t ACCEL_THR_MAX = 700;
const static uint32_t ACCEL_THR_MIN = 300;

// Devices
MPU9250 IMU;

boolean resetMPU9250 = false;

void setup()
{
  Serial.begin(115200);
  // Power ON Stabilizing...
  delay(500);
  M5.begin();
  Wire.begin();
  
  MPU9250_init();
  
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.println("M5Stack Balance Mode start");
  

}

void MPU9250_init()
{
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) { // WHO_AM_I should always be 0x68
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    IMU.MPU9250SelfTest(IMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    //IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
    delay(1000); 

    IMU.initMPU9250();

    byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);
    if ( d == 0x48 ) {
      Serial.println("AK8963 is online...");

      IMU.initAK8963(IMU.magCalibration);
    }
  }
}

void loop() {
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0] -
               IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1] -
               IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2] -
               IMU.magbias[2];

    // Must be called before updating quaternions!
    IMU.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az,
      IMU.gx * DEG_TO_RAD, IMU.gy * DEG_TO_RAD, IMU.gz * DEG_TO_RAD,
      IMU.mx, IMU.my, IMU.mz, IMU.deltat);

    // Serial print and/or display at 0.5 s rate independent of data rates
    IMU.delt_t = millis() - IMU.count;

    // update LCD once per half-second independent of read rate
    // if (IMU.delt_t > 500)
    if (IMU.delt_t > 100)
    {
#if 0
#ifdef LCD
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(GREEN ,BLACK);
      M5.Lcd.setCursor(0, 0); M5.Lcd.print("MPU9250/AK8963");
      M5.Lcd.setCursor(0, 32); M5.Lcd.print(" x     y     z  ");

      M5.Lcd.setCursor(0,   48); M5.Lcd.print((int)(1000*IMU.ax));
      M5.Lcd.setCursor(36,  48); M5.Lcd.print((int)(1000*IMU.ay));
      M5.Lcd.setCursor(72,  48); M5.Lcd.print((int)(1000*IMU.az));
      M5.Lcd.setCursor(108, 48); M5.Lcd.print("mg");

      M5.Lcd.setCursor(0,   64); M5.Lcd.print((int)(IMU.gx));
      M5.Lcd.setCursor(36,  64); M5.Lcd.print((int)(IMU.gy));
      M5.Lcd.setCursor(72,  64); M5.Lcd.print((int)(IMU.gz));
      M5.Lcd.setCursor(108, 64); M5.Lcd.print("o/s");

      M5.Lcd.setCursor(0,   96); M5.Lcd.print((int)(IMU.mx));
      M5.Lcd.setCursor(36,  96); M5.Lcd.print((int)(IMU.my));
      M5.Lcd.setCursor(72,  96); M5.Lcd.print((int)(IMU.mz));
      M5.Lcd.setCursor(108, 96); M5.Lcd.print("mG");
#endif // LCD  
#endif

#if 1
#ifdef LCD
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(GREEN ,BLACK);
      M5.Lcd.setTextSize(3);
      if(      1000*IMU.ax >=  ACCEL_THR_MAX && abs(1000*IMU.ay) <= ACCEL_THR_MIN && abs(1000*IMU.az) <= ACCEL_THR_MIN){
        M5.Lcd.setCursor(50, 50); M5.Lcd.print("<-");
      }else if(1000*IMU.ax <= -ACCEL_THR_MAX && abs(1000*IMU.ay) <= ACCEL_THR_MIN && abs(1000*IMU.az) <= ACCEL_THR_MIN){
        M5.Lcd.setCursor(50, 50); M5.Lcd.print("->");
      }else if(1000*IMU.ay >=  ACCEL_THR_MAX && abs(1000*IMU.ax) <= ACCEL_THR_MIN && abs(1000*IMU.az) <= ACCEL_THR_MIN){
        M5.Lcd.setCursor(50, 50); M5.Lcd.print("A");
      }else if(1000*IMU.ay <= -ACCEL_THR_MAX && abs(1000*IMU.ax) <= ACCEL_THR_MIN && abs(1000*IMU.az) <= ACCEL_THR_MIN){
        M5.Lcd.setCursor(50, 50); M5.Lcd.print("Z");
      }else if(1000*IMU.az >=  ACCEL_THR_MAX && abs(1000*IMU.ax) <= ACCEL_THR_MIN && abs(1000*IMU.ay) <= ACCEL_THR_MIN){
        M5.Lcd.setCursor(50, 50); M5.Lcd.print("O");
      }else if(1000*IMU.az <= -ACCEL_THR_MAX && abs(1000*IMU.ax) <= ACCEL_THR_MIN && abs(1000*IMU.ay) <= ACCEL_THR_MIN){
        M5.Lcd.setCursor(50, 50); M5.Lcd.print("M");
      }else{
        M5.Lcd.setCursor(50, 50); M5.Lcd.print("not stable");
      }

#endif // LCD  
#endif

      IMU.count = millis();
      IMU.sumCount = 0;
      IMU.sum = 0;

    } // if (IMU.delt_t > 500)

    if ( resetMPU9250 ) {
      Serial.println("reset MPU !");
      MPU9250_init();
      resetMPU9250 = false;
    }
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  M5.update();
}

