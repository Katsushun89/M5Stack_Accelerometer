#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#define LCD

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
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

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
      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
      // x-axis and Earth magnetic North (or true North if corrected for local
      // declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
      float qw = getQ()[0];
      float qx = getQ()[1];
      float qy = getQ()[2];
      float qz = getQ()[3];
//      // original
//      IMU.yaw   = atan2(
//        2.0f * (qx * qy + qw * qz),
//        qw * qw + qx * qx - qy * qy - qz * qz);
//      IMU.pitch = -asin(2.0f * qx * qz - qw * qy);
//      IMU.roll  = atan2(
//        2.0f * (qw * qx + qy * qz),
//        qw * qw - qx * qx - qy * qy + qz * qz);
//      // original
      // wikipedia
      float sinr = 2.0 * ( qw * qx + qy * qz );
      float cosr = 1.0 - 2.0 * ( qx * qx + qy * qy );
      IMU.roll = atan2( sinr, cosr );

      float sinp = 2.0 * ( qw * qy - qz * qx );
      if ( fabs( sinp ) >= 1.0 )
//        IMU.pitch = copysign( M_PI / 2, sinp );
        IMU.pitch = (sinp > 0) ?(M_PI / 2) :-(M_PI / 2);
      else
        IMU.pitch = asin( sinp );

      float siny = 2.0 * ( qw * qz + qx * qy );
      float cosy = 1.0 - 2.0 * ( qy * qy + qz * qz );
      IMU.yaw = atan2( siny, cosy );
      // wikipedia
      
      IMU.pitch *= RAD_TO_DEG;
      IMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      // Declination of Tokyo (35°40'59.0"N 139°48'32.0"E) is
      //   7° 29' W  ± 0° 18' changing by 0° 4' W on 2018-04-19
//      IMU.yaw   -= 8.5;
      IMU.yaw   += 7.3;
      IMU.roll  *= RAD_TO_DEG;

      //M5.Lcd.setCursor( 0,140);  M5.Lcd.printf("   yaw: %+6.1f",(IMU.yaw));
      //M5.Lcd.setCursor( 0,150);  M5.Lcd.printf(" pitch: %+6.1f",(IMU.pitch));
      //M5.Lcd.setCursor( 0,160);  M5.Lcd.printf("  roll: %+6.1f",(IMU.roll));

      // With these settings the filter is updating at a ~145 Hz rate using the
      // Madgwick scheme and >200 Hz using the Mahony scheme even though the
      // display refreshes at only 2 Hz. The filter update rate is determined
      // mostly by the mathematical steps in the respective algorithms, the
      // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
      // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
      // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
      // presumably because the magnetometer read takes longer than the gyro or
      // accelerometer reads. This filter update rate should be fast enough to
      // maintain accurate platform orientation for stabilization control of a
      // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050
      // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
      // well!

      M5.Lcd.setCursor(0, 140);
      M5.Lcd.printf("yaw:%6.1f   pitch:%6.1f   roll:%6.1f  ypr \r\n",(IMU.yaw), (IMU.pitch), (IMU.roll));
      //M5.Lcd.setCursor( 0, 170);  M5.Lcd.printf("rt(Hz): %6.1f", (float) IMU.sumCount / IMU.sum);

      IMU.count = millis();
      IMU.sumCount = 0;
      IMU.sum = 0;

    } // if (IMU.delt_t > 500)

   // M5.Lcd.setCursor( 0, 0);  M5.Lcd.printf("x(deg.): %6.1f", angle_x * 180.0);
   // M5.Lcd.setCursor( 0,10);  M5.Lcd.printf("y(deg.): %6.1f", angle_y * 180.0);

    if ( resetMPU9250 ) {
      Serial.println("reset MPU !");
      MPU9250_init();
      resetMPU9250 = false;
    }
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  M5.update();
}

