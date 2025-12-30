// Mahony AHRS for the ICM_20948  S.J. Remington 6/2021
// Requires the Sparkfun ICM_20948 library
// Standard sensor orientation X North (yaw=0), Y West, Z up
// magnetometer Y and Z axes are reflected to reconcile with accelerometer.

// New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc cross Mag) as the orientation reference vectors
// heavily modified from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
//
// To collect data for calibration, use the companion program ICM_20948_get_cal_data
//
/*
  Sparkfun ICM_20948
  Hardware setup: This library supports communicating with the
  ICM_20948 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
  ICM_20948 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VIN ------------- 5V or 3.3V
   GND ------------- GND

*/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
//#include <simplot.h> //SIMPLOT FOR DEBUGGING PURPOSE ONLY
#include "GPSport.h"

//////////////////////////
// ICM_20948 Library Init //
//////////////////////////
// default settings for accel and magnetometer

#define WIRE_PORT Wire // desired Wire port.
//#define AD0_VAL 1      // value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when
// the ADR jumper is closed the value becomes 0

#define G_OFFSET_MAX 1000
#define G_OFFSET_MIN -1000
#define B_MAX 1000
#define B_MIN -1000
#define AINV_MAX 50
#define AINV_MIN-50

ICM_20948_I2C imu; // create an ICM_20948_I2C object imu;

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The compass will NOT work well or at all if these are not correct

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float Gscale = (M_PI / 180.0) * 0.00763; //250 dps scale sensitivity = 131 dps/LSB
float p[] = {1, 0, 0};  //X marking on sensor board points toward yaw = 0

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

unsigned long now = 0, last = 0; //micros() timers for AHRS loop
float deltat = 0;  //loop time in seconds

#define PRINT_SPEED 300 //300 // ms between angle prints
unsigned long lastPrint = 0; // Keep track of print time

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output

// --- START: add yaw-rate and yaw-acceleration calculation ---
static float filtered_psi_dot = 0.0f;  // filtered yaw rate (rad/s)
static float filtered_yaw_accel = 0.0f;// filtered yaw angular acceleration (rad/s^2)


////Gyro default scale 250 dps. Convert to radians/sec subtract offsets
//float G_offset[3] = {240.7, 227.7, -4.8};
////Accel scale: divide by 16604.0 to normalize
//float A_B[3] = { 470.7 , -83.03 , 300.65 };
//float A_Ainv[3][3] = {
//{ 0.06232 , -0.00413 , -0.00479 },
//{ -0.00413 , 0.0632 , -0.0017 },
//{ -0.00479 , -0.0017 , 0.0589 }};
////Mag scale divide by 369.4 to normalize
//float M_B[3] = { -80.39 , -35.59 , 18.54 };
//float M_Ainv[3][3] = {
//{ 4.47829 , -0.06586 , 0.02004 },
//{ -0.06586 , 4.53222 , 0.00443 },
//{ 0.02004 , 0.00443 , 4.46886 }};
// Hardcoded values are managed at DevICM20948.cpp
float G_offset[3];
float A_B[3];
float A_Ainv[3][3];
float M_B[3];
float M_Ainv[3][3];

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void vector_cross(float a[3], float b[3], float out[3]);
void vector_normalize(float a[3]);
float vector_dot(float a[3], float b[3]);


extern bool ICM20948AHRS_setup(bool orientation = false)
{
  //Orientation not implemented
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  imu.begin(WIRE_PORT, 0);
  if (imu.status != ICM_20948_Stat_Ok) {
	  imu.begin(WIRE_PORT, 1);
	  if (imu.status != ICM_20948_Stat_Ok) {
		  DEBUG_print("ICM_90248 not detected\n");
		  return false;
	  }
  }
  return true;
}

// Returns predicted yaw delta (degrees) after dt_future seconds
float ICM20948AHRS_predictYawDelta(float dt_future= 1) {
    // filtered_psi_dot: yaw rate (rad/s)
    // yaw_accel: yaw acceleration (rad/s^2)

    float delta_yaw_rad =
          filtered_psi_dot * dt_future
        + 0.5f * filtered_yaw_accel * dt_future * dt_future;

    // Convert to degrees
    float delta_yaw_deg = delta_yaw_rad * 180.0f / PI;

    return delta_yaw_deg;
}

float get_filtered_psi_dot (void) {
	return filtered_psi_dot;
}

float get_yaw_accel (void) {
	return filtered_yaw_accel;
}



float ICM20948AHRS_loop()
{

  static float GAxyz[3], AAxyz[3], MAxyz[3]; //centered and scaled gyro/accel/mag data
  static float ACxyz[3], MCxyz[3]; //centered and scaled accel/mag data
  static float deltat_avg=1;
  float alfa = 0.7;
  // Update the sensor values whenever new data is available
  if ( imu.dataReady() ) {

    imu.getAGMT();

    get_scaled_IMU(GAxyz, AAxyz, MAxyz);

    // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

    MAxyz[1] = -MAxyz[1]; //reflect Y and Z
    MAxyz[2] = -MAxyz[2]; //must be done after offsets & scales applied to raw data

    now = micros(); // TODO: Overflow after a few hours
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;

    deltat_avg = deltat_avg*alfa + deltat*(1-alfa);

    MahonyQuaternionUpdate(AAxyz[0], AAxyz[1], AAxyz[2], GAxyz[0], GAxyz[1], GAxyz[2],
                           MAxyz[0], MAxyz[1], MAxyz[2], deltat);


    if ((deltat-deltat_avg) > 0.03) {
    	//DEBUG_print("!ICM20948: Time overflow\n");
    	return yaw;
    }



      // Define Tait-Bryan angles. Strictly valid only for approximately level movement

      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order.
      //
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.

      // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
      // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock

      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));

      // Compute body rates p,q,r from GAxyz (these are already scaled in get_scaled_IMU)
      float p = GAxyz[0]; // roll rate (rad/s)
      float q = GAxyz[1]; // pitch rate (rad/s)
      float r = GAxyz[2]; // yaw rate about body Z (rad/s)

      // Avoid division by near-zero cos(pitch)
      float cos_pitch = cos(pitch);
      if (fabs(cos_pitch) < 1e-3f) cos_pitch = (cos_pitch >= 0) ? 1e-3f : -1e-3f;

      // Convert body rates to Euler yaw rate (psi_dot) using standard transform:
      // psi_dot = (sin(roll)/cos(pitch)) * q + (cos(roll)/cos(pitch)) * r
      float psi_dot = (sin(roll) / cos_pitch) * q + (cos(roll) / cos_pitch) * r;

      // Simple exponential filter for yaw rate to reduce noise (alpha between 0..1, smaller=>more smoothing)
      #define PSI_DOT_ALPHA 0.90f

      float last_psi_dot = filtered_psi_dot;
      filtered_psi_dot = PSI_DOT_ALPHA * filtered_psi_dot + (1.0f - PSI_DOT_ALPHA) * psi_dot;

      // Compute yaw angular acceleration (difference divided by dt) with small safeguard
      float dt_local = deltat;
      if (dt_local <= 0.0f) dt_local = 1e-6f;

      // original: // if ((deltat-deltat_avg) > 0.03) { ...
       // if ((deltat-deltat_avg) > 0.03) { ...

      // New: compute yaw acceleration
	  #define YAW_ACCEL 0.95f
      filtered_yaw_accel = filtered_yaw_accel*YAW_ACCEL + (1.0f-YAW_ACCEL)* (filtered_psi_dot - last_psi_dot) / dt_local;

      // to degrees
      yaw   *= 180.0 / PI;
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      yaw = -(yaw);// + declination);
      if (yaw < 0) yaw += 360.0;
      if (yaw >= 360.0) yaw -= 360.0;

      ICM20948AHRS_predictYawDelta();
  }
  return yaw;
}

//// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
//// applies magnetic declination
//int get_heading(float acc[3], float mag[3], float p[3], float magdec){
//	float W[3], N[3]; //derived direction vectors
//	// cross "Up" (acceleration vector, g) with magnetic vector (magnetic north + inclination) with  to produce "West"
//	vector_cross(acc, mag, W);
//	vector_normalize(W);
//
//	// cross "West" with "Up" to produce "North" (parallel to the ground)
//	//vector_cross(W, acc, N);
//	vector_normalize(N);
//
//	// compute heading in horizontal plane, correct for local magnetic declination in degrees
//
//	float h = -atan2(vector_dot(W, p), vector_dot(N, p)) * 180 / M_PI; //minus: conventional nav, heading increases North to East
//	int heading = round(h + magdec);
//	heading = (heading + 720) % 360; //apply compass wrap
//	return heading;
//}

// vector math
float vector_dot(float a[3], float b[3]){
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
// basic vector operations
void vector_cross(float a[3], float b[3], float out[3]){
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
void vector_normalize(float a[3]){
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
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
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
    eInt[0] += ex;      // accumulate integral error
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


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
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

void get_scaled_IMU2(float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
  Axyz[0] = imu.agmt.acc.axes.x;
  Axyz[1] = imu.agmt.acc.axes.y;
  Axyz[2] = imu.agmt.acc.axes.z;
  Mxyz[0] = imu.agmt.mag.axes.x;
  Mxyz[1] = imu.agmt.mag.axes.y;
  Mxyz[2] = imu.agmt.mag.axes.z;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply offsets (bias) and scale factors from Magneto
  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}



extern bool ICM20948AHRS_getOffsets(float aG_offset[3], float aA_B[3], float aA_Ainv[3][3], float aM_B[3], float aM_Ainv[3][3]){
	for (int i = 0; i < 3; i++) {
		aG_offset[i] = G_offset[i];
		aA_B[i] = A_B[i];
		aM_B[i] = M_B[i];
	}

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
        	aA_Ainv[i][j] = A_Ainv[i][j];
        	aM_Ainv[i][j] = M_Ainv[i][j];
        }
    }
	return true;
}

extern bool ICM20948AHRS_checkOffsets(float aG_offset[3], float aA_B[3], float aA_Ainv[3][3], float aM_B[3], float aM_Ainv[3][3]){
	for (int i = 0; i < 3; i++) {
		if (aG_offset[i] > G_OFFSET_MAX) return false;
		if (aG_offset[i] < G_OFFSET_MIN) return false;
		if (aA_B[i] > B_MAX) return false;
		if (aA_B[i] < B_MIN) return false;
		if (aM_B[i] > B_MAX) return false;
		if (aM_B[i] < B_MIN) return false;
	}

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
    		if (aA_Ainv[i][j] > AINV_MAX) return false;
    		if (aA_Ainv[i][j] < AINV_MIN) return false;
    		if (aM_Ainv[i][j] > AINV_MAX) return false;
    		if (aM_Ainv[i][j] < AINV_MIN) return false;
        }
    }
	return true;
}

extern bool ICM20948AHRS_setOffsets(float aG_offset[3], float aA_B[3], float aA_Ainv[3][3], float aM_B[3], float aM_Ainv[3][3]){
	for (int i = 0; i < 3; i++) {
		G_offset[i] = aG_offset[i];
		A_B[i] = aA_B[i];
		M_B[i] = aM_B[i];
	}

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
        	A_Ainv[i][j] = aA_Ainv[i][j];
        	M_Ainv[i][j] = aM_Ainv[i][j];
        }
    }
	return true;
}
//
//bool setoffsets_G(float aG_offset[3]){
//	for (int i = 0; i < 3; i++) {
//		G_offset[i] = aG_offset[i];
//	}
//	return true;
//}

extern bool ICM20948AHRS_setoffsets2(float aB[3], float aAinv[3][3], char sensor){
	switch (sensor) {
	case 'G':
		for (int i = 0; i < 3; i++) {
			G_offset[i] = aB[i];
		}
		break;
	case 'A':
		for (int i = 0; i < 3; i++) {
			A_B[i] = aB[i];
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				A_Ainv[i][j] = aAinv[i][j];
			}
		}
		break;
	case 'M':
		for (int i = 0; i < 3; i++) {
			M_B[i] = aB[i];
		}

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				M_Ainv[i][j] = aAinv[i][j];
			}
		}
		break;
	}
	return true;
}
/////////////////////////////////////////////////////
// CALIBRATION FUNCTIONS
/////////////////////////////////////////////////////

//extern bool ICM20948AHRS_calibration_setup()
//{
//  // gyro offset values for calibration
//  static float gyro[3] = {0};
//  int offset_count = 500; //average this many values for gyro
//  int i;
//  // reset
//  for (i = 0; i < 3; i++) {gyro[i]=0;}
//
//  for (i = 0; i < offset_count; i++) {
//    if (imu.dataReady())
//    {
//      imu.getAGMT();         // The values are only updated when you call 'getAGMT'
//      gyro[0] += imu.agmt.gyr.axes.x;
//      gyro[1] += imu.agmt.gyr.axes.y;
//      gyro[2] += imu.agmt.gyr.axes.z;
//    }
//  }
//
//  for (i = 0; i < 3; i++) {
//	  gyro[i] = gyro[i] / offset_count;
//  }
//  setoffsets_G(gyro); //Load gyro offsets
//
//return true;
//}

extern int16_t* ICM20948AHRS_calibration_loop(char sensor) {
	static int16_t gyro_acc_mag[3] = {0};

//  // get values for calibration of gyro/acc/mag
    if (imu.dataReady()) {
      imu.getAGMT();         // The values are only updated when you call 'getAGMT'

      switch (sensor) {
      case 'G':
    	  gyro_acc_mag[0] = imu.agmt.gyr.axes.x;
    	  gyro_acc_mag[1] = imu.agmt.gyr.axes.y;
    	  gyro_acc_mag[2] = imu.agmt.gyr.axes.z;
    	  break;
      case 'A':
    	  gyro_acc_mag[0] = imu.agmt.acc.axes.x;
    	  gyro_acc_mag[1] = imu.agmt.acc.axes.y;
    	  gyro_acc_mag[2] = imu.agmt.acc.axes.z;
    	  break;
      case 'M':
    	  gyro_acc_mag[0] = imu.agmt.mag.axes.x;
    	  gyro_acc_mag[1] = imu.agmt.mag.axes.y;
    	  gyro_acc_mag[2] = imu.agmt.mag.axes.z;
    	  break;
      }
    }
    return gyro_acc_mag;
}
