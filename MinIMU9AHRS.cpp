/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
#define IMU_V5

# define COMPONENTS_ON_TOP true
# define COMPONENTS_BOTTOM false
#define MAX_ERROR 15

int SENSOR_SIGN[9];

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>
#include "GPSport.h" // uncomment this line to print debug messages to serial monitor

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpose timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
//byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};


/**************************************************/
//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!).
void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
{
  for(int x = 0; x < 3; x++)
  {
    for(int y = 0; y < 3; y++)
    {
      mat[x][y] = 0;

      for(int w = 0; w < 3; w++)
      {
        mat[x][y] += a[x][w] * b[w][y];
      }
    }
  }
}

//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
  float op=0;

  for(int c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }

  return op;
}

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar.
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
  for(int c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2;
  }
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}




#ifdef IMU_V5

#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 gyro_acc;
LIS3MDL mag;

#else // older IMUs through v4

#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

#endif

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

LIS3MDL::vector<int16_t> m_min = {-1000, -1000, -1000};
LIS3MDL::vector<int16_t> m_max = {1000, 1000, 1000};


//#define M_X_MIN -1000
//#define m_min.y -1000
//#define M_Z_MIN -1000
//#define M_X_MAX +1000
//#define M_Y_MAX +1000
//#define M_Z_MAX +1000


void I2C_Init()
{
  Wire.begin();
}

void Gyro_Init()
{
#ifdef IMU_V5
  // Accel_Init() should have already called gyro_acc.init() and enableDefault()
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
#else
  gyro.init();
  gyro.enableDefault();
  gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
#endif
}

void Read_Gyro()
{
#ifdef IMU_V5
  gyro_acc.readGyro();

  AN[0] = gyro_acc.g.x;
  AN[1] = gyro_acc.g.y;
  AN[2] = gyro_acc.g.z;
#else
  gyro.read();

  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
#endif

  gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init()
{
#ifdef IMU_V5
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
#else
  compass.init();
  compass.enableDefault();
  switch (compass.getDeviceType())
  {
    case LSM303::device_D:
      compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
#endif
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
#ifdef IMU_V5
  gyro_acc.readAcc();

  AN[3] = gyro_acc.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = gyro_acc.a.y >> 4;
  AN[5] = gyro_acc.a.z >> 4;
#else
  compass.readAcc();

  AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  AN[4] = compass.a.y >> 4;
  AN[5] = compass.a.z >> 4;
#endif
  accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void Compass_Init()
{
#ifdef IMU_V5
  mag.init();
  mag.enableDefault();
#else
  // LSM303: doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
#endif
}

void Read_Compass()
{
#ifdef IMU_V5
  mag.read();

  magnetom_x = SENSOR_SIGN[6] * mag.m.x;
  magnetom_y = SENSOR_SIGN[7] * mag.m.y;
  magnetom_z = SENSOR_SIGN[8] * mag.m.z;
#else
  compass.readMag();

  magnetom_x = SENSOR_SIGN[6] * compass.m.x;
  magnetom_y = SENSOR_SIGN[7] * compass.m.y;
  magnetom_z = SENSOR_SIGN[8] * compass.m.z;
#endif
}

void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;

  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);

  // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
  c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*m_min.x) / (m_max.x - m_min.x) - SENSOR_SIGN[6]*0.5;
  c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*m_min.y) / (m_max.y - m_min.y) - SENSOR_SIGN[7]*0.5;
  c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*m_min.z) / (m_max.z - m_min.z) - SENSOR_SIGN[8]*0.5;

  // Tilt compensated Magnetic filed X:
  MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
}

/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;

  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift.
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;


  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);

  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);

  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading

  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.

  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
}
*/
/**************************************************/

void Matrix_update(void)
{
  Gyro_Vector[0]=Gyro_Scaled_X(gyro_x); //gyro x roll
  Gyro_Vector[1]=Gyro_Scaled_Y(gyro_y); //gyro y pitch
  Gyro_Vector[2]=Gyro_Scaled_Z(gyro_z); //gyro Z yaw

  Accel_Vector[0]=accel_x;
  Accel_Vector[1]=accel_y;
  Accel_Vector[2]=accel_z;

  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement

 #if OUTPUTMODE==1
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
 #else                    // Uncorrected data (no drift correction)
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
 #endif

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    }
  }
}

void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

// return true - Calibrated ok
// return false - Not calibrated
extern bool MinIMU9AHRS_ag_calibration_loop(void) {
	static bool repeatCal = true;
	static int AN_OFFSET_ERROR[6];
	static int AN_PREV[6];

		//reset variables
		repeatCal = false;
		for(int y=0; y<6; y++) {
			AN_OFFSET[y]=0;
			AN_OFFSET_ERROR[y]=0;
		}
		for(int i=0;i<32;i++) {   // We take some readings...
			Read_Gyro();
			Read_Accel();
			for(int y=0; y<6; y++) {
				// Cumulate values
				AN_OFFSET[y] += AN[y];
				if (i>0) {
					// Cumulate error
					AN_OFFSET_ERROR[y] += abs(AN[y]-AN_PREV[y]);
				}
				AN_PREV[y]=AN[y];
			}
		delay(20);
		}

		for(int y=0; y<6; y++) {
			AN_OFFSET_ERROR[y]=AN_OFFSET_ERROR[y]/31;
			if (abs(AN_OFFSET_ERROR[y])>MAX_ERROR) {
				return false; // no calibrado
			}
		}

	for(int y=0; y<6; y++)
	  AN_OFFSET[y] = AN_OFFSET[y]/32;

	AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

	DEBUG_print("Ok\n");

	return true; // calibrado correctamente
}

extern void MinIMU9AHRS_setup(bool orientation = COMPONENTS_ON_TOP)
{

	if (orientation ==COMPONENTS_BOTTOM) {
	   // X axis pointing forward
	   // Y axis pointing to the right
	   // and Z axis pointing down.
	// Positive pitch : nose up
	// Positive roll : right wing down
	// Positive yaw : clockwise
	//SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
	SENSOR_SIGN[0] = 1;
	SENSOR_SIGN[1] = 1;
	SENSOR_SIGN[2] = 1;
	SENSOR_SIGN[3] = -1;
	SENSOR_SIGN[4] = -1;
	SENSOR_SIGN[5] = -1;
	SENSOR_SIGN[6] = 1;
	SENSOR_SIGN[7] = 1;
	SENSOR_SIGN[8] = 1;
	} else { //COMPONENTS_ON_TOP
	   // X axis pointing forward
	   // Y axis pointing to the left
	   // and Z axis pointing up.
	// Positive pitch : nose down
	// Positive roll : right wing down
	// Positive yaw : counterclockwise
	//SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
		SENSOR_SIGN[0] = 1;
		SENSOR_SIGN[1] = -1;
		SENSOR_SIGN[2] = -1;
		SENSOR_SIGN[3] = -1;
		SENSOR_SIGN[4] = 1;
		SENSOR_SIGN[5] = 1;
		SENSOR_SIGN[6] = 1;
		SENSOR_SIGN[7] = -1;
		SENSOR_SIGN[8] = -1;

	}

  I2C_Init();

  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(100);//(20) to allow accel. to stabilize

  timer=millis();
  delay(20);
  counter=0;
}

extern float MinIMU9AHRS_loop() //Main Loop
{
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;



    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***

    return ToDeg(yaw);
  }

}

// Calibration code from LIS3MDL library example code

LIS3MDL::vector<int16_t> running_min = {32767, 32767, 32767};
LIS3MDL::vector<int16_t> running_max = {-32768, -32768, -32768};


extern bool MinIMU9AHRS_calibration_setup(void) {
	//Reset values
	running_min = {32767, 32767, 32767};
	running_max = {-32768, -32768, -32768};
	return true;
}

bool equal (LIS3MDL::vector<int16_t> l, LIS3MDL::vector<int16_t> r) {
	return (l.x==r.x && l.y==r.y && l.z==r.z);
}

extern bool MinIMU9AHRS_calibration_loop(void)
{
	bool changed= false;
	mag.read();

  running_min.x = min(running_min.x, mag.m.x);
  running_min.y = min(running_min.y, mag.m.y);
  running_min.z = min(running_min.z, mag.m.z);

  running_max.x = max(running_max.x, mag.m.x);
  running_max.y = max(running_max.y, mag.m.y);
  running_max.z = max(running_max.z, mag.m.z);

  //Update calibration offsets
  if (!equal(m_min,running_min)) {
	  m_min = running_min;
	  changed = true;
  }
  if (!equal(m_max, running_max)) {
	  m_max = running_max;
	  changed = true;
  }

  return changed;
}





struct s_offset {int16_t x, y, z;};
extern bool MinIMU9AHRS_getOffsets(s_offset &am_min, s_offset &am_max, s_offset &ag_offset, s_offset &aa_offset){
	am_min.x = m_min.x;
	am_min.y = m_min.y;
	am_min.z = m_min.z;
	am_max.x = m_max.x;
	am_max.y = m_max.y;
	am_max.z = m_max.z;
	ag_offset.x = AN_OFFSET[0];
	ag_offset.y = AN_OFFSET[1];
	ag_offset.z = AN_OFFSET[2];
	aa_offset.x = AN_OFFSET[3];
	aa_offset.y = AN_OFFSET[4];
	aa_offset.z = AN_OFFSET[5];

	return true;
}
extern bool MinIMU9AHRS_setOffsets(s_offset &am_min, s_offset &am_max, s_offset &ag_offset, s_offset &aa_offset){
	m_min.x = am_min.x;
	m_min.y = am_min.y;
	m_min.z = am_min.z;
	m_max.x = am_max.x;
	m_max.y = am_max.y;
	m_max.z = am_max.z;
	AN_OFFSET[0] = ag_offset.x;
	AN_OFFSET[1] = ag_offset.y;
	AN_OFFSET[2] = ag_offset.z;
	AN_OFFSET[3] = aa_offset.x;
	AN_OFFSET[4] = aa_offset.y;
	AN_OFFSET[5] = aa_offset.z;
	return true;
}


