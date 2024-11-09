#include "IMUDevice.h"

#ifdef BNO055_INTERNAL_FUSION
	#include "DevBNO055Int.h"
#endif

#ifdef BNO055_EXTERNAL_FUSION
	#include "DevBNO055Ext.h"
#endif

#ifdef MINIMU9V5
	#include "DevMinIMU9V5.h"
#endif

#ifdef ICM20948
	#include "DevICM20948.h"
#endif

IMUDevice::IMUDevice()
{
}

IMUDevice::~IMUDevice ()
{
}

IMUDevice *IMUDevice::createIMUDevice(void)
{
#ifdef BNO055_INTERNAL_FUSION
	return new DevBNO055Int; // create the imu object
#endif

#ifdef BNO055_EXTERNAL_FUSION
	return new DevBNO055Ext;
#endif

#ifdef MINIMU9V5
	return new DevMinIMU9V5;
#endif

#ifdef ICM20948
	return new DevICM20948;
#endif

}
