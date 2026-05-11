#include "IMUDevice.h"

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
#ifdef MINIMU9V5
	return new DevMinIMU9V5;
#endif

#ifdef ICM20948
	return new DevICM20948;
#endif

}
