#include "ADXL362.h"
#define MAX_ACCEL_DEVICES 1
short x_data[MAX_ACCEL_DEVICES];
short y_data[MAX_ACCEL_DEVICES];
short z_data[MAX_ACCEL_DEVICES];
#include <iostream>
using namespace std;
int main()
{
	CADXL362Accel m_sensors[MAX_ACCEL_DEVICES];

	CADXL362Accel *p_accel = &m_sensors[0];
	p_accel->InitializeSPIDevice();

	if (p_accel->m_active == true && p_accel->m_calibrated == true)
	{	
		while (1)
		{
			p_accel->AccelRead(&x_data[0], &y_data[0], &z_data[0], false); // This can set the accel inactive if error reading
			//cout << "X " << x_data[0] << " Y " << y_data[0] << " Z " << z_data[0] << endl;
		}

	}

	return 0;
}