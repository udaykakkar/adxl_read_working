#include "ADXL362.h"
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
// #include "globals.h"
// #include "UtilityThread.h"
// #include "SystemPrint.h"
// #include "RefClock.h"
// #include "QT_Mode.h"
// #include "QT_Error.h"
// #include "QuakeTripConfig.h"
// #include "Data_Logging.h"


/*!class AccelSetup
	\brief This Class provides set of basic implementation of ADXL accelerometer sensor

	This Class is the imoplementation for ADXL sensor.
	Runs an IOCTL on adxl object , checks for any stuck count in axcel sensor \n
	Also contains all low level functions for the sensor - like read register, write register, ADXL setup etc.

*/

#define READ_BIT				0x80
#define START_READ_ADDRESS		0x02  // can only read by starting at this register
#define SINGLE_AXIS_FAIL_COUNT 80

#define MAX_ACCEL_READ_ERROR_COUNT 5

extern short g_x_raw;
//extern CQT_Mode qt_mode;
//extern DEBUG_SETTINGS g_debug_settings;
//extern ERROR_TRACKING g_error_tracking;

// ADXL 362 Registers
#define CMD_WRITE				0x0A
#define CMD_READ				0x0B

#define NOISE_LEVEL_NORMAL		0
#define NOISE_LEVEL_LOW			1
#define NOISE_LEVEL_ULTRA_LOW	2

#define SELF_TEST_READ_COUNT 20

CADXL362Accel::CADXL362Accel(void)
{
	m_active = true;
	m_raw_buffer_index = 0;
	m_sensor_index = 0;
}

void CADXL362Accel::SetDeviceName(std::string name)
{
	m_device_name = name;
}
std::string CADXL362Accel::GetDeviceName(void)
{
	return m_device_name;
}

void CADXL362Accel::SetSensorIndex(int value)
{
	m_sensor_index = value;
}

bool CADXL362Accel::InitializeSPIDevice()
{
	// example m_device_name = "/dev/spidev1.0"
	bool error = m_spi_ctrl.OpenDevice("/dev/spidev2.1");
	//TempCalibrate(20);
	m_calibrated = true;
	m_sensor_fail = false;
	this->m_show_uncalibrated_data = 1;
	m_x_accel_old = 0; 
	m_y_accel_old = 0;
	m_z_accel_old = 0;
	m_accel_read_error_count = 0;

	return error;
}

CADXL362Accel::~CADXL362Accel(void)
{
	
}

void CADXL362Accel::ResetAndMeasure()
{
	WriteRegister(REG_SOFT_RESET, 0x52);		// Write to SOFT RESET, "R"
	usleep(10000);
	WriteRegister(REG_POWER_CONTROL, 2);		// Needed to start measurement
}

void CADXL362Accel::StartSelfTest()
{
	WriteRegister(REG_SELF_TEST, 0x01);
	usleep(10000);	
}

void CADXL362Accel::EndSelfTest()
{
	WriteRegister(REG_SELF_TEST, 0x00);
	usleep(10000);	
}

//! Shows device info from the sensors registers
/*!	
	
*/
void CADXL362Accel::ReadDeviceInfo()
{
	DEVICE_INFO dev_info;
	
	memset(&dev_info, 0, sizeof(dev_info));
	
	dev_info.devid_ad = ReadRegister(0);
	dev_info.devid_mst = ReadRegister(1);
	dev_info.partid = ReadRegister(2);
	dev_info.revid = ReadRegister(3);
	
	//printf("%s devid_id = %d  devid_mst = %d  partid = %d  revid = %d\n",
	m_device_name.c_str(),
	dev_info.devid_ad,
	dev_info.devid_mst,
	dev_info.partid,
	dev_info.revid;
}


float CADXL362Accel::TempCalibrate(float current_room_temp) // temp calibrate is called only once to calibrate at current room temp, thereafter it is not at all required.
{

	m_temp_scale = 1;
	short temp;
	short temp_L = ReadRegister(20);
	short temp_H = ReadRegister(21);
	//uint8_t data = 0;
	//printf("temp_L %d\n",temp_L);
	//printf("temp_H %d\n",temp_H);
	temp_H = (temp_H & 0x0F) << 8;
	temp = temp_H | temp_L;
	temp = ConvertToDecimal(temp);
	//printf("temp %d\n",temp);
	m_temp_scale = current_room_temp / temp;  // scale in degree C/LSB
	// The temperature each time should be the temp*m_temp_scale

	std::ostringstream room_temp;
	room_temp << current_room_temp;

	std::ostringstream temp_scale;
	temp_scale << m_temp_scale;
	std::string Temp_Calibrated = "Calibration of Temp Sensor done at " + room_temp.str() + " °C room temp and resolution " + temp_scale.str() + " °C/LSB";
	//Data_Logging::WriteLog(logINFO,Temp_Calibrated);
	return (m_temp_scale);
		
}


float CADXL362Accel::TempRead() // temp calibrate is called only once to calibrate at current room temp, thereafter it is not at all required.
{

	//Read12BitData(&x,&y,&z,&t);
	float temp;
	short temp_L = ReadRegister(20);
	short temp_H = ReadRegister(21);
	temp_H = (temp_H & 0x0F) << 8;
	temp = temp_H | temp_L;
	temp = ConvertToDecimal(temp);

	temp = temp * m_temp_scale;
	//printf("temp %d\n",temp);
	return (temp);
		
		
}

/****************************************************************************************
/	Reads a single register
/
*/
uint8_t CADXL362Accel::ReadRegister(int reg_index)
{
	int ret;
		
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));
	uint8_t dataBuffer[3];
	uint8_t rxBuffer[3];
	memset(dataBuffer, 0, sizeof(dataBuffer));
	memset(rxBuffer, 0, sizeof(rxBuffer));

	dataBuffer[0] = CMD_READ;
	dataBuffer[1] = reg_index;	// Register
					  
	xfer.tx_buf = (unsigned long)dataBuffer;
	xfer.rx_buf = (unsigned long)rxBuffer;

	xfer.len = sizeof(rxBuffer);
	xfer.delay_usecs = m_spi_ctrl.m_delay; 
	xfer.speed_hz = m_spi_ctrl.m_speed;
	xfer.bits_per_word = 8;
	ret = ioctl(m_spi_ctrl.m_fd, SPI_IOC_MESSAGE(1), &xfer);
  
	// For debugging
	printf("SPI result: %d\n", ret);
	printf("Data: %02X - %02X  - %02X\n", rxBuffer[0], rxBuffer[1], rxBuffer[2]);

	if (ret < 1)
	{
		m_spi_ctrl.Pabort("can't send spi message read");
	}

	return rxBuffer[2];
}

/* 
// Reads the 3 axis 12 bit data from the accelerometer 
// Returns 0 if no error
*/
int CADXL362Accel::AccelRead(short *x_accel, short *y_accel, short *z_accel, bool invert_read)
{
	int ret;
	int n;
		
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));
				
	uint8_t txBuffer[8];	/* read byte, address byte and 6 data bytes (2 bytes for each axis) */
	uint8_t rxBuffer[8];
	memset(txBuffer, 0, sizeof(txBuffer));
	memset(rxBuffer, 0, sizeof(rxBuffer));

	txBuffer[0] = CMD_READ;
	txBuffer[1] = 0x0E;	// 12 bit x accel data register
					  
	xfer.tx_buf = (unsigned long)txBuffer;
	xfer.rx_buf = (unsigned long)rxBuffer;

	xfer.len = sizeof(rxBuffer);
	xfer.delay_usecs = m_spi_ctrl.m_delay; 
	xfer.speed_hz = m_spi_ctrl.m_speed;
	xfer.bits_per_word = 8;
	ret = ioctl(m_spi_ctrl.m_fd, SPI_IOC_MESSAGE(1), &xfer);

	if (ret < 1)
	{
		m_spi_ctrl.Pabort("can't send spi message read");
		return ret;
	}
	short tmpData = 0;

	tmpData = (rxBuffer[3] & 0x0F) << 8;
	*x_accel = tmpData | rxBuffer[2];
	*x_accel = ConvertToDecimal(*x_accel);

	tmpData = (rxBuffer[5] & 0x0F) << 8;
	*y_accel = tmpData | rxBuffer[4];
	*y_accel = ConvertToDecimal(*y_accel);

	tmpData = (rxBuffer[7] & 0x0F) << 8;
	*z_accel = tmpData | rxBuffer[6];
	*z_accel = ConvertToDecimal(*z_accel);

	/*if (g_debug_settings.stuck_value_test.stuck_accel_z == 1 && g_debug_settings.stuck_value_test.stuck_sensor_index == this->m_sensor_index)
	{
		*z_accel = g_debug_settings.stuck_value_test.stuck_value; 
	}*/

	if (this->m_show_uncalibrated_data == 1)
	{
		printf("%s %05d   %05d  %05d \n", m_device_name.c_str(), *x_accel, *y_accel, *z_accel);
	}
		
				
	// code for checking accel-sensor failure.
	// If the value is constant for a long time or the value is 0 for a long time for any axis, it is a sensor failure.
	if (*x_accel == m_x_accel_old)
	{
		m_stuck_x_count++; 
	}
	else
	{
		m_stuck_x_count = 0;
	}

	if (*y_accel == m_y_accel_old)
	{
		m_stuck_y_count++; 
	}
	else
	{
		m_stuck_y_count = 0;
	}

	if (*z_accel == m_z_accel_old)
	{
		m_stuck_z_count++; 
	}
	else
	{
		m_stuck_z_count = 0;
	}

	m_x_accel_old = *x_accel;
	m_y_accel_old = *y_accel;
	m_z_accel_old = *z_accel;

		
		
	raw_accel_buffer[m_raw_buffer_index].x = *x_accel;
	raw_accel_buffer[m_raw_buffer_index].y = *y_accel;
	raw_accel_buffer[m_raw_buffer_index].z = *z_accel;
	m_raw_buffer_index++;
	if (m_raw_buffer_index >= ACCEL_RAW_DATA_BUFFER)
	{
		m_raw_buffer_index = 0;
	}

	bool error = false;
	char error_msg[160];
	int message_len = 160;
	error_msg[0] = '\0';
		
	if (*x_accel < MIN_ACCEL_VALUE || *x_accel > MAX_ACCEL_VALUE)
	{
		AddToErrorMessage(error_msg, "range error - x axis", *x_accel, message_len);
		error = true;
	}

	if (*y_accel < MIN_ACCEL_VALUE || *y_accel > MAX_ACCEL_VALUE)
	{
		AddToErrorMessage(error_msg, "range error - y axis", *y_accel, message_len);
		error = true;
	}

	if (*z_accel < MIN_ACCEL_VALUE || *z_accel > MAX_ACCEL_VALUE) 
	{
		AddToErrorMessage(error_msg, "range error - z axis", *z_accel, message_len);
		error = true;
	}
					
	if (m_stuck_x_count  >= SINGLE_AXIS_FAIL_COUNT)
	{
		AddToErrorMessage(error_msg, "stuck count - x axis", m_stuck_x_count, message_len);
		m_stuck_x_count = 0;
		error = true;
	}
	if (m_stuck_y_count  >= SINGLE_AXIS_FAIL_COUNT)
	{
		AddToErrorMessage(error_msg, "stuck count - y axis", m_stuck_y_count, message_len);
		m_stuck_y_count = 0;
		error = true;
	}
	if (m_stuck_z_count  >= SINGLE_AXIS_FAIL_COUNT)
	{
			
		AddToErrorMessage(error_msg, "stuck count - z axis", m_stuck_z_count, message_len);
		m_stuck_z_count = 0;
		error = true;
	}

	if (error == true)
	{
		
			
		m_accel_read_error_count++;
			
		std::string log_error_msg = "Accel: " + m_device_name + " " + std::string(error_msg);
		cout << log_error_msg << endl;
		//printf(log_error_msg.c_str());	
			
		//Data_Logging::WriteLog(logINFO,log_error_msg);
		//Data_Logging::WriteLog(logINFO,"Accel Self Test called for " + m_device_name);
			
			
		if (m_accel_read_error_count <  MAX_ACCEL_READ_ERROR_COUNT)
		{
			if (SelfTest() == 1)
			{
				std::string sensor_fail_recover = m_device_name + " Sensor recovered after freezing few counts.";
				//Data_Logging::WriteLog(logERROR, sensor_fail_recover);
				m_active = true;
				printf("%s\n", sensor_fail_recover.c_str());
				
			}
			else
			{
				std::string sensor_fail = m_device_name + " Sensor failed due to stuck axis data.";
				//Data_Logging::WriteLog(logERROR, sensor_fail);
				m_sensor_fail = true;
				printf("%s\n", sensor_fail.c_str());
			}	
		}
		else
		{
			log_error_msg = "Accel: " + m_device_name + " disabled due to max accel read errors";
			m_active = false;
			//g_error_tracking.sensor_failure++;
			//Data_Logging::WriteLog(logERROR, log_error_msg);
			printf("%s\n", log_error_msg.c_str());
			//g_debug_settings.stuck_value_test.stuck_accel_z = 0;
		}
			
	}
		

	//if ((m_calibrated == true) && (m_sensor_fail == false ))
	if (m_calibrated == true)
		// this check is required to prevent false alarms and email sending in case of sensor failure 
		// We are assuming the sensor is working as long as any axis is non zero
	{
		//calibrating_done = true;
		//printf(" raw accel %d\n",*z_accel);
		*x_accel -= m_calibration_data.axis[0];
		*y_accel -= m_calibration_data.axis[1];
		*z_accel -= m_calibration_data.axis[2];	
		//printf(" calibratedaccel %d\n",*z_accel);
	}
			
	// must check value range after calibration
	if (*x_accel > MAX_ACCEL_VALUE)
		*x_accel -= MAX_ACCEL_VALUE;
	else if (*x_accel < MIN_ACCEL_VALUE) 
		*x_accel += MAX_ACCEL_VALUE;

	if (*y_accel > MAX_ACCEL_VALUE)
		*y_accel -= MAX_ACCEL_VALUE;
	else if (*y_accel < MIN_ACCEL_VALUE) 
		*y_accel += MAX_ACCEL_VALUE;

	if (*z_accel > MAX_ACCEL_VALUE)
		*z_accel -= MAX_ACCEL_VALUE;
	else if (*z_accel < MIN_ACCEL_VALUE) 
		*z_accel += MAX_ACCEL_VALUE;

	if (invert_read == true)
	{
		*x_accel *= -1;
		*y_accel *= -1;
	}
	return 0;
}


void CADXL362Accel::AddToErrorMessage(char *msg, const char *id_str, int value, int max_message_len)
{
	int add_len = strlen(id_str) + 5 + 5;

	if (strlen(msg) + add_len + 2 > max_message_len)
	{
		return;
	}

	if (strlen(msg) > 0)
	{
		sprintf(&msg[strlen(msg)], ", ");
	}
	sprintf(&msg[strlen(msg)], "%s = %d", id_str, value);
}
	

bool CADXL362Accel::GetSelfTestAverageValues(short *x, short *y, short *z)
{
	short accel_data[NUM_AXIS][SELF_TEST_READ_COUNT];
	short x_raw, y_raw, z_raw;
	int m; 

	long sum_x = 0;
	long sum_y = 0;
	long sum_z = 0;
		
	bool retCode = true;

	for (m = 0; m < SELF_TEST_READ_COUNT; m++)
	{
		AccelUncalibratedRead(&x_raw, &y_raw, &z_raw);
		accel_data[0][m] = x_raw;
		accel_data[1][m] = y_raw;
		accel_data[2][m] = z_raw;
	}
				
	for (m = 0; m < SELF_TEST_READ_COUNT; m++)
	{
		sum_x += (long)accel_data[0][m];
		sum_y += (long)accel_data[1][m];
		sum_z += (long)accel_data[2][m];
	}

	*x = (short)(sum_x / SELF_TEST_READ_COUNT);
	*y = (short)(sum_y / SELF_TEST_READ_COUNT);
	*z = (short)(sum_z / SELF_TEST_READ_COUNT);


	if ((sum_x + sum_y + sum_z) == 0)
	{
		printf("Calibration Failed. Check Sensor\n");
		//Data_Logging::WriteLog(logERROR,"Calibration Failed. Check Sensor");
		retCode = false;
	}
		
	return retCode;
		
}


// Simple read with no error checking or calibration data added
int CADXL362Accel::AccelUncalibratedRead(short *x_accel, short *y_accel, short *z_accel)
{
	int ret;
	int n;
				
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));
				
	uint8_t txBuffer[8];	/* read byte, address byte and 6 data bytes (2 bytes for each axis) */
	uint8_t rxBuffer[8];
	memset(txBuffer, 0, sizeof(txBuffer));
	memset(rxBuffer, 0, sizeof(rxBuffer));

	txBuffer[0] = CMD_READ;
	txBuffer[1] = 0x0E;	// 12 bit x accel data register
					  
	xfer.tx_buf = (unsigned long)txBuffer;
	xfer.rx_buf = (unsigned long)rxBuffer;

	xfer.len = sizeof(rxBuffer);
	xfer.delay_usecs = m_spi_ctrl.m_delay; 
	xfer.speed_hz = m_spi_ctrl.m_speed;
	xfer.bits_per_word = 8;
	ret = ioctl(m_spi_ctrl.m_fd, SPI_IOC_MESSAGE(1), &xfer);

	if (ret < 1)
	{
		m_spi_ctrl.Pabort("can't send spi message read");
		return ret;
	}
	short tmpData = 0;
	tmpData = (rxBuffer[3] & 0x0F) << 8;
	*x_accel = tmpData | rxBuffer[2];
	*x_accel = ConvertToDecimal(*x_accel);

	tmpData = (rxBuffer[5] & 0x0F) << 8;
	*y_accel = tmpData | rxBuffer[4];
	*y_accel = ConvertToDecimal(*y_accel);

	tmpData = (rxBuffer[7] & 0x0F) << 8;
	*z_accel = tmpData | rxBuffer[6];
	*z_accel = ConvertToDecimal(*z_accel);
		
	/*if (g_debug_settings.stuck_value_test.stuck_accel_z == 1 && g_debug_settings.stuck_value_test.stuck_sensor_index == this->m_sensor_index)
	{
		*z_accel = g_debug_settings.stuck_value_test.stuck_value; 
	}*/
			
	// code for checking accel-sensor failure.
	// If the value is constant for a long time or the value is 0 for a long time for any axis, it is a sensor failure.

	//printf("accel  %.5f  %.5f   %.5f\n",(float)*x_accel/1024,(float) *y_accel/1024,(float) *z_accel/1024);
	return 0;
}

void CADXL362Accel::RawBufferSave()
{
	//printf("accel sensor fail , %d , dumping data \n",n);
	std::ostringstream data_buffer("");
	//data_buffer << n;
	data_buffer << "DUMPING DATA FOR: " + GetDeviceName();
	data_buffer << "\n";
		

	for (int i = m_raw_buffer_index; i < ACCEL_RAW_DATA_BUFFER; i++)
	{
		data_buffer << raw_accel_buffer[i].x;
		data_buffer << " ";
		data_buffer << raw_accel_buffer[i].y;
		data_buffer << " ";
		data_buffer << raw_accel_buffer[i].z;
		data_buffer << "\n";
	}
	for (int i = 0; i < m_raw_buffer_index; i++)
	{
		data_buffer << raw_accel_buffer[i].x;
		data_buffer << " ";
		data_buffer << raw_accel_buffer[i].y;
		data_buffer << " ";
		data_buffer << raw_accel_buffer[i].z;
		data_buffer << "\n";
	}
	//Data_Logging::WriteLog(logINFO,data_buffer.str());
}
// Converts 12bit 2's complement to Decimal
short CADXL362Accel::ConvertToDecimal(short data)
{
	short inv_mask = 0xFFF;

	// Accel is 12 bit
	// and is in 2's complement
	// Convert to +-
	if (data & 0x0800)
	{
		data = ~data  & inv_mask;
		data += 1;
		data *= -1;
	}

	return data;
}

void CADXL362Accel::WriteRegister(int reg_index, uint8_t data)
{
	int ret = 0;

	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));
	uint8_t dataBuffer[3];
	uint8_t rxBuffer[3];
	memset(dataBuffer, 0, sizeof(dataBuffer));
	memset(rxBuffer, 0, sizeof(rxBuffer));

		
	dataBuffer[0] = CMD_WRITE;
	dataBuffer[1] = reg_index;	// Register
	dataBuffer[2] = data;		
					  
	xfer.tx_buf = (unsigned long)dataBuffer;
	xfer.rx_buf = (unsigned long)rxBuffer;

	// xfer.len = 4;
	xfer.len = sizeof(rxBuffer);
	xfer.delay_usecs = m_spi_ctrl.m_delay; 
	xfer.speed_hz = m_spi_ctrl.m_speed;
	xfer.bits_per_word = 8;
	ret = ioctl(m_spi_ctrl.m_fd, SPI_IOC_MESSAGE(1), &xfer);
  
	if (ret < 1)
	{
		m_spi_ctrl.Pabort("can't send spi message write");
	}
}

	
/****************************************************************************************
	/ Sets range to +-2g and BW to 100HZ (22 hex)
	/other values HEX: 2E for +-4g and 1500 HZ default
	36 Hex for +-8g and 1500 HZ
	*/

void CADXL362Accel::Initialize()
{
	ResetAndMeasure();
	SetNoiseLevelMode(2);
	SetOutputDataRate(400);
	SetRange(2);
	SetHalfBand(1);
	// to be done
}

/* 
Returns scaler for converting 10 bit to g's.
If range is +-2 g then we are dividing the 10 bit value by 4 g range
so 1 g = 256
*/
int CADXL362Accel::GetScaler()
{
	return this->m_accel_scaler;
}
	
void CADXL362Accel::GetAccelRangeBand(int *p_range, int *p_bandwidth)
{
	// to be done
}

int CADXL362Accel::GetRange()
{
	uint8_t filter_ctl = ReadRegister(REG_FILTER_CONTROL);

	uint8_t range_bits = filter_ctl & 0xC0;
	range_bits = range_bits >> 6;
	int range = 0;

	switch (range_bits)
	{
	case 0:
		range = 2;
		//m_accel_scaler = 1024;
		break;
	case 1:
		range = 4;
		//m_accel_scaler = 512;
		break;
	case 2:
		range = 8;
		//m_accel_scaler = 256;
		break;
	case 3:
		range = 8;
		//m_accel_scaler = 256;
		break;
	}
	return (range);
}


bool CADXL362Accel::SetRange
	(
	short range	// 2 = +- 2g, 4 = +- 4g, 8 = +- 8g
	)
{
	bool error = false;
	uint8_t set_bits = 0;
	switch (range)
	{
	case 2:
		set_bits = 0;
		m_accel_scaler = 1024;
		break;
	case 4:
		set_bits = 1;
		m_accel_scaler = 512;
		break;
	case 8:
		set_bits = 2;
		m_accel_scaler = 256;
		break;
	default:
		printf("Invalid range (%d) passed to CADXL362Accel::SetRange(\n", range);
		error = true;
		break;
	}

	if (error == false)
	{
		set_bits = set_bits << 6;
		WriteRegisterBits(REG_FILTER_CONTROL, 0xC0, set_bits);
	}

	printf("m_accel_scaler = %d, range = %d\n", m_accel_scaler, range);

	usleep(15000); // time for accelerometer internal filter to adjust, values below 5000 (us) can fail
		
	return error;
}

void CADXL362Accel::PrintSetup()
{
	// to be done
	// Show
	// 1. noise level
	// 2. Band width
	// 3. range in g's
	// 4. scaler per g
	// 5. Output data rate
	// 6. Data Measurement ON/OFF

	printf("Device: %s ADXL 362 setup:\n", m_device_name.c_str());

	uint8_t filter_ctl = ReadRegister(REG_FILTER_CONTROL);
	uint8_t range_bits = filter_ctl & 0xC0;
	uint8_t half_BW = filter_ctl & 0x10;
	half_BW = half_BW >> 4;
	range_bits = range_bits >> 6;
	int range = 0;
	float Bandwidth = 0;
	switch (range_bits)
	{
	case 0:
		range = 2;
		m_accel_scaler = 1024;
		break;
	case 1:
		range = 4;
		m_accel_scaler = 512;
		break;
	case 2:
		range = 8;
		m_accel_scaler = 256;
		break;
	case 3:
		range = 8;
		m_accel_scaler = 256;
		break;
	}

	if (range > 0)
	{
		printf("  Range +- %d g's\n", range);
	}
	else
	{
		printf("  Error: Range not detected.\n", range);
	}




	// Output data rate
	uint8_t data_rate_bits = filter_ctl & 0x07;
	float sample_rate = 0;
	switch (data_rate_bits)
	{
	case 0x00:
		sample_rate = 12.5;
		break;
	case 0x01:
		sample_rate = 25;
		break;
	case 0x02:
		sample_rate = 50;
		break;
	case 0x03:
		sample_rate = 100;
		break;
	case 0x04:
		sample_rate = 200;
		break;
	case 0x05:
		sample_rate = 400;
		break;
	case 0x06:
		sample_rate = 400;
		break;
	case 0x07:
		sample_rate = 400;
		break;
	default:
		printf("  Error: Invalid sample rate (%X) read from accelerometer\n", data_rate_bits);
		break;

	}

	if (sample_rate > 0)
	{
			
		if (half_BW)
		{
			Bandwidth = sample_rate / 4;
		}
		else
		{
			Bandwidth = sample_rate / 2;
		}
		printf("  ODR (Output Data Rate) = %.1f Hz\n", sample_rate);
		printf("  Bandwidth = %.2f Hz\n", Bandwidth);
	}
				
	uint8_t pwr_ctl = ReadRegister(REG_POWER_CONTROL);
	uint8_t measure_bits = pwr_ctl & 0x03;
	uint8_t noise_bits = pwr_ctl & 0x30;
	noise_bits = noise_bits >> 4;
	char noise_mode[50]; 
	char measurement_mode[50];

	switch (noise_bits)
	{
	case 0x00:
		strcpy(noise_mode, "Normal Operation");
		break;
	case 0x01:
		strcpy(noise_mode, "Low Noise Mode");
		break;
	case 0x02:
		strcpy(noise_mode, "Ultra Low Noise Mode");
		break;
	default:
		printf("  Error: Invalid noise mode (%X) read from accelerometer\n", noise_bits);
		break;
	}

	if (noise_mode != NULL)
	{
		printf("  Noise Levels %s \n", noise_mode);
	}
		

	switch (measure_bits)
	{
	case 0x00:
		strcpy(measurement_mode, "Standby");
		break;
	case 0x01:
		strcpy(measurement_mode, "Reserved");
		break;
	case 0x02:
		strcpy(measurement_mode, "ON Measurement");
		break;
	default:
		printf("  Error: Invalid measurement mode (%X) read from accelerometer\n", measure_bits);
		break;
	}
						
	if (noise_mode != NULL)
	{
		printf(" Measurement Mode %s \n", measurement_mode);
	}

}



// Sets the noise level. Lower noise uses more power
void CADXL362Accel::SetNoiseLevelMode
	(
	short level	// 00 = normal, 1 = low noise, 2 = ultralow noise
	)
{
	level = level << 4;
	WriteRegisterBits(REG_POWER_CONTROL, 0x30, level);
}


void CADXL362Accel::SetHalfBand(short band_half)
{
	band_half = band_half << 4;
	WriteRegisterBits(REG_FILTER_CONTROL, 0x10, band_half);
}


bool CADXL362Accel::WriteRegisterBits
	(
	int reg_index, 
	uint8_t set_mask, 
	uint8_t data)	// bits must already be shifted to match mask)
{

	uint8_t clear_mask = ~set_mask;

	uint8_t pvs_value = ReadRegister(reg_index);
	uint8_t new_value = (pvs_value & clear_mask) | (data & set_mask);
	WriteRegister(reg_index, new_value);
}

	

bool CADXL362Accel::SetOutputDataRate(short rate_hertz)
{

	bool error = false;
	uint8_t set_bits = 0;
	switch (rate_hertz)
	{
	case 25:
		set_bits = 1;
		break;
	case 50:
		set_bits = 2;
		break;
	case 100:
		set_bits = 3;
		break;
	case 200:
		set_bits = 4;
		break;
	case 400:
		set_bits = 5;
		break;
	default:
		printf("Invalid rate_hertz (%d) passed to CADXL362Accel::SetOutputDataRate(\n", rate_hertz);
		error = true;
		break;
	}

	if (error == false)
	{
		WriteRegisterBits(REG_FILTER_CONTROL, 0x07, set_bits);
	}
		
	return error;

}

void CADXL362Accel::SetCalibrated(bool value)
{
	m_calibrated = value;
}

	
bool CADXL362Accel::GetSensorFail()
{
	return m_sensor_fail;
}


void CADXL362Accel::ResetSensorFail()
{
	m_sensor_fail = false;
}

	
bool CADXL362Accel::AccelCalibrate()
{
	//RefClock Calib_time_check;

	//Calib_time_check.StartRefClock();
		
	short accel_data[NUM_AXIS][CALIBRATE_COUNT];
	short x, y, z, t;
	int m; 
	std::ostringstream x_s, y_s, z_s;
	int accel_scaler = this->GetScaler();
		
	//Data_Logging::WriteLog(logINFO,"Old "+ log_calibration);

	usleep(200000);

	for (m = 0; m < CALIBRATE_COUNT; m++)
	{
		AccelCalibrationRead(&x, &y, &z);
			
		for (int i = 0; i < 500000; i++)
		{
			
		}

		accel_data[0][m] = x;
		accel_data[1][m] = y;
		accel_data[2][m] = z;
	}

	long sum_x = 0;
	long sum_y = 0;
	long sum_z = 0;

	int count = 0;

	for (m = 0; m < CALIBRATE_COUNT; m++)
	{
		sum_x += accel_data[0][m];
		sum_y += accel_data[1][m];
		sum_z += accel_data[2][m];
	}
				
	m_calibration_data.axis[0] = (short)(sum_x / CALIBRATE_COUNT);
	m_calibration_data.axis[1] = (short)(sum_y / CALIBRATE_COUNT);
	m_calibration_data.axis[2] = (short)(sum_z / CALIBRATE_COUNT);


	if ((sum_x + sum_y + sum_z) == 0)
	{
		printf("Calibration Failed. Check Sensor\n");
		//Data_Logging::WriteLog(logERROR,"Calibration Failed. Check Sensor");
		return (false);
	}
		
	x_s << (float)m_calibration_data.axis[0] / accel_scaler;
	y_s << (float)m_calibration_data.axis[1] / accel_scaler;
	z_s << (float)m_calibration_data.axis[2] / accel_scaler;
	string log_calibration = m_device_name + " Calibration Data X_Axis:" + x_s.str() + " Y_Axis:" + y_s.str() + " Z_Axis:" + z_s.str();

	//Data_Logging::WriteLog(logINFO,log_calibration);
				
	printf("Calibration values %s (g's): x = %.3f, y = %.3f, z = %.3f\n",
		m_device_name.c_str(),
		(float)m_calibration_data.axis[0]/accel_scaler,
		(float)m_calibration_data.axis[1] / accel_scaler,
		(float)m_calibration_data.axis[2]/accel_scaler);
		
	

	//Data_Logging::WriteLog(logINFO,"Accel Sensor Calibrated");

	return true;
}

int CADXL362Accel::AccelCalibrationRead(short *x_accel, short *y_accel, short *z_accel)
{
	int ret;
	int n;
		
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer));
				
	uint8_t txBuffer[8];	/* read byte, address byte and 6 data bytes (2 bytes for each axis) */
	uint8_t rxBuffer[8];
	memset(txBuffer, 0, sizeof(txBuffer));
	memset(rxBuffer, 0, sizeof(rxBuffer));

	txBuffer[0] = CMD_READ;
	txBuffer[1] = 0x0E;	// 12 bit x accel data register
					  
	xfer.tx_buf = (unsigned long)txBuffer;
	xfer.rx_buf = (unsigned long)rxBuffer;

	xfer.len = sizeof(rxBuffer);
	xfer.delay_usecs = m_spi_ctrl.m_delay; 
	xfer.speed_hz = m_spi_ctrl.m_speed;
	xfer.bits_per_word = 8;
	ret = ioctl(m_spi_ctrl.m_fd, SPI_IOC_MESSAGE(1), &xfer);

	if (ret < 1)
	{
		m_spi_ctrl.Pabort("can't send spi message read");
		return ret;
	}
	short tmpData = 0;
	tmpData = (rxBuffer[3] & 0x0F) << 8;
	*x_accel = tmpData | rxBuffer[2];
	*x_accel = ConvertToDecimal(*x_accel);

	tmpData = (rxBuffer[5] & 0x0F) << 8;
	*y_accel = tmpData | rxBuffer[4];
	*y_accel = ConvertToDecimal(*y_accel);

	tmpData = (rxBuffer[7] & 0x0F) << 8;
	*z_accel = tmpData | rxBuffer[6];
	*z_accel = ConvertToDecimal(*z_accel);

	short value = *x_accel;
	if (value > 2048)
	{ 
		value -= 2048;
	}
	else if (value < -2048)
	{
		value += 2048;
	}
	*x_accel = value;


	value = *y_accel;
	if (value > 2048)
	{ 
		value -= 2048;
	}
	else if (value < -2048)
	{
		value += 2048;
	}
	*y_accel = value;


	value = *z_accel;
	if (value > 2048)
	{ 
		value -= 2048;
	}
	else if (value < -2048)
	{
		value += 2048;
	}
	*z_accel = value;
}	/* end of AccelCalibrationRead() */

int CADXL362Accel::GetVerticalAxis()
{
	int scaler = this->GetScaler();

	float min_one_g = scaler * 0.8;

	if (abs(this->m_calibration_data.axis[0]) > min_one_g)
	{
		printf("  X-axis is vertical axis\n");
		return 0;
	}
	if (abs(this->m_calibration_data.axis[1]) > min_one_g)
	{
		printf("  Y-axis is vertical axis\n");
		return 1;
	}
	if (abs(this->m_calibration_data.axis[2]) > min_one_g)
	{
		printf("  Z-axis is vertical axis\n");
		return 2;
	}

	return -1;

}	/* end of GetVerticalAxis() */


/*! \fn void SelfTest()
	\brief Sets the sensor chip to self test mode and runs a self test
	
	Returns 1 if successful
*/
int CADXL362Accel::SelfTest()
{
	int ret_code = 0;

			//CQT_Mode::m_sub_modes.accel_self_test_running = true;
	usleep(100000);

	SetRange(4);
		
	if (GetRange() == 4)
	{
		if (Accel_sensor_check() == 0)
		{
			printf("Accel Self Test Passed \n");
			//Data_Logging::WriteLog(logINFO,"Accel Sensor Self test passed");
			ret_code = 1;
		}
		else 
		{
			printf("Sensor fail due to self test.\n");
			//Data_Logging::WriteLog(logERROR,"Sensor fail due to self test.");
			ret_code = 2;
		}
	}
	else
	{
		printf("unable to set range for accel self test \n");
	}
	
	SetRange(2);

	usleep(100000);
	//CQT_Mode::m_sub_modes.accel_self_test_running = false;

	return ret_code;
}	/* end of SelfTest() */

/****************************************************************************************/
// Returns 0 if no error
//
/****************************************************************************************/
int CADXL362Accel::Accel_sensor_check()
{
	
	short x_base, y_base, z_base;
	short x_test, y_test, z_test;
	float x_diff, y_diff, z_diff;

	short accel_scaler = GetScaler();
		
	GetSelfTestAverageValues(&x_base, &y_base, &z_base);
	StartSelfTest();
	
	//int temp = g_debug_settings.stuck_value_test.stuck_accel_z;
	//g_debug_settings.stuck_value_test.stuck_accel_z = 0;

	usleep(1000);
	
	GetSelfTestAverageValues(&x_test, &y_test, &z_test);
	
	//g_debug_settings.stuck_value_test.stuck_accel_z = temp;
	
	x_diff = (float)(x_test - x_base) / (float)accel_scaler * 1000;
	y_diff = (float)(y_base - y_test) / (float)accel_scaler * 1000;			// self test applies a negative force to the y axis only
	z_diff = (float)(z_test - z_base) / (float)accel_scaler * 1000;

	//printf("%d \n",(x_test - x_base));
	//printf("%d \n", accel_scaler);

	printf("Base       %4d %4d %4d\n", x_base, y_base, z_base);
	printf("Test       %4d %4d %4d\n", x_test, y_test, z_test);
	printf("Diff (mg's) %.0f %.0f %.0f\n", x_diff, y_diff, z_diff);
	printf("Max allowed x y z  = %.0f %.0f %.0f\n ", MaxAllowedAccel(0), MaxAllowedAccel(1), MaxAllowedAccel(2));
	if ((x_diff > MaxAllowedAccel(0)) || (y_diff > MaxAllowedAccel(1)) || (z_diff > MaxAllowedAccel(2)))
	{
		//sensor problem
		printf("Self test sensor max has failed \n ");
		//printf("Base       %4d %4d %4d\n",x_base, y_base, z_base);
		//printf("Test       %4d %4d %4d\n",x_test, y_test, z_test);
		printf("Diff (mg's) %.0f %.0f %.0f\n", x_diff, y_diff, z_diff);
		printf("Max allowed x y z  = %.0f %.0f %.0f\n ", MaxAllowedAccel(0), MaxAllowedAccel(1), MaxAllowedAccel(2));
		return (-1);
	}

	printf("Base       %4d %4d %4d\n", x_base, y_base, z_base);
	printf("Test       %4d %4d %4d\n", x_test, y_test, z_test);
	printf("Diff (mg's) %.0f %.0f %.0f\n", x_diff, y_diff, z_diff);
	printf("Min allowed x y z  = %.0f %.0f %.0f\n ", MinAllowedAccel(0), MinAllowedAccel(1), MinAllowedAccel(2));
	if ((x_diff < MinAllowedAccel(0)) ||  (y_diff < MinAllowedAccel(1)) || (z_diff < MinAllowedAccel(2)))
	{
		//sensor problem
		printf("Self test sensor min has failed\n ");
		//printf("Base       %4d %4d %4d\n",x_base, y_base, z_base);
		//printf("Test       %4d %4d %4d\n",x_test, y_test, z_test);
		printf("Diff (mg's) %.0f %.0f %.0f\n", x_diff, y_diff, z_diff);
		printf("Min allowed x y z  = %.0f %.0f %.0f\n ", MinAllowedAccel(0), MinAllowedAccel(1), MinAllowedAccel(2));
		return (-1);
	}

	
	EndSelfTest();
	return (0);

}




float CADXL362Accel::MaxAllowedAccel(int axis)
{
	// See Table 1 SELF TEST section of ADXL362.pdf for the 710, 650 and 450 values
	
	float Self_Test_cushion = 1.10;
	if (axis == 0)
	{
		return (3 * 710 * Self_Test_cushion);
	}
	else if (axis == 1)
	{
		return (3 * 710 * Self_Test_cushion);
	}
	else if (axis == 2)
	{
		return (3 * 650 * Self_Test_cushion);
	}

}
float CADXL362Accel::MinAllowedAccel(int axis)
{
	float Self_Test_cushion = 0.75;
	if (axis == 0)
	{
		return (3 * 450 * Self_Test_cushion);
	}
	else if (axis == 1)
	{
		return (3 * 450 * Self_Test_cushion);
	}
	else if (axis == 2)
	{
		return (3 * 350 * Self_Test_cushion);
	}
}