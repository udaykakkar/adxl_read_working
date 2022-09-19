#pragma once
#pragma once

#include <stdint.h>
#include <unistd.h>
#include "SPI_Control.h"
#include "Calibrate.h"


using namespace std;
#include <string>	

#define  ERROR_NO_ERROR				0
#define ERROR_NO_ACCEL_DATA			1
#define MAX_ACCEL_VALUE 2048
#define MIN_ACCEL_VALUE -2048
#define ACCEL_RAW_DATA_BUFFER 3*100 // 3 second buffer

#define DEFAULT_SCALER 1024
#define EARTHQUAKE_WAITING_TIME 30  // 5 minutes of earthquake waiting time

// See the Register map in doc\adxl326.pdf
struct DEVICE_INFO
{
	uint8_t devid_ad;
	uint8_t devid_mst;
	uint8_t partid;
	uint8_t revid;
};

struct QT_RAW_ACCEL_BUFFER
{
	short x; 
	short y;
	short z;
};

class CADXL362Accel
{

public:

	static const short REG_SOFT_RESET = 0x1F;
	static const short REG_INT_MAP1 = 0x2A;
	static const short REG_INT_MAP2 = 0x2B;
	static const short REG_FILTER_CONTROL = 0x2C;
	static const short  REG_POWER_CONTROL =	0x2D;
	static const short  REG_SELF_TEST =	0x2E;
		
	CADXL362Accel(void);
	~CADXL362Accel(void);
	void ParseOpts(int argc, char *argv[]);
	
	
	// New ADXL 362 functions
	//void InitializeSPIDevice(const char *device);
	bool InitializeSPIDevice();
	void Initialize();
	void SetDeviceName(std::string name);
	void SetSensorIndex(int index);
	std::string GetDeviceName(void);
	void AddToErrorMessage(char *msg, const char *id_str, int value, int max_message_len);
	
	int AccelRead(short *x_accel, short *y_accel, short *z_accel, bool invert_raw_read);
	float TempCalibrate(float current_room_temp);
	float TempRead();
	void RawBufferSave();
	
	void SetRangeBand();
	void SetHalfBand(short  band_half);
	void PrintSetup();
	int  GetRange();
	void ResetSensorFail();
	int GetScaler();

	// Self Testing functions
	int SelfTest();
		
	// Calibration functions
	void SetCalibrated(bool value);
	bool AccelCalibrate();
	int GetVerticalAxis();
	bool GetSensorFail();
	void ReadDeviceInfo();

// Public properties
	
	int m_accel_scaler;
	bool m_active;
	CSPI_Control m_spi_ctrl;
	int m_show_uncalibrated_data;
	bool m_sensor_fail;
	
	bool m_calibrated;
	std::string m_device_name;
	QT_RAW_ACCEL_BUFFER raw_accel_buffer[ACCEL_RAW_DATA_BUFFER];
	

private:
	
	// Methods
	void StartSelfTest();
	void EndSelfTest();
	int SetScaler();
	uint8_t ReadRegister(int reg_index);
	void WriteRegister(int reg_index, uint8_t data);
	short ConvertToDecimal(short data);
	void SetNoiseLevelMode(short noise_level);
	bool SetOutputDataRate(short rate_hertz);
	bool WriteRegisterBits(int reg_index, uint8_t mask, uint8_t data);

	bool SetRange(short range);
	int InitSpi();
	void ResetAndMeasure();
	int AccelCalibrationRead(short *x_accel, short *y_accel, short *z_accel);
	void GetAccelRangeBand(int *p_range, int *p_bandwidth);
	
	// Sensor Self Test
	int Accel_sensor_check();
	float MaxAllowedAccel(int axis);
	float MinAllowedAccel(int axis);
	bool GetSelfTestAverageValues(short *x, short *y, short *z);
	int AccelUncalibratedRead(short *x_accel, short *y_accel, short *z_accel);
	
	
	
	void ZeroCrossCheck(int *x_accel, int *y_accel, int *z_accel);
	int m_stuck_x_count;
	int m_stuck_y_count;
	int m_stuck_z_count;
	int m_x_zero_cross_count;
	int m_y_zero_cross_count;
	int m_z_zero_cross_count;

	void Pabort(const char *s);

	// properties
	int m_sensor_index;
	float m_temp_scale;
	
	
	short m_x_accel_old;
	short m_y_accel_old;
	short m_z_accel_old;

	short m_x_accel_old_calib;
	short m_y_accel_old_calib;
	short m_z_accel_old_calib;
	int m_raw_buffer_index;
	
	
	
	CALIBRATION_DATA m_calibration_data;
	
	int m_accel_read_error_count;
	

};