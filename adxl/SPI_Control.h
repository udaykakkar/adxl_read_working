#pragma once
#pragma once

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>

class CSPI_Control
{

public:
	CSPI_Control();
	~CSPI_Control(void);
	bool OpenDevice(const char *p_spi_device_name);
	void ParseOpts(int argc, char *argv[]);
	void Pabort(const char *s);
	int InitSpi();
	int GetFD();
	uint16_t m_delay;
	uint32_t m_speed;	// 10000 us = 100 Hz
	uint8_t m_bits;
	int m_fd;

	void SetMode(uint32_t mode_bits);

	void PrintUsage(const char *prog);
	void PrintSetup();

private:
	
	

};