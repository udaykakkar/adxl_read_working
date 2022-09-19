
#include "SPI_Control.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>	// perror()
#include <stdlib.h>  // for about()
#include <getopt.h>  // for static const struct option lopts[];

uint32_t m_mode;

//static const char *device = "/dev/spidev2.2";  //  spidev2.0 = first device on UEXT 1
static const char *device = "/dev/spidev2.0"; 
static int verbose;
char *input_tx;


CSPI_Control::CSPI_Control()
{
	m_bits = 8;
	m_speed = 400000;
	m_delay = 0;
}

bool CSPI_Control::OpenDevice(const char *p_spi_device_name)
{
	m_fd = open(p_spi_device_name, O_RDWR);
	if (m_fd >= 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}


CSPI_Control::~CSPI_Control(void)
{
	close(m_fd);
}

void CSPI_Control::Pabort(const char *s)
{
	perror(s);
	abort();
}

int CSPI_Control::GetFD()
{
	return m_fd;
}

void CSPI_Control::SetMode(uint32_t mode_bits)
{
	//m_mode |= SPI_LSB_FIRST;
	m_mode |= mode_bits;
}

/****************************************************************************************
/
/
*/

int CSPI_Control::InitSpi()
{
	int ret;

	ret = ioctl(m_fd, SPI_IOC_WR_MODE, &m_mode);
	if (ret == -1)
		Pabort("can't set spi mode");

	ret = ioctl(m_fd, SPI_IOC_RD_MODE, &m_mode);
	if (ret == -1)
		Pabort("can't get spi mode");


	/*
	 * bits per word
	 */
	ret = ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &m_bits);
	if (ret == -1)
		Pabort("can't set bits per word");

	ret = ioctl(m_fd, SPI_IOC_RD_BITS_PER_WORD, &m_bits);
	if (ret == -1)
		Pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &m_speed);
	if (ret == -1)
		Pabort("can't set max speed hz");

	ret = ioctl(m_fd, SPI_IOC_RD_MAX_SPEED_HZ, &m_speed);
	if (ret == -1)
		Pabort("can't get max speed hz");

	

}

void CSPI_Control::PrintSetup()
{
	printf("SPI setup to accelerometer\n");
	printf("  spi mode: 0x%x\n", m_mode);
	printf("  bits per word: %d\n", m_bits);
	printf("  SPI speed: %d Hz (%d KHz)\n", m_speed, m_speed / 1000);
	printf("\n");
}

/****************************************************************************************
/
/
*/

void CSPI_Control::ParseOpts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device", 1, 0, 'D' },
			{ "speed", 1, 0, 's' },
			{ "delay", 1, 0, 'd' },
			{ "bpw", 1, 0, 'b' },
			{ "loop", 0, 0, 'l' },
			{ "cpha", 0, 0, 'H' },
			{ "cpol", 0, 0, 'O' },
			{ "lsb", 0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire", 0, 0, '3' },
			{ "no-cs", 0, 0, 'N' },
			{ "ready", 0, 0, 'R' },
			{ "dual", 0, 0, '2' },
			{ "verbose", 0, 0, 'v' },
			{ "quad", 0, 0, '4' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR24p:v", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			m_speed = atoi(optarg);
			break;
		case 'd':
			m_delay = atoi(optarg);
			break;
		case 'b':
			m_bits = atoi(optarg);
			break;
		case 'l':
			m_mode |= SPI_LOOP;
			break;
		case 'H':
			m_mode |= SPI_CPHA;
			break;
		case 'O':
			m_mode |= SPI_CPOL;
			break;
		case 'L':
			m_mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			m_mode |= SPI_CS_HIGH;
			break;
		case '3':
			m_mode |= SPI_3WIRE;
			break;
		case 'N':
			m_mode |= SPI_NO_CS;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'R':
			m_mode |= SPI_READY;
			break;
		case 'p':
			input_tx = optarg;
			break;
#if(UNDEFINED)
		case '2':
			m_mode |= SPI_TX_DUAL;
			break;\
		case '4':
			m_mode |= SPI_TX_QUAD;
			break;
#endif
		default:
			//print_usage(argv[0]);
			break;
		}
	}
#if(UNDEFINED)
	if (m_mode & SPI_LOOP) 
	{
		if (m_mode & SPI_TX_DUAL)
		{
			m_mode |= SPI_RX_DUAL;
		}
		if (m_mode & SPI_TX_QUAD)
		{
			m_mode |= SPI_RX_QUAD;
		}
	}
#endif
}

void CSPI_Control::PrintUsage(const char *prog)
{

	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n"
	     "  -v --verbose  Verbose (show tx buffer)\n"
	     "  -p            Send data (e.g. \"1234\\xde\\xad\")\n"
	     "  -N --no-cs    no chip select\n"
	     "  -R --ready    slave pulls low to pause\n"
	     "  -2 --dual     dual transfer\n"
	     "  -4 --quad     quad transfer\n");

}
