#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

#include <errno.h>
#include <wiringPi.h>

/******************************************************************************
 * Local Macro Declarations                                                    * 
 ******************************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define SX1257_WRITE_BIT                0x80
#define SX1257_READ_BIT                 0x00

#define SX1257_PATH 	"/dev/spidev0.0"

// sx1257 SPI
static uint32_t mode = 0;
static uint8_t bits = 8;
static uint32_t speed = 7800000;
static uint16_t delay_usecs = 10;

static int fd, fpga_fd;

static uint8_t buf[2048];

typedef struct sx1257_registerSetting {
  uint8_t addr;
  uint8_t val;
} registerSetting_t;

static const registerSetting_t preferredSettings[]=
{
	{0x00, 0x01},//	RegMode
	{0x01, 0xCD},//	RegFrfRxMsb
	{0x02, 0x8E},//	RegFrfRxMid
	{0x03, 0x39},//	RegFrfRxLsb
	{0x04, 0xCD},//	RegFrfTxMsb
	{0x05, 0x8E},//	RegFrfTxMid
	{0x06, 0x39},//	RegFrfTxLsb
	//{0x07, 0x21},//	RegVersion
	{0x08, 0x2E},//	RegTxGain
	//{0x09, 0x1F},//	RegTxMixerTank
	{0x0A, 0x60},//	RegTxBw
	{0x0B, 0x02},//	RegTxDacBw
	{0x0C, 0x3F},//	RegRxAnaGain
	{0x0D, 0xFD},//	RegRxBw
	{0x0E, 0x06},//	RegRxPLLBw
	{0x0F, 0x00},//	RegDioMapping
	{0x10, 0x02},//	RegClkSelect
	//{0x11, 0x06},//	RegModeStatus
	{0x1A, 0x02},//	RegLowBatThres
	//{0x19, 0xAD},//	RegTestPdsTrim
};

#define FPGA_PATH 		"/dev/spidev0.1"

// Use GPIO Pin 25, which is Pin 6 for wiringPi library
#define IRQ 6
#define RST 5
 
// the event counter 
volatile int eventCounter = 0;
volatile uint8_t ready = 0;

#define numSamples 1064
uint8_t iq[numSamples] = {0,};

int sx1257_write_register(uint8_t reg, uint8_t value)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay_usecs,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// Reg
	buf[transfer.len++] = SX1257_WRITE_BIT | reg;
	buf[transfer.len++] = value;

	// send the SPI message (all of the above fields, inc. buffers)
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	
	return ret;
}

void sx1257_write_reg_settings(const registerSetting_t *reg_settings,
		uint16_t sizeof_reg_settings)
{
	int i = sizeof_reg_settings / sizeof(registerSetting_t);

	if(reg_settings != NULL) {
		while(i--) {
			sx1257_write_register(reg_settings->addr,
					reg_settings->val);
			reg_settings++;
		}
	}
}

int sx1257_read_register(uint8_t reg, uint8_t *data)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay_usecs,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// Reg
	buf[transfer.len++] = SX1257_READ_BIT | reg;
	buf[transfer.len++] = 0xa5;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
		*data = buf[transfer.len-1];

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}

int sx1257_init(char * spi_path)
{
	// The following calls set up the sx1257 SPI bus properties
	if((fd = open(spi_path, O_RDWR))<0){
		perror("SPI Error: Can't open device.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't set bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

	// Check that the properties have been set
	// printf("SPI: %s\n", spi_path);
	// printf("SPI Mode is: %d\n", mode);
	// printf("SPI Bits is: %d\n", bits);
	// printf("SPI Speed is: %d\n", speed);

	return 0;
}

int fpga_init(char * spi_path)
{
	// The following calls set up the sx1257 SPI bus properties
	if((fpga_fd = open(spi_path, O_RDWR))<0){
		perror("SPI Error: Can't open device.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_WR_MODE, &mode)==-1){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_RD_MODE, &mode)==-1){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't set bits per word.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fpga_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

	// Check that the properties have been set
	// printf("SPI: %s\n", spi_path);
	// printf("SPI Mode is: %d\n", mode);
	// printf("SPI Bits is: %d\n", bits);
	// printf("SPI Speed is: %d\n", speed);

	return 0;
}

static void hex_dump(const void *src, size_t length, size_t line_size,
		     char *prefix)
{
	int i = 0;
	const unsigned char *address = src;
	const unsigned char *line = address;
	unsigned char c;

	printf("%s | ", prefix);
	while (length-- > 0) {
		printf("%02X ", *address++);
		//printf("%d ", *address++);
		if (!(++i % line_size) || (length == 0 && i % line_size)) {
			if (length == 0) {
				while (i++ % line_size)
					printf("__ ");
			}
			printf(" | ");  /* right close */
			while (line < address) {
				c = *line++;
				printf("%c", (c < 33 || c == 255) ? 0x2E : c);
			}
			printf("\n");
			if (length > 0)
				printf("%s | ", prefix);
		}
	}
}

int
fpga_read(uint8_t *data, uint16_t len)
{
	int ret;
	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)buf,
		.len = 0,
		.delay_usecs = delay_usecs,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	
	transfer.len += len;

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(ret < 0)
		return ret;
	else
	{
		int j;
		for (j = 0; j < transfer.len-7; j++)
		{
			data[j] = buf[j+7];
		}
	}

	// send the SPI message (all of the above fields, inc. buffers)
	return ret;
}

void myInterrupt(void) {
   	eventCounter++;
   	ready = 1;
   	fpga_read(iq, numSamples);
}

int main(int argc, char* argv[]){

	sx1257_init(SX1257_PATH);
	sx1257_write_reg_settings(preferredSettings, sizeof(preferredSettings));
	
	uint8_t value = 0;
	sx1257_read_register(0x07, &value);
	// printf("Chip Version: %02x\n", value);

	sx1257_read_register(0x01, &value);
	// printf("RegFrfRxMsb: %02x\n", value);

	sx1257_read_register(0x02, &value);
	// printf("RegFrfRxMid: %02x\n", value);

	sx1257_read_register(0x03, &value);
	// printf("RegFrfRxLsb: %02x\n", value);

	sx1257_read_register(0x0C, &value);
	// printf("RegRxAnaGain: %02x\n", value);

	sx1257_read_register(0x0D, &value);
	// printf("RegRxBw: %02x\n", value);

	sx1257_read_register(0x0E, &value);
	// printf("RegRxPLLBw: %02x\n", value);

	sx1257_read_register(0x0F, &value);
	// printf("RegDioMapping: %02x\n", value);
	
	sx1257_read_register(0x10, &value);
	// printf("RegClkSelect: %02x\n", value);
	
	sx1257_read_register(0x11, &value);
	// printf("RegModeStatus: %02x\n", value);
	
	sx1257_read_register(0x1A, &value);
	// printf("RegLowBatThres: %02x\n", value);
	
	sx1257_read_register(0x19, &value);
	// printf("RegTestPdsTrim: %02x\n", value);
	
	sx1257_write_register(0x00, 0x03);

	close(fd);

	speed = 15600000;
	//speed = 31200000;
	
	fpga_init(FPGA_PATH);

	  // sets up the wiringPi library
	if (wiringPiSetup () < 0) {
		fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));
		return 1;
	}

	pinMode(RST, OUTPUT);
	digitalWrite(RST, HIGH);
	usleep(10000);
	digitalWrite(RST, LOW);

  	// set Pin 25/0 generate an interrupt on high-to-low transitions
  	// and attach myInterrupt() to the interrupt
	if ( wiringPiISR (IRQ, INT_EDGE_RISING, &myInterrupt) < 0 ) {
		fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
		return 1;
	}

	// display counter value every second.
	int j = 0;
	while(j < 5000)
	{
		if(ready){
			ready = 0;
        	int i;
        	for(i = 0; i < numSamples-7; i++)
        		if((i < 1000) || (iq[i] < 64))
            		printf("%d,", iq[i]);
        	//printf("\n");
        	j++;
		}
        //usleep(1000);
	}

	return 0;
}
