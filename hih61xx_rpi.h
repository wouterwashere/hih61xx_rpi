#ifndef HIH61XX_RPI_H
#define HIH61XX_RPI_H

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <math.h>
#include "smbus.h"

#define BMP085_I2C_ADDRESS 0x77

/* Declare variables */
const short BMP180_OVERSAMPLING_SETTING = 3;
short ac1, ac2, ac3, b1, b2, mb, mc, md, height;
unsigned short ac4, ac5, ac6;
long ut, up, x1, x2, b5, b6, t, x3, b3, x1, x2, x3, p, temperature, pressure, pressure_at_sea_level;
unsigned long b4, b7;

/* Connect to BMP180 */
int bmp180_open()
{
	/* Open device */
	int fd;
	if((fd = open("/dev/i2c-1", O_RDWR))<0)
		exit(1);

	/* Set options and address */
	if(ioctl(fd, I2C_SLAVE, BMP085_I2C_ADDRESS)<0)
	{
		close(fd);
		exit(1);
	}
	return fd;
}

/* Read two words from the BMP180 and supply it as a 16 bit integer, MSB first */
unsigned short bmp180_read_16bit(int fd, unsigned char address)
{
	unsigned long res = i2c_smbus_read_word_data(fd, address);
	if (res < 0)
	{
		close(fd);
		exit(1);
	}

	/* Convert result to 16 bits and swap bytes */
	res = ((res << 8) & 0xFF00) | ((res >> 8) & 0xFF);
	return res;
}

/* Read out EEPROM registers, 16 bit */
void bmp180_get_cal_param()
{
	/* Connect to BMP180 */
	int fd = bmp180_open();

	/* Read out registers */
	ac1 = bmp180_read_16bit(fd, 0xAA);
	ac2 = bmp180_read_16bit(fd, 0xAC);
	ac3 = bmp180_read_16bit(fd, 0xAE);
	ac4 = bmp180_read_16bit(fd, 0xB0);
	ac5 = bmp180_read_16bit(fd, 0xB2);
	ac6 = bmp180_read_16bit(fd, 0xB4);
	b1 = bmp180_read_16bit(fd, 0xB6);
	b2 = bmp180_read_16bit(fd, 0xB8);
	mb = bmp180_read_16bit(fd, 0xBA);
	mc = bmp180_read_16bit(fd, 0xBC);
	md = bmp180_read_16bit(fd, 0xBE);
	close(fd);
}

/* Write a byte to the BMP085 */
void bmp180_write(int fd, unsigned char address, unsigned char value)
{
	if (i2c_smbus_write_byte_data(fd, address, value) < 0)
	{
		close(fd);
		exit(1);
	}
}

/* Read uncompensated temperature value */
long bmp180_get_ut()
{
	ut = 0;
	int fd = bmp180_open();

	/* Write 0x2E into Register 0xF4 to request a temperature reading */
	bmp180_write(fd, 0xF4, 0x2E);

	/* Wait 4.5ms */
	usleep(4500);

	/* Read register 0xF6, 16 bit */
	ut = bmp180_read_16bit(fd, 0xF6);

	/* Close the i2c file */
	close (fd);

	return ut;
}

/* Read a block */
void bmp180_read_block(int fd, unsigned char address, unsigned char length, unsigned char *values)
{
	if(i2c_smbus_read_i2c_block_data(fd, address, length, values) < 0)
	{
		close(fd);
		exit(1);
	}
}

/* Read uncompensated pressure value */
long bmp180_get_up()
{
	up = 0;
	int fd = bmp180_open();

	/* Write 0x34 + (BMP180_OVERSAMPLING_SETTING << 6) into register 0xF4 */
	bmp180_write(fd, 0xF4, 0x34 + (BMP180_OVERSAMPLING_SETTING << 6));

	/* Wait for conversion */
	usleep(4.5 + (BMP180_OVERSAMPLING_SETTING * 7) * 1000);

	/* Read register 0xF6 (MSB), 0xF7 (LSB), 0xF8 (XLSB) */
	unsigned char values[3];
	bmp180_read_block(fd, 0xF6, 3, values);

	/* Sort values */
	up = (((unsigned int) values[0] << 16) | ((unsigned int) values[1] << 8) | (unsigned int) values[2]) >> (8 - BMP180_OVERSAMPLING_SETTING);
	return up;
}

/* Calculate true pressure */
long bmp180_calpressure(long up)
{
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 / 4096 )) / 2048;
	x2 = ac2 * b6 / 2048;
	x3 = x1 + x2;
	b3 = ((((long)ac1 * 4 + x3) << BMP180_OVERSAMPLING_SETTING) + 2) / 4;
	x1 = ac3 * b6 / 8192;
	x2 = (b1 * (b6 * b6 / 4096)) / 65536;
	x3 = ((x1 + x2) + 2) / 4;
	b4 = ac4 * (unsigned long)(x3 + 32768) / 32768;
	b7 = ((unsigned long) up - b3) * (50000 >> BMP180_OVERSAMPLING_SETTING);
	if (b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;
	x1 = (p / 256) * (p / 256);
	x1 = (x1 * 3038) / 65536;
	x2 = (-7357 * p) / 65536;
	long result = (p + (x1 + x2 + 3791) / 16);
	return result;
}

/* Calculate true temperature */
long bmp180_get_temperature(long ut)
{
	x1 = (ut - ac6) * ac5 / 32768;
	x2 = mc * 2048 / (x1 + md);
	b5 = x1 + x2;
	long result = ((b5 + 8) / 16);
	return result;
}

/* Calculate pressure at sea level */
long bmp180_pressure_at_sea_level(long pressure)
{
	return pressure_at_sea_level = (pressure) * powf (1 - ((0.0065 * height) / (temperature + (0.0065 * height) + 273.15)), -5.257);
}

#endif
