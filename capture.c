/*
Copyright (c) 2014, Pure Engineering LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// C libs
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <limits.h>
#include <time.h>

// System libs
#include <sys/ioctl.h>

// Linux kernel libs
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>

// Lepton SDK libs
#include "LEPTON_Types.h"
#include "LEPTON_SDK.h"
#include "LEPTON_OEM.h"
#include "LEPTON_SYS.h"


/**
**/
static void pabort(const char *s)
{
	perror(s);
	abort();
}


///===========================DEFINE GPIO PARAMETERS FOR VSYNC===========================///
// Device name of gpio chip on which the VSYNC data is sent
static const char *gpio_device = "/dev/gpiochip1";


///===========================DEFINE SPI PROTOCOL PARAMETERS===========================///
// Device name of SPI port of Lepton camera (may have to change to /dev/spidev0.1)
static const char *spi_device = "/dev/spidev0.0";

/* Clock polarity and phase mode. The Lepton 3.x VOSPI protocol uses mode 3.
| MODE || CPOL | CPHA |
|------||------|------|
|  0   ||  0   |  0   |
|  1   ||  0   |  1   |
|  2   ||  1   |  0   |
|  3   ||  1   |  1   |
*/
static const uint8_t mode = 3;

// Number of bits per word. The default is 8
static const uint8_t bits = 8;

// Maximum serial clock frequency (in Hz.) that the board may set.
// The highest allowed value is 24 MHz.
static const uint32_t speed = 23500000;


///===========================DEFINE VOSPI PROTOCOL PARAMETERS===========================///
/* The number of bytes per VOSPI frame packet.
In video format mode Raw14, there are 164 bytes per frame packet.
In video format mode RGB888, there are 244 bytes per frame packet. */
#define PACKET_SIZE (164)

/* The number of packets to recieve in a single transfer after
synchronization has occured */
#define N_PACKETS (59)

/* The number of bytes in N_PACKET frame packets */
#define N_PACKET_SIZE (PACKET_SIZE * N_PACKETS)

/* The number of bytes of the ID section of the frame packet header
The first bit of the ID field is always zero.
The next three bits are the TTT bits.
For all packets except for packet number 20, the TTT bits are ignored.
The TTT bits are bits 1-3.
On packet 20, the TTT bits encode the segment number (1,2,3,4) of itself,
the previous 19 packets, and packets 21-59. If the segment number is 0,
the segment is invalid and the entire segment is discarded.
The last 12 bits encode the packet number, 0 - 59 with telemetry
disabled and 0 - 60 with telemetry enabled. Packet numbers restart from
0 on each new segment.
The ID section also encodes the discard condition. Any packet whose ID
section meets the condition (ID & 0x0F00) == 0x0F00 is a discard packet*/
#define ID_SIZE (2)

/* The number of bytes of the CRC section of the frame packet header
The CRC portion of the packet header contains a 16-bit cyclic
redundancy check (CRC), computed using the following polynomial:
x^(16) + x^(12) + x^(5) + x^(0)
The CRC is calculated over the entire packet, including the
ID and CRC fields. However, the four most-significant
bits of the ID and allsixteen bits of the CRC are set to zero
for calculation of the CRC. */
#define CRC_SIZE (2)

/* Number of bytes in frame packet before payload */
#define HEADER_SIZE (ID_SIZE + CRC_SIZE)

/* The number of bytes per packet payload */
#define PAYLOAD_SIZE (PACKET_SIZE - HEADER_SIZE)

// Create a buffer to hold a single frame. Each frame is 120x160 pixels.
// Each pixel is an unsigned 16 bit integer
static uint16_t frame[120][160] = { 0 };
static uint8_t seg_buf[PACKET_SIZE*60];
//static uint8_t frame_packet[PACKET_SIZE];
//static uint8_t n_frame_packet[N_PACKET_SIZE];


///===========================LEPTON COMMAND FUNCTIONS===========================///
/**
**/
int get_lepton_status(void)
{
	// Open I2C port
	LEP_CAMERA_PORT_DESC_T lepton_port;
	LEP_RESULT status = LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);
	if(status != LEP_OK) return -1;

	// Check status
	LEP_STATUS_T sys_status;
	status = LEP_GetSysStatus(&lepton_port, &sys_status);
	if(status != LEP_OK) return -1;

	// Read status
	if(sys_status.camStatus != LEP_SYSTEM_READY) return -1;

	// 0 on success
	return 0;
}


/**
**/
int wait_until_ready(void)
{
	int status = get_lepton_status();
	int count = 0;
	while(status != 0)
	{
		// Wait for 1s then poll status again
		usleep(500000);
		status = get_lepton_status();
		printf("...");
		fflush(stdout);

		// Failure if the wait time is greater than 10s
		count++;
		if(count >= 10) return -1;
	}

	// 0 on success
	return 0;
}


/**
**/
int reboot_lepton(void)
{
	printf("===BOOTING===\n");

	// Open I2C port
	LEP_CAMERA_PORT_DESC_T lepton_port;
	LEP_RESULT status = LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);
	if(status != LEP_OK) return -1;

	// Reboot
	status = LEP_RunOemReboot(&lepton_port);
	printf("...", status);
	fflush(stdout);
	while(status != LEP_OK)
	{
		usleep(2000000);
		status = LEP_RunOemReboot(&lepton_port);
		printf("...", status);
		fflush(stdout);
	}

	// Wait until ready
	usleep(2000000);
	int ready = wait_until_ready();
	if(ready != 0) return -1;
	printf(" SYSTEM READY\n");
	return 0;
}


/**
**/
int init_vsync(void)
{
	// Open I2C port
	LEP_CAMERA_PORT_DESC_T lepton_port;
	LEP_RESULT status = LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);
	if(status != LEP_OK) return -1;

	// Check current GPIO mode
	LEP_OEM_GPIO_MODE_E gpio_mode = LEP_OEM_END_GPIO_MODE;
	status = LEP_GetOemGpioMode(&lepton_port, &gpio_mode);
	if(status != LEP_OK) return -1;

	// Set GPIO mode to VSYNC
	if(gpio_mode != LEP_OEM_GPIO_MODE_VSYNC)
	{
		status = LEP_SetOemGpioMode(&lepton_port, LEP_OEM_GPIO_MODE_VSYNC);
		if(status != LEP_OK) return -1;
	}

	// 0 on success
	return 0;
}


/**
**/
int set_video_format_raw14(void)
{
	// Open I2C port
	LEP_CAMERA_PORT_DESC_T lepton_port;
	LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);

	// Check the current video format
	LEP_OEM_VIDEO_OUTPUT_FORMAT_E format = LEP_END_VIDEO_OUTPUT_FORMAT;
	LEP_RESULT status = LEP_GetOemVideoOutputFormat(&lepton_port, &format);
	if(status != LEP_OK) return -1;

	// Set RAW14 as video format
	if(format != LEP_VIDEO_OUTPUT_FORMAT_RAW14)
	{
		format = LEP_VIDEO_OUTPUT_FORMAT_RAW14;
		LEP_RESULT status = LEP_SetOemVideoOutputFormat(&lepton_port, format);
		if(status != LEP_OK) return -1;
	}

	// 0 on success
	return 0;
}


///===========================BITBANG SPI FUNCTIONS===========================///
/**
**/
int open_gpio_write(int line, struct gpiohandle_request *rq)
{
 	// Open GPIO chip and get file descriptor
	int fd = open(gpio_device, O_WRONLY);
	if (fd < 0) return -1;

	// Request line handle for GPIO chip 0, header 7J1 at requested line
	rq->lineoffsets[0] = line;
	rq->lines = 1;
	rq->flags = GPIOHANDLE_REQUEST_OUTPUT;
	if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, rq) < 0) return -1;

	// 0 on success
	close(fd);
	return 0;
}


/**
**/
int open_gpio_read(int line, struct gpiohandle_request *rq)
{
 	// Open GPIO chip and get file descriptor
	int fd = open(gpio_device, O_RDONLY);
	if (fd < 0) return -1;

	// Request line handle for GPIO chip 0, header 7J1 at requested line
	rq->lineoffsets[0] = line;
	rq->lines = 1;
	rq->flags = GPIOHANDLE_REQUEST_INPUT;
	if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, rq) < 0) return -1;

	// 0 on success
	close(fd);
	return 0;
}


/**
**/
void set_gpio_high(struct gpiohandle_request *rq, struct gpiohandle_data *dat)
{
	// Set GPIO pin at rq high
	dat->values[0] = 1;
	ioctl(rq->fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, dat);
}


/**
**/
void set_gpio_low(struct gpiohandle_request *rq, struct gpiohandle_data *dat)
{
	// Set GPIO pin at rq low
	dat->values[0] = 0;
	ioctl(rq->fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, dat);
}


/**
**/
uint8_t read_gpio(struct gpiohandle_request *rq, struct gpiohandle_data *dat)
{
	// Read and return from rq
	ioctl(rq->fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, dat);
	return dat->values[0];
}


/**
**/
void read_8bit_word_spi3(uint8_t *word, struct gpiohandle_request *clk_rq, struct gpiohandle_data *clk_dat, struct gpiohandle_request *rd_rq, struct gpiohandle_data *rd_dat)
{
	// Read 8 bits
	for(int i=0; i<8; i++)
	{
		// Data shift on falling edge
		set_gpio_low(clk_rq, clk_dat);

		// Data read on rising edge
		set_gpio_high(clk_rq, clk_dat);
		word[i] = read_gpio(rd_rq, rd_dat);
	}
}


///===========================FRAME STREAM AND SAVE FUNCTIONS===========================///
/**
**/
void save_pgm_file(void)
{
	int i;
	int j;
	unsigned int maxval = 0;
	unsigned int minval = UINT_MAX;
	char image_name[32];
	int image_index = 0;

	// Find next available file name up to IMG_9999.pgm
	do {
		sprintf(image_name, "IMG_%.4d.pgm", image_index);
		image_index += 1;
		if (image_index > 9999)
		{
			image_index = 0;
			break;
		}
	} while (access(image_name, F_OK) == 0);

	// Open file
	FILE *f = fopen(image_name, "w");
	if (f == NULL)
	{
		pabort("error opening save file");
	}

	// Find min and max pixel values (ignore 0 pixel values)
	for(i=0;i<120;i++)
	{
		for(j=0;j<160;j++)
		{
			if (frame[i][j] > maxval) {
				maxval = frame[i][j];
			}
			if ((frame[i][j] < minval) && (frame[i][j] != 0)) {
				minval = frame[i][j];
			}
		}
	}


	// Save image data to pgm file
	fprintf(f,"P2\n160 120\n%u\n",maxval-minval);
	for(i=0;i<120;i++)
	{
		for(j=0;j<160;j++)
		{
			if(frame[i][j]==0) frame[i][j] = minval;
			fprintf(f,"%d ", frame[i][j] - minval);
		}
		fprintf(f,"\n");
	}
	fprintf(f,"\n\n");

	// Close file
	fclose(f);
}


/**
**/
void read_packet_num(uint8_t byte_0, uint8_t byte_1, uint16_t *packet_num)
{
	*packet_num = (uint16_t)byte_0; //0x00_(byte 0)
	*packet_num = *packet_num << 8; //0x(byte 0)_00
	*packet_num = *packet_num | (uint16_t)byte_1; //0x(byte 0)_(byte 1)
	*packet_num = *packet_num & 0x0fff; //0x0(bits 4-7 of byte 0)_(byte 1)
}


/**
**/
void read_segment_num(uint8_t byte_0, uint16_t *segment_num)
{
	*segment_num = (uint16_t)byte_0; //first 8 bits
	*segment_num = *segment_num & 0x70; // isolate bits 1-3
	*segment_num = *segment_num >> 4; // push bits 1-3 to the end
}


/**
**/
void unpack_raw14_payload(uint16_t packet_num, uint8_t payload_size, uint8_t *payload)
{
	// Get the row and col number from the packet number
	uint16_t row_num = packet_num / 2;
	uint16_t col_num = 0;
	if(packet_num & 0x0001) col_num = 80;

	// Loop trough payload
	uint16_t pix_val;
	for(uint8_t i = 0; i < payload_size; i+=2)
	{
		// Extract pixel value
		pix_val = payload[i];
		pix_val = pix_val << 8;
		pix_val = pix_val | payload[i+1];
		pix_val = pix_val & 0x3fff;

		// Add pixel to frame
		frame[row_num][col_num] = pix_val;
		col_num++;
	}
}


/**
**/
int transfer_segment(int *spi_fd)
{
	int status;
	uint8_t i;
	uint8_t num_packets_left;
	uint16_t n_packet_ind;
	uint16_t packet_ind;
	uint16_t expected_packet_num;
	uint16_t packet_num = 65535;
	uint16_t segment_num;

	// Recieve discard packets until the first valid packet is detected
	do {
		// Read a single frame packet and determine validity
		status = read(*spi_fd, &seg_buf[0], PACKET_SIZE);
		if((seg_buf[0] & 0x0f) == 0x0f) continue;

		// If the packet is valid, read the packet number
		packet_num = seg_buf[1];
		if(packet_num != 0)
		{
			printf("Unexpected packet number: Expected 0, Got %d\n", packet_num);
			return -1;
		}

		// Set the expected packet number and the index of the next packet
		// in the segment buffer
		expected_packet_num = 1;
		packet_ind = PACKET_SIZE;

	} while (packet_num != 0);

	// Recieve valid packets in N PACKET chunks
	while(packet_num + N_PACKETS < 60)
	{
		// Read N_PACKETS frame packets
		status = read(*spi_fd, &seg_buf[packet_ind], N_PACKET_SIZE);

		// Extract data from N_PACKETS packets
		for(i = 0; i < N_PACKETS; i++)
		{
			// Check for correct packet number
			packet_num = seg_buf[packet_ind + 1];
			if(packet_num != expected_packet_num)
			{
				printf("Unexpected packet number: Expected %d, Got %d\n", expected_packet_num, packet_num);
				return -1;
			}

			// Update expected packet number and the index of the next packet
			// in the segment buffer
			expected_packet_num++;
			packet_ind += PACKET_SIZE;
		}
	}

	// Grab the last chunk of packets
	num_packets_left = 59-packet_num;
	status = read(*spi_fd, &seg_buf[packet_ind], PACKET_SIZE*num_packets_left);

	// Extract data the last chunk of  packets
	for(i = 0; i < num_packets_left; i++)
	{
		// Check for correct packet number
		packet_num = seg_buf[packet_ind + 1];
		if(packet_num != expected_packet_num)
		{
			printf("Unexpected packet number: Expected %d, Got %d\n", expected_packet_num, packet_num);
			return -1;
		}

		// Update expected packet number and the index of the next packet
		// in the segment buffer
		expected_packet_num++;
		packet_ind += PACKET_SIZE;
	}

	// Unpack image
//	printf("Success! Saving...\n");
//	for(i = 0; i < 60; i++)
//	{
//		packet_ind = PACKET_SIZE*i;
//		unpack_raw14_payload(i, PAYLOAD_SIZE, &seg_buf[HEADER_SIZE+packet_ind]);
//	}
//	save_pgm_file();

	// 0 on success
	return 0;
}

int main(int argc, char *argv[])
{
	///=====================LOCAL VARIABLE DECLARATION=====================///
	uint8_t rd_mode;
	uint8_t rd_bits;
	uint32_t rd_speed;
	int status = 0;
	int spi_fd;


	///=====================REBOOT CAMERA=====================///
	reboot_lepton();


	///=====================INITIALIZE SPI DEVICE=====================///
	// Create file descriptor for SPI device
	spi_fd = open(spi_device, O_RDWR);
	if (spi_fd < 0)
	{
		pabort("can't open spi device");
	}


	///=====================CONFIGURE SPI DEVICE=====================///
	// Attempt to set SPI communication mode
	status = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
	if (status == -1)
	{
		close(spi_fd);
		pabort("can't set spi mode"); // Abort if mode set failure
	}

	// Attempt to read SPI communication mode
	status = ioctl(spi_fd, SPI_IOC_RD_MODE, &rd_mode);
	if (status == -1)
	{
		close(spi_fd);
		pabort("can't get spi mode"); // Abort if mode read failure
	}
	if (rd_mode != mode)
	{
		close(spi_fd);
		pabort("set spi mode failure"); // Abort if set mode and read mode are different
	}

	// Attempt to set SPI bits per word
	status = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (status == -1)
	{
		close(spi_fd);
		pabort("can't set bits per word"); // Abort if bits set failure
	}

	// Attempt to read SPI bits per word
	status = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &rd_bits);
	if (status == -1)
	{
		close(spi_fd);
		pabort("can't get bits per word"); // Abort if bits read failure
	}
	if (rd_mode != mode)
	{
		close(spi_fd);
		pabort("set spi bits failure"); // Abort if set bits and read bits are different
	}

	// Attempt to set maximum allowed SPI read speed
	status = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (status == -1)
	{
		close(spi_fd);
		pabort("can't set max speed hz"); // Abort if speed set failure
	}

	// Attempt to read maximum allowed SPI read speed
	status = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &rd_speed);
	if (status == -1)
	{
		close(spi_fd);
		pabort("can't get max speed hz"); // Abort if speed read failure
	}
	if (rd_speed != speed)
	{
		close(spi_fd);
		pabort("set spi speed failure"); // Abort if set speed and read speed are different
	}

	// print configure SPI settings
	printf("\n\n===SPI CONFIG===\n", rd_mode);
	printf("Device: %s\n", spi_device);
	printf("Mode: %d\nBits per Word: %d\nClock: %d MHz\n", rd_mode, rd_bits, rd_speed/1000000);


	///=====================CAMERA CONFIGURATION=====================///
	// Set camera GPIO mode to VSYNC
	status = init_vsync();
	if(status==0)
	{
		printf("\n\n===CAMERA CONFIG===\n");
		printf("Lepton GPIO Mode: LEP_OEM_GPIO_MODE_VSYNC\n");
	}

	// Set camera video output format to RAW14
	status = set_video_format_raw14();
	if(status==0)
	{
		printf("Video output format: RAW14\n");
	}
	else
	{
		pabort("failed to set video format to RAW14");
	}


	///=====================IMAGE CAPTURE OPERATIONS=====================///
	// Ensure Lepton camera status is good
	printf("\n\n===CAMERA STATUS===\n");
	status = wait_until_ready();
	if(status==0)
	{
		printf("Camera status good\n");
	}
	else
	{
		pabort("camera status returned failure");
	}

	// Transfer 10 segments
	for(int i = 0; i < 10; i++)
	{
		status = transfer_segment(&spi_fd);
		if(status<0)
		{
			printf("Waiting for desync reset...\n");
			usleep(185000);
		}
		else printf("SUCCESS\n");
		printf("----------------------------------------------------------\n\n");
	}


	///=====================TERMINAL OPERATIONS=====================///
	close(spi_fd);
	return 0;
}
