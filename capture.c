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

// System libs
#include <sys/ioctl.h>

// Linux kernel libs
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>

// Lepton SDK libs
#include "LEPTON_Types.h"
#include "LEPTON_SDK.h"
#include "LEPTON_OEM.h"
#include "LEPTON_SYS.h"


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

// Number of bits per word. Must be 32.
static const uint8_t bits = 32;

// Maximum serial clock frequency (in Hz.) that the board may set.
// The highest allowed value is 23 MHz.
static const uint32_t speed = 23000000;


///===========================DEFINE VOSPI PROTOCOL PARAMETERS===========================///
/* The number of bytes per VOSPI frame packet.
In video format mode Raw14, there are 164 bytes per frame packet.
In video format mode RGB888, there are 244 bytes per frame packet. */
#define PACKET_SIZE (164)

/* The number of packets to recieve in a single transfer after
synchronization has occured. MUST BE 1 less than the number of packets in one segment */
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

// Create a buffer to hold a single segment.
static uint8_t seg_buf[PACKET_SIZE*60];

// Create a buffer to hold a single frame. Each frame is 120x160 pixels.
// Each pixel is an unsigned 16 bit integer
static uint16_t frame[120][160] = { 0 };


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
	// IO for booting
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
		printf("Error opening save file: Aborting save operation\n");
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
uint16_t get_ind(uint16_t des_ind)
{
	uint16_t block = des_ind / 4;
	return 8*block + 3 - des_ind;
}


/**
**/
void unpack_raw14_payload(int segment_num)
{
	uint16_t packet_ind;
	uint16_t pix_ind_0;
	uint16_t pix_ind_1;
	uint16_t row_num;
	uint16_t col_num;
	uint16_t pix_val;

	// Go through each 60 packets in the segment
	for(int packet_num = 0; packet_num < 60; packet_num++)
	{
		// Get the index of the first element of the current packet
		packet_ind = PACKET_SIZE*packet_num;

		// Get the row number of the packet
		row_num = 30*(segment_num-1) + (packet_num / 2);

		// Get the first column number of the packet
		col_num = 0;
		if(packet_num & 0x0001) col_num = 80;

		for(int pix_num = 0; pix_num < 80; pix_num++)
		{
			// Get the indices of the current pixel
			pix_ind_0 = get_ind(packet_ind + HEADER_SIZE + 2*pix_num);
			pix_ind_1 =  get_ind(packet_ind + HEADER_SIZE + 2*pix_num + 1);

			// Extract the pixel value
			pix_val = seg_buf[pix_ind_0];
			pix_val = pix_val << 8;
			pix_val = pix_val | seg_buf[pix_ind_1];
			pix_val = pix_val & 0x3fff;

			// Add pixel to frame
			frame[row_num][col_num] = pix_val;
			col_num++;
		}
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
	uint8_t segment_num;

	// Recieve discard packets until the first valid packet is detected
	do {
		// Read a single frame packet and determine validity
		status = read(*spi_fd, &seg_buf[0], PACKET_SIZE);
		if((seg_buf[get_ind(0)] & 0x0f) == 0x0f) continue;

		// If the packet is valid, read the packet number
		packet_num = seg_buf[get_ind(1)];
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
			packet_num = seg_buf[get_ind(packet_ind + 1)];
			if(packet_num != expected_packet_num)
			{
				printf("Unexpected packet number: Expected %d, Got %d\n", expected_packet_num, packet_num);
				return -1;
			}

			// Get the segment number
			if(packet_num==20)
			{
				segment_num = seg_buf[get_ind(packet_ind)];
				segment_num = segment_num >> 4;
			}

			// Update expected packet number and the index of the next packet
			// in the segment buffer
			expected_packet_num++;
			packet_ind += PACKET_SIZE;
		}
	}

	// Unpack image
	//printf("Success! Saving...\n");
	//for(i = 0; i < 60; i++)
	//{
	//	packet_ind = PACKET_SIZE*i;
	//	unpack_raw14_payload(i, PAYLOAD_SIZE, &seg_buf[HEADER_SIZE+packet_ind]);
	//}
	//save_pgm_file();

	// 0 on succes
	printf("Segment recieved: %d\n", segment_num);
	return segment_num;
}

int main(int argc, char *argv[])
{
	///=====================REBOOT CAMERA=====================///
	reboot_lepton();

	///=====================CONFIGURE SPI DEVICE=====================///
	// Create file descriptor for SPI device
	int spi_fd = open(spi_device, O_RDWR);
	if (spi_fd < 0)
	{
		printf("Can't open spi device. Are DT overlays \"spicc\" and \"spicc-spidev\" enabled?\n"); // Abort if can't open SPI
		return -1;
	}

	// Set SPI communication mode
	int status = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
	if (status == -1)
	{
		close(spi_fd);
		printf("Can't set SPI mode\n"); // Abort if mode set failure
		return -1;
	}

	// Set SPI bits per word
	status = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (status == -1)
	{
		close(spi_fd);
		printf("Can't set SPI bits per word\n"); // Abort if bits set failure
		return -1;
	}

	// Set SPI clock
	status = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (status == -1)
	{
		close(spi_fd);
		printf("Can't set SPI clock speed\n"); // Abort if speed set failure
		return -1;
	}

	// print configure SPI settings
	printf("\n\n===SPI CONFIG===\n");
	printf("Device: %s\n", spi_device);
	printf("Mode: %d\nBits per Word: %d\nClock: %d MHz\n", mode, bits, speed/1000000);


	///=====================CAMERA CONFIGURATION=====================///
	// Set camera GPIO mode to VSYNC
	status = init_vsync();
	if(status==0)
	{
		printf("\n\n===CAMERA CONFIG===\n");
		printf("Lepton GPIO Mode: LEP_OEM_GPIO_MODE_VSYNC\n");
	}
	else
	{
		close(spi_fd);
		printf("Can't initialize VSYNC. Is DT overlay \"i2c-ao\" enabled?\n");
		return -1;
	}

	// Set camera video output format to RAW14
	status = set_video_format_raw14();
	if(status==0)
	{
		printf("Video output format: RAW14\n");
	}
	else
	{
		close(spi_fd);
		printf("Can't set video format to RAW14\n");
		return -1;
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
		printf("Camera status returned failure\n");
		return -1;
	}

	// Transfer 25 segments
	printf("\n\n===TRANSMITTING===\n");
	int expected_segment = 1;
	int segment_num;
	for(int i = 0; i < 25; i++)
	{
		segment_num = transfer_segment(&spi_fd);
		if(segment_num<0)
		{
			printf("Waiting for desync reset...\n");
			usleep(185000);
		}
		else if(segment_num == expected_segment)
		{
			unpack_raw14_payload(segment_num);
			expected_segment++;
			if(expected_segment == 5)
			{
				expected_segment = 1;
				save_pgm_file();
			}
		}
		printf("-----------------\n");
	}


	///=====================TERMINAL OPERATIONS=====================///
	close(spi_fd);
	return 0;
}
