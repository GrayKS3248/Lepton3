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
#include <fcntl.h>
#include <limits.h>

// System libs
#include <sys/ioctl.h>

// Linux kernel libs
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>

// Lepton SDK libs
#include "LEPTON_Types.h"
#include "LEPTON_SDK.h"
#include "LEPTON_OEM.h"
#include "LEPTON_SYS.h"


///===========================DEFINE SPI PROTOCOL PARAMETERS===========================///
// Device name of SPI port Lepton camera is on.
static const char *spi_device = "/dev/spidev0.0";

// SPI mode. Lepton uses mode 3.
#define SPI_MODE (3)

// Number of bytes per SPI word. Valid values: 1, 2, 4. Default is 4.
#define SPI_BYTES_PER_WORD (4)

// SPI clock in Hz. Maximum is 23 MHz.
#define SPI_SPEED (23000000)


///===========================DEFINE VOSPI PROTOCOL PARAMETERS===========================///
//The number of bytes per VOSPI frame packet. Default is 164.
#define PACKET_SIZE (164)

// Number of bytes in frame packet before payload. Default is 4.
#define HEADER_SIZE (4)

// The number of packets in a frame segment. Default is 60.
#define PACKETS_PER_SEGMENT (60)

// The number of bytes in a frame segment
#define SEGMENT_SIZE (PACKET_SIZE * PACKETS_PER_SEGMENT)


///===========================LEPTON COMMAND FUNCTIONS===========================///
/**
**/
int get_lepton_status(void)
{
	// Open I2C port
	LEP_CAMERA_PORT_DESC_T lepton_port;
	LEP_RESULT status = LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);
	if(status != LEP_OK)
	{
		printf("While: get_lepton_status(), could not: LEP_OpenPort(LEP_UINT16, LEP_CAMERA_PORT_E, LEP_UINT16, LEP_CAMERA_PORT_DESC_T_PTR)\n");
		return -1;
	}

	// Check status
	LEP_STATUS_T sys_status;
	status = LEP_GetSysStatus(&lepton_port, &sys_status);
	if(status != LEP_OK)
	{
		printf("While: get_lepton_status(), could not: LEP_GetSysStatus(LEP_CAMERA_PORT_DESC_T_PTR, LEP_STATUS_T_PTR)\n");
		return -1;
	}

	// Read status
	if(sys_status.camStatus != LEP_SYSTEM_READY) return 1;

	// 0 on success
	return 0;
}


/**
**/
int wait_until_ready(void)
{
	// Get the current status and prepare to wait
	int status = get_lepton_status();

	// Wait until ready or timed out
	int count = 0;
	while(status != 0)
	{
		// Check status for failure
		if(status < 0)
		{
			printf("While: wait_until_ready(), could not: get_lepton_status()\n");
			return -1;
		}

		// Wait for 0.5s then poll status again
		usleep(500000);
		status = get_lepton_status();
		printf("...");
		fflush(stdout);

		// Failure if the wait time is greater than 10s
		count++;
		if(count >= 10)
		{
			printf("While: wait_until_ready(), timed out during: get_lepton_status()\n");
			return -1;
		}
	}

	// 0 on success
	return 0;
}


/**
**/
int reboot_lepton(void)
{
	// IO for booting
	printf("\n\n===BOOTING===\n");

	// Open I2C port
	LEP_CAMERA_PORT_DESC_T lepton_port;
	LEP_RESULT status = LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);
	if(status != LEP_OK)
	{
		printf("While: reboot_lepton(), could not: LEP_OpenPort(LEP_UINT16, LEP_CAMERA_PORT_E, LEP_UINT16, LEP_CAMERA_PORT_DESC_T_PTR)\n");
		return -1;
	}

	// Reboot
	int count = 1;
	status = LEP_RunOemReboot(&lepton_port);
	printf("...", status);
	fflush(stdout);
	while(status != LEP_OK)
	{
		if(count >= 5)
		{
			printf("While: reboot_lepton(), timed out during: LEP_RunOemReboot(LEP_CAMERA_PORT_DESC_T_PTR)\n");
			return -1;
		}
		usleep(2000000);
		status = LEP_RunOemReboot(&lepton_port);
		printf("...", status);
		fflush(stdout);
		count++;
	}

	// Wait until ready
	usleep(2000000);
	if(wait_until_ready() < 0)
	{
		printf("While: reboot_lepton(), could not: wait_until_ready()\n");
		return -1;
	}
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
	if(status != LEP_OK)
	{
		printf("While: init_vsync(), could not: LEP_OpenPort(LEP_UINT16, LEP_CAMERA_PORT_E, LEP_UINT16, LEP_CAMERA_PORT_DESC_T_PTR)\n");
		return -1;
	}

	// Check current GPIO mode
	LEP_OEM_GPIO_MODE_E gpio_mode = LEP_OEM_END_GPIO_MODE;
	status = LEP_GetOemGpioMode(&lepton_port, &gpio_mode);
	if(status != LEP_OK)
	{
		printf("While: init_vsync(), could not: LEP_GetOemGpioMode(LEP_CAMERA_PORT_DESC_T_PTR, LEP_OEM_GPIO_MODE_E_PTR)\n");
		return -1;
	}

	// Set GPIO mode to VSYNC
	if(gpio_mode != LEP_OEM_GPIO_MODE_VSYNC)
	{
		status = LEP_SetOemGpioMode(&lepton_port, LEP_OEM_GPIO_MODE_VSYNC);
		if(status != LEP_OK)
		{
			printf("While: init_vsync(), could not: LEP_SetOemGpioMode(LEP_CAMERA_PORT_DESC_T_PTR, LEP_OEM_GPIO_MODE_E)\n");
			return -1;
		}
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
	LEP_RESULT status = LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);
	if(status != LEP_OK)
	{
		printf("While: set_video_format_raw14(), could not: LEP_OpenPort(LEP_UINT16, LEP_CAMERA_PORT_E, LEP_UINT16, LEP_CAMERA_PORT_DESC_T_PTR)\n");
		return -1;
	}

	// Check the current video format
	LEP_OEM_VIDEO_OUTPUT_FORMAT_E format = LEP_END_VIDEO_OUTPUT_FORMAT;
	status = LEP_GetOemVideoOutputFormat(&lepton_port, &format);
	if(status != LEP_OK)
	{
		printf("While: set_video_format_raw14(), could not: LEP_GetOemVideoOutputFormat(LEP_CAMERA_PORT_DESC_T_PTR, LEP_OEM_VIDEO_OUTPUT_FORMAT_E_PTR)\n");
		return -1;
	}

	// Set RAW14 as video format
	if(format != LEP_VIDEO_OUTPUT_FORMAT_RAW14)
	{
		format = LEP_VIDEO_OUTPUT_FORMAT_RAW14;
		LEP_RESULT status = LEP_SetOemVideoOutputFormat(&lepton_port, format);
		if(status != LEP_OK)
		{
			printf("While: set_video_format_raw14(), could not: LEP_SetOemVideoOutputFormat(LEP_CAMERA_PORT_DESC_T_PTR, LEP_OEM_VIDEO_OUTPUT_FORMAT_E)\n");
			return -1;
		}
	}

	// 0 on success
	return 0;
}


///===========================SPI CONFIG FUNCTIONS===========================///
/**
**/
int open_spi(int *fd)
{
	*fd = open(spi_device, O_RDWR);
	if (*fd < 0)
	{
		printf("While: open_spi(int*), could not: open(const char*, mode_t)\n");
		return -1;
	}
	return 0;
}


/**
**/
int set_spi_mode(int *fd)
{
	const int mode = SPI_MODE;
	if(ioctl(*fd, SPI_IOC_WR_MODE, &mode) == -1)
	{
		printf("While: set_spi_mode(int*), could not: ioctl(int, unsinged long, const int*)\n");
		return -1;
	}
	return 0;
}


/**
**/
int set_spi_bits_per_word(int *fd)
{
	const int bits = 8*SPI_BYTES_PER_WORD;
	if(ioctl(*fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
	{
		printf("While: set_spi_bits_per_word(int*), could not: ioctl(int, unsinged long, const int*)\n");
		return -1;
	}
	return 0;
}


/**
**/
int set_spi_speed(int *fd)
{
	const int speed = SPI_SPEED;
	if(ioctl(*fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
	{
		printf("While: set_spi_speed(int*), could not: ioctl(int, unsinged long, const int*)\n");
		return -1;
	}
	return 0;

}


///===========================FRAME STREAM AND SAVE FUNCTIONS===========================///
/**
**/
int save_pgm(uint16_t frame[120][160])
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
			printf("While: save_pgm_file(), failed: no available image names\n");
			return -1;
		}
	} while (access(image_name, F_OK) == 0);

	// Open file
	FILE *f = fopen(image_name, "w");
	if (f == NULL)
	{
		printf("While: save_pgm_file(), could not: fopen(const char*, const char*)\n");
		return -1;
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
	return 0;
}


/**
**/
uint16_t get_ind(uint16_t des_ind)
{
	uint16_t block_num = des_ind / SPI_BYTES_PER_WORD;
	uint16_t new_ind = SPI_BYTES_PER_WORD * (2*block_num + 1) - 1 - des_ind;
	return new_ind;
}


/**
**/
void unpack_raw14_payload(int segment_num, uint8_t *seg_buf, uint16_t frame[120][160])
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
int transfer_segment(int *spi_fd, uint8_t *seg_buf)
{
	uint8_t i;
	uint16_t packet_ind;
	uint16_t expected_packet_num;
	uint16_t packet_num = 65535;
	uint8_t segment_num;

	// Recieve discard packets until the first valid packet is detected
	do {
		// Read a single frame packet and determine validity
		read(*spi_fd, &seg_buf[0], PACKET_SIZE);
		if((seg_buf[get_ind(0)] & 0x0f) == 0x0f) continue;

		// If the packet is valid, read the packet number
		packet_num = seg_buf[get_ind(1)];
		if(packet_num != 0)
		{
			printf("Unexpected packet number: expected 0, got %d\n", packet_num);
			return -1;
		}

		// Set the expected packet number and the index of the next packet
		// in the segment buffer
		expected_packet_num = 1;
		packet_ind = PACKET_SIZE;

	} while (packet_num != 0);

	// Recieve valid packets until the segment is fully transmitted
	while(packet_num + (PACKETS_PER_SEGMENT-1) < 60)
	{
		// Read the entire segment except for the first packet
		// The first packet has already been read
		read(*spi_fd, &seg_buf[packet_ind], SEGMENT_SIZE-PACKET_SIZE);

		// Extract data from the rest of the segment
		for(i = 0; i < PACKETS_PER_SEGMENT-1; i++)
		{
			// Check for correct packet number
			packet_num = seg_buf[get_ind(packet_ind + 1)];
			if(packet_num != expected_packet_num)
			{
				printf("Unexpected packet number: expected %d, got %d\n", expected_packet_num, packet_num);
				return -1;
			}

			// Get the segment number
			if(packet_num==20)
			{
				segment_num = seg_buf[get_ind(packet_ind)];
				segment_num = segment_num >> 4;

				// If the segment is 0, do not continue to unpack
				if(segment_num==0) return segment_num;

				// If the segment is <0 or >4, desync has occured
				if(segment_num<0 || segment_num>4) return -1;
			}

			// Update expected packet number and the index of the next packet
			// in the segment buffer
			expected_packet_num++;
			packet_ind += PACKET_SIZE;
		}
	}

	// Segment number on succes
	return segment_num;
}

int main(int argc, char *argv[])
{
	///=====================CONFIGURE SPI DEVICE=====================///
	// Configure SPI device
	int spi_fd;
	if(open_spi(&spi_fd) < 0) exit(EXIT_FAILURE);
	if(set_spi_mode(&spi_fd) < 0) exit(EXIT_FAILURE);
	if(set_spi_bits_per_word(&spi_fd) < 0) exit(EXIT_FAILURE);
	if(set_spi_speed(&spi_fd) < 0) exit(EXIT_FAILURE);

	// Print SPI settings
	printf("===SPI CONFIG===\n");
	printf("Device: %s\n", spi_device);
	printf("Mode: %d\nBits per Word: %d\nClock: %d MHz\n", SPI_MODE, SPI_BYTES_PER_WORD*8, SPI_SPEED/1000000);


	///=====================CAMERA CONFIGURATION=====================///
	// Configure camera settings
	if(init_vsync() < 0) exit(EXIT_FAILURE);
	if(set_video_format_raw14() < 0) exit(EXIT_FAILURE);

	// Print camera settings
	printf("\n\n===CAMERA CONFIG===\n");
	printf("GPIO mode: VSYNC\n");
	printf("Video output format: RAW14\n");


	///=====================CAMERA STATUS=====================///
	// Ensure Lepton camera status is good
	if(wait_until_ready() < 0) exit(EXIT_FAILURE);

	// Print camera status
	printf("\n\n===CAMERA STATUS===\n");
	printf("Camera status good\n");


	///=====================IMAGE CAPTURE OPERATIONS=====================///
	// Transfer up to 20 segments per frame
	printf("\n\n===TRANSMITTING===\n");
	uint8_t seg_buf[SEGMENT_SIZE];
	uint16_t frame[120][160];
	int expected_segment = 1;
	int segment_num;
	int num_desync = 0;
	int num_frames = 0;
	int num_frames_wanted = 1;
	for(int i = 0; i < 20*num_frames_wanted; i++)
	{
		// Get a frame segment
		segment_num = transfer_segment(&spi_fd, &seg_buf[0]);

		// Segment numbers <0 indicate desynchronization
		if(segment_num<0)
		{
			num_desync++;
			expected_segment = 1;
			printf("Desync occured recieving: Segment Number %d\n", expected_segment);
			printf("Waiting for desync reset...\n");
			printf("----------------------\n");

			// If too many desynchronizations are detected, reboot
			if(num_desync > 3)
			{
				num_desync = 0;
				printf("Too many desyncs detecting. Rebooting...\n\n\n");
				if(reboot_lepton() < 0)
				{
					printf("Can't boot camera. Is DT overlay \"i2c-ao\" enabled?\n");
					close(spi_fd);
					return -1;
				}
				printf("\n\n===TRANSMITTING===\n");
			}

			// If the number of desyncs are in bounds, wait for frame to time out
			else usleep(185000);
		}

		// If the expected segment is recieved, unpack the payload data
		else if(segment_num == expected_segment)
		{
			unpack_raw14_payload(segment_num, &seg_buf[0], frame);
			expected_segment++;

			// Expected segment 5 indicates full frame (4 segments) has been recieved
			if(expected_segment == 5)
			{
				// Update trackers
				num_frames++;
				expected_segment = 1;
				num_desync = 0;
				if(save_pgm(frame) < 0) exit(EXIT_FAILURE);
				printf("Image captured\n");

				// Terminal operations post capture
				if(num_frames >= num_frames_wanted)
				{
					close(spi_fd);
					return 0;
				}
			}
		}
	}


	///=====================FAILURE OPERATIONS=====================///
	printf("Capture failed\n");
	printf("----------------------\n");
	close(spi_fd);
	return -1;
}
