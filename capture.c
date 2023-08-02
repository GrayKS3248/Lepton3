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

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <limits.h>
#include <time.h>

#include <sys/ioctl.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>

#include "LEPTON_Types.h"
#include "LEPTON_SDK.h"
#include "LEPTON_OEM.h"

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
// The highest allowed value is 20 MHz.
static const uint32_t speed = 16000000;

// If nonzero, how long to delay after the last bit transfer
// before optionally deselecting the device before the next transfer.
static const uint16_t delay = 128;

// 0 to deselect device before starting the next transfer.
static const uint8_t deselect = 1;


///===========================DEFINE VOSPI PROTOCOL PARAMETERS===========================///
/* The number of bytes per VOSPI frame packet.
In video format mode Raw14, there are 164 bytes per frame packet.
In video format mode RGB888, there are 244 bytes per frame packet. */
#define FRAME_PACKET_SIZE (164)

/* The number of bytes of the ID section of the frame packet header
The first bit of the ID field is always zero.
The next three bits are the TTT bits.
For all packets except for packet number 20, the TTT bits are ignored.
On packet 20, the TTT bits encode the segment number (1,2,3,4) of itself,
the previous 19 packets, and packets 21-59. If the segment number is 0,
the segment is invalid and the entire segment is discarded.
The last 12 bits encode the packet number, 0 - 59 with telemetry
disabled and 0 - 60 with telemetry enabled. Packet numbers restart from
0 on each new segment.
The ID section also encodes the discard condition. Any packet whose ID
section meets the condition (ID & 0x0F00) == 0x0F00 is a discard packet*/
#define FRAME_PACKET_ID_SIZE (2)

/* The number of bytes of the CRC section of the frame packet header
The CRC portion of the packet header contains a 16-bit cyclic
redundancy check (CRC), computed using the following polynomial:
x^(16) + x^(12) + x^(5) + x^(0)
The CRC is calculated over the entire packet, including the
ID and CRC fields. However, the four most-significant
bits of the ID and allsixteen bits of the CRC are set to zero
for calculation of the CRC. */
#define FRAME_PACKET_CRC_SIZE (2)

/* The number of bytes of a single VOSPI segment.
A VOSPI segment is a continuous sequence of VoSPI packets
consisting of one quarter of a frame of pixel data. Each segment
countains either 60 packets (telemetry disabled) or 61 packet
(telemetry enabled) */
#define FRAME_SEGMENT_SIZE (9840)

// Create SPI recieve buffer for a single frame packet of FRAME_PACKET_SIZE bytes
uint8_t frame_packet[FRAME_PACKET_SIZE];

// Create space to hold a single frame segment
static uint8_t frame_segment[FRAME_SEGMENT_SIZE];

// Create a buffer to hold a single frame. Each frame is 120x160 pixels.
// Each pixel is an unsigned 16 bit integer
static unsigned int frame[120][160];


/**
**/
int init_vsync(void)
{
	// Open an I2C port to the command and
	// control interface of the Lepton camera
	LEP_CAMERA_PORT_DESC_T lepton_port;

	//LEP_UINT16 @portID: User defined value to identify a specific comm port.
	//Useful when multiple cameras are attached to a single Host.
	//LEP_CAMERA_PORT_E @portType: LEP_CCI_TWI or LEP_CCI_SPI.
	//LEP_UINT16 @portBaudRate: Port-specific kHz baud. TWI supports only 400.
	//LEP_CAMERA_PORT_DESC_T_PTR @portDescPtr: Pointer to Lepton port file descriptor
	LEP_OpenPort(1, LEP_CCI_TWI, 400, &lepton_port);

	// Set GPIO mode to VSYNC
	//LEP_CAMERA_PORT_DESC_T_PTR @portDescPtr: Pointer to Lepton port file descriptor
	//LEP_OEM_GPIO_MODE_E @gpioMode:
	//LEP_OEM_GPIO_MODE_GPIO = 0,
	//LEP_OEM_GPIO_MODE_I2C_MASTER = 1,
	//LEP_OEM_GPIO_MODE_SPI_MASTER_VLB_DATA = 2,
	//LEP_OEM_GPIO_MODE_SPIO_MASTER_REG_DATA = 3,
	//LEP_OEM_GPIO_MODE_SPI_SLAVE_VLB_DATA = 4,
	//LEP_OEM_GPIO_MODE_VSYNC = 5
	LEP_RESULT result = LEP_SetOemGpioMode(&lepton_port, LEP_OEM_GPIO_MODE_VSYNC);
	if(result != 0)
	{
		pabort("failed to set GPIO mode"); //abort if the GPIO mode cannot be set
	}


	// Check mode after setting VSYNC
	LEP_OEM_GPIO_MODE_E gpio_mode = LEP_OEM_END_GPIO_MODE;
	result = LEP_GetOemGpioMode(&lepton_port, &gpio_mode);
	if(result != 0)
	{
		pabort("failed to read current GPIO mode"); //abort if GPIO mode read failure
	}
	if(gpio_mode != LEP_OEM_GPIO_MODE_VSYNC)
	{
		pabort("incorrect GPIO mode"); //abort if the GPIO mode is not set
	}

	// return 0 on success
	return 0;
}


/**
**/
int reboot_lepton(void)
{
	
}


/**
**/
/*static void save_pgm_file(void)
{
	int i;
	int j;
	unsigned int maxval = 0;
	unsigned int minval = UINT_MAX;
	char image_name[32];
	int image_index = 0;

	do {
		sprintf(image_name, "IMG_%.4d.pgm", image_index);
		image_index += 1;
		if (image_index > 9999)
		{
			image_index = 0;
			break;
		}

	} while (access(image_name, F_OK) == 0);

	FILE *f = fopen(image_name, "w");
	if (f == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}

	printf("Calculating min/max values for proper scaling...\n");
	for(i=0;i<60;i++)
	{
		for(j=0;j<80;j++)
		{
			if (lepton_image[i][j] > maxval) {
				maxval = lepton_image[i][j];
			}
			if (lepton_image[i][j] < minval) {
				minval = lepton_image[i][j];
			}
		}
	}
	printf("maxval = %u\n",maxval);
	printf("minval = %u\n",minval);

	fprintf(f,"P2\n80 60\n%u\n",maxval-minval);
	for(i=0;i<60;i++)
	{
		for(j=0;j<80;j++)
		{
			fprintf(f,"%d ", lepton_image[i][j] - minval);
		}
		fprintf(f,"\n");
	}
	fprintf(f,"\n\n");

	fclose(f);
} */

int transfer_segment(int fd)
{
	clock_t start = clock();

	// Status variables
	int status;

	// Discard tracking variables
	int discard_packet;
	int num_packets_discarded = 0;

	// Packet ID tracking variables
	uint16_t expected_packet_number = 0;
	uint16_t packet_num = 0;
	uint8_t segment_num;

	/// struct spi_ioc_transfer: describes a single SPI transfer
	/// @tx_buf: Holds pointer to userspace buffer with transmit data, or null.
	/// If no data is provided, zeroes are shifted out.
	/// @rx_buf: Holds pointer to userspace buffer for receive data, or null.
	/// @len: Length of tx and rx buffers, in bytes.
	/// @speed_hz: Temporary override of the device's bitrate.
	/// @bits_per_word: Temporary override of the device's wordsize.
	/// @delay_usecs: If nonzero, how long to delay after the last bit transfer
	/// before optionally deselecting the device before the next transfer.
	/// @cs_change: 0 to deselect device before starting the next transfer.
	struct spi_ioc_transfer mesg = {
		.tx_buf = (unsigned long)NULL,
		.rx_buf = (unsigned long)frame_packet,
		.len = FRAME_PACKET_SIZE,
		.speed_hz = speed,
		.bits_per_word = bits,
		.delay_usecs = delay,
		.cs_change = deselect,
	};

	// Recieve 60 valid packets (may include additional discard packets)
	while(packet_num < 60)
	{
		// Transfer one frame packet. status gives the number of bytes
		// recieved.
		status = ioctl(fd, SPI_IOC_MESSAGE(1), &mesg);
		if (status < 1)
		{
			// status < 1 indicates transfer failure
			pabort("can't send spi message");
		}

		// Check if the recieved packet is a discard packet.
		// If it is, ignore the packet and continue to the next.
		discard_packet = (frame_packet[0] & 0x0f) == 0x0f;
		if(discard_packet)
		{
			num_packets_discarded++;
			continue;
		}

		// Retrieve the packet num by extracting the last
		// 12 bits of the frame ID
		packet_num = frame_packet[0]; //0x00_(byte 0)
		packet_num = packet_num << 8; //0x(byte 0)_00
		packet_num = packet_num | frame_packet[1]; //0x(byte 0)_(byte 1)
		packet_num = packet_num & 0x0fff; //0x0(bits 4-7 of byte 0)_(byte 1)

		// Print packet data
		// printf("[num discarded: %d] (recieved packet num: %d) (expected packet num: %d)\n", num_packets_discarded, packet_num, expected_packet_number);

		// Synchronization error detection
		if (expected_packet_number != packet_num)
		{
			double elapsed = (double)(clock() - start) / CLOCKS_PER_SEC;
			printf("unexpected packet number - expected %d - got %d\n", expected_packet_number, packet_num);
			printf("Time spent transferring packets: %1.3fms\n", 1000.*elapsed);
			return -1;
		}

		// If desync is not detected, extract packet data
		// If this is packet number 20, extract the TTT bits.
		// These encode the packet stream's segment number
		if (packet_num == 20)
		{
			segment_num = frame_packet[0]; //first 8 bits
			segment_num = segment_num & 0x70; // isolate bits 1-3
			segment_num = segment_num >> 4; // push bits 1-3 to the end
		}

		// Iterate the expected packet number for the next recieve packet loop
		expected_packet_number++;

	}

	printf("SEGMENT: %d", segment_num);

	return 0;

}

int main(int argc, char *argv[])
{
	///=====================LOCAL VARIABLE DECLARATION=====================///
	uint8_t rd_mode;
	uint8_t rd_bits;
	uint32_t rd_speed;
	int status = 0;


	///=====================INITIALIZE SPI DEVICE=====================///
	// Create file descriptor for SPI device
	int spi_fd;
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
	printf("spi mode: %d\n", rd_mode);
	printf("bits per word: %d\n", rd_bits);
	printf("max speed: %d Hz (%d MHz)\n", rd_speed, rd_speed/1000000);


	///=====================ENGAGE VSYNC MODE=====================///
	init_vsync();


	///=====================INITIALIZE GPIO DEVICE=====================///
	int gpio_fd;
	struct gpiohandle_request gpio_line_handle;
	struct gpiohandle_data gpio_line_data;

 	// Open GPIO chip and get file descriptor
	gpio_fd = open(gpio_device, O_RDONLY);
	if (gpio_fd < 0)
	{
		close(spi_fd);
		pabort("can't open GPIO device");
	}

	// Get line handle for GPIO chip 0, line 82 (Header: 7J1, Pin: 38)
	gpio_line_handle.lineoffsets[0] = 82;
	gpio_line_handle.lines = 1;
	gpio_line_handle.flags = GPIOHANDLE_REQUEST_INPUT;
	status = ioctl(gpio_fd, GPIO_GET_LINEHANDLE_IOCTL, &gpio_line_handle);
	if (status < 0)
	{
		close(gpio_fd);
		close(spi_fd);
		pabort("unable to get GPIO line handle");
	}


	///=====================IMAGE CAPTURE OPERATIONS=====================///
	// Poll GPIO chip 0, line 82 (Header: 7J1, Pin: 38) for next VSYNC pulse
	// The VSYNC signal is low during illegal frame read times ~9.42ms
	// The VSYNC signal is high during legal frame read times ~74us
	uint8_t prev_vsync_val = 0;
	uint8_t curr_vsync_val;
	uint16_t vsync_pulse_num = 0;
	clock_t start_time, end_time;
	double elapsed_time;
	start_time = clock();
	for(int i = 0; i < 10000; i++)
	{
		// Retrieve data from GPIO line handle
		status = ioctl(gpio_line_handle.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &gpio_line_data);

		// Status < 0 indicates failure to communicate with line handle
		if (status < 0)
		{
			close(spi_fd);
			close(gpio_fd);
			close(gpio_line_handle.fd);
			pabort("error while polling event from GPIO");
		}

		// Detect legal frame read pulse
		curr_vsync_val = gpio_line_data.values[0];
		if(curr_vsync_val==1 && prev_vsync_val==0)
		{
			status = transfer_segment(spi_fd); // If VSYNC edge is detected, transfer segment

			// Keep track of VSYNC servicing periods
			end_time = clock();
			elapsed_time = (double)(end_time-start_time) / CLOCKS_PER_SEC;
			printf("Frame pulse: %d. Time spent servicing VSYNC pulse: %1.3fms.\n", vsync_pulse_num, 1000.0*elapsed_time);
			vsync_pulse_num++;

			// If dsync occurs, wait for frame to time out to reset.
			if(status<0)
			{
				usleep(1000000);
			}

			// Restart the clock for the next VSYNC servicing period
			start_time = clock();
		}
		prev_vsync_val=curr_vsync_val;

		// Reduce processor usage
		// VSYNC edges must be detected within two clock cycles
		// of the Lepton master clock. Setting this sleep too high
		// will result in failure to synchronize and/or maintain
		// sync with Lepton module
		//usleep(1);
	}


	///=====================TERMINAL OPERATIONS=====================///
	close(spi_fd);
	close(gpio_fd);
	close(gpio_line_handle.fd);
	//save_pgm_file();

	return 0;
}
