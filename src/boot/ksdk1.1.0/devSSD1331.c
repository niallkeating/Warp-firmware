#include <stdint.h>
#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"



volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}



int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
    
    
    
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);



	/*
	 *	Any post-initialization drawing commands go here.
	 */
    
    writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(0x00);
    writeCommand(0x00);
    writeCommand(95);
    writeCommand(63);
    writeCommand(0x41);
    writeCommand(0x82);
    writeCommand(0x85);
    writeCommand(0x41);
    writeCommand(0x82);
    writeCommand(0x85);
    writeCommand(0x81);
    writeCommand(0x7F);
    writeCommand(0x82);
    writeCommand(0x7F);
    writeCommand(0x83);
    writeCommand(0x7F);
    writeCommand(0x87);
    writeCommand(0xF);
	//...
    


	return 0;
}


void
rectangle(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *width, uint8_t start_pos)
{
    writeCommand(kSSD1331CommandDRAWRECT); //draw rectangle mode
    writeCommand(start_pos); //starting column coordiantes
    writeCommand(0); //starting row coordinates
    writeCommand(*width + start_pos); //finishing column coordiantes
    writeCommand(63); //finishing row coordinates
    writeCommand(*r); //outline colours
    writeCommand(*g);
    writeCommand(*b);
    writeCommand(*r); //fill colours
    writeCommand(*g);
    writeCommand(*b);

}

void
spectrum(uint8_t *red_, uint8_t *orange_, uint8_t *yellow_, uint8_t *green_, uint8_t *blue_, uint8_t *purple_)
{
    uint8_t width = 16;
    rectangle(&red_[0], &red_[1], &red_[2], &width, 0);
    rectangle(&orange_[0], &orange_[1], &orange_[2], &width, 16);
    rectangle(&yellow_[0], &yellow_[1], &yellow_[2], &width, 32);
    rectangle(&green_[0], &green_[1], &green_[2], &width, 48);
    rectangle(&blue_[0], &blue_[1], &blue_[2], &width, 64);
    rectangle(&purple_[0], &purple_[1], &purple_[2], &width, 80);
    
}
void
set_contrast(uint8_t *red_c, uint8_t *green_c, uint8_t *blue_c)
{
    writeCommand(0x81);
    writeCommand(*blue_c);
    writeCommand(0x82);
    writeCommand(*green_c);
    writeCommand(0x83);
    writeCommand(*red_c);

}

float
dot_prod(const float x[3], const uint8_t y[3])
{
    float res = 0.0;
    for (int i = 0; i<3; i++)
    {
        res += x[i] * (float)y[i];
    }
    return res;
}

void
matrix_vector_mult(const float mat[3][3], const uint8_t vec[3], float *result)
{
    for (int i = 0; i<3; i++)
    {
        result[i] = dot_prod(mat[i], vec);
    }
}

void
float2colour(float* colour_raw, uint8_t* colour_processed, int len)
{
    for (int i=0; i<len; i++)
    {
        if(colour_raw[i] > 63.0)
        {
            colour_processed[i] = 63;
        }
        else if (colour_raw[i] < 0)
        {
            colour_processed[i] = 0;
        }
        else
        {
            colour_processed[i] = ceil(colour_raw[i]);
        }
    }
}

void
adjust_spectrum(const float blind_mat[3][3])
{
    uint8_t red2[3], orange2[3], yellow2[3], green2[3], blue2[3], purple2[3];
    float red_res[3], orange_res[3], yellow_res[3], green_res[3], blue_res[3], purple_res[3];
    
    matrix_vector_mult(blind_mat, red, red_res);
    float2colour(red_res, red2, 3);
    matrix_vector_mult(blind_mat, orange, orange_res);
    float2colour(orange_res, orange2, 3);
    matrix_vector_mult(blind_mat, yellow, yellow_res);
    float2colour(yellow_res, yellow2, 3);
    matrix_vector_mult(blind_mat, green, green_res);
    float2colour(green_res, green2, 3);
    matrix_vector_mult(blind_mat, blue, blue_res);
    float2colour(blue_res, blue2, 3);
    matrix_vector_mult(blind_mat, purple, purple_res);
    float2colour(purple_res, purple2, 3);
    
    spectrum(red2, orange2, yellow2, green2, blue2, purple2);
}
