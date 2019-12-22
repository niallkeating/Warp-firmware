/*
 *	See https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino for the Arduino driver.
 */

#ifndef WARP_BUILD_ENABLE_DEVSSD1331
#define WARP_BUILD_ENABLE_DEVSSD1331
#endif

#include <math.h>

typedef enum
{
	kSSD1331ColororderRGB		= 1,
	kSSD1331DelaysHWFILL		= 3,
	kSSD1331DelaysHWLINE		= 1,
} SSD1331Constants;

typedef enum
{
	kSSD1331CommandDRAWLINE		= 0x21,
	kSSD1331CommandDRAWRECT		= 0x22,
	kSSD1331CommandCLEAR		= 0x25,
	kSSD1331CommandFILL		= 0x26,
	kSSD1331CommandSETCOLUMN	= 0x15,
	kSSD1331CommandSETROW		= 0x75,
	kSSD1331CommandCONTRASTA	= 0x81,
	kSSD1331CommandCONTRASTB	= 0x82,
	kSSD1331CommandCONTRASTC	= 0x83,
	kSSD1331CommandMASTERCURRENT	= 0x87,
	kSSD1331CommandSETREMAP		= 0xA0,
	kSSD1331CommandSTARTLINE	= 0xA1,
	kSSD1331CommandDISPLAYOFFSET	= 0xA2,
	kSSD1331CommandNORMALDISPLAY	= 0xA4,
	kSSD1331CommandDISPLAYALLON	= 0xA5,
	kSSD1331CommandDISPLAYALLOFF	= 0xA6,
	kSSD1331CommandINVERTDISPLAY	= 0xA7,
	kSSD1331CommandSETMULTIPLEX	= 0xA8,
	kSSD1331CommandSETMASTER	= 0xAD,
	kSSD1331CommandDISPLAYOFF	= 0xAE,
	kSSD1331CommandDISPLAYON	= 0xAF,
	kSSD1331CommandPOWERMODE	= 0xB0,
	kSSD1331CommandPRECHARGE	= 0xB1,
	kSSD1331CommandCLOCKDIV		= 0xB3,
	kSSD1331CommandPRECHARGEA	= 0x8A,
	kSSD1331CommandPRECHARGEB	= 0x8B,
	kSSD1331CommandPRECHARGEC	= 0x8C,
	kSSD1331CommandPRECHARGELEVEL	= 0xBB,
	kSSD1331CommandVCOMH		= 0xBE,
} SSD1331Commands;

const uint8_t red[3] = {37, 0, 2};
const uint8_t orange[3] = {55, 15, 0};
const uint8_t yellow[3] = {53, 43, 0};
const uint8_t green[3] = {0, 55, 0};
const uint8_t blue[3] = {0, 0, 55};
const uint8_t purple[3] = {27, 0, 26};

const float normal[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
const float deuteranopia[3][3] = {{0.625, 0.375, 0}, {0.7, 0.3, 0}, {0, 0.3, 0.7}}; //0.375, 0
const float protanopia[3][3] = {{0.567, 0.433, 0}, {0.558, 0.442, 0}, {0, 0.242, 0.758}}; //
const float tritanopia[3][3] = {{0.95, 0.05, 0}, {0, 0.433, 0.567}, {0, 0.475, 0.525}};

int	devSSD1331init(void);
void rectangle(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *width, uint8_t start_pos);
void spectrum(uint8_t *red_, uint8_t *orange_, uint8_t *yellow_, uint8_t *green_, uint8_t *blue_, uint8_t *purple_);
void set_contrast(uint8_t *red_c, uint8_t *green_c, uint8_t *blue_c);
float dot_prod(const float x[3], const uint8_t y[3]);
void matrix_vector_mult(const float mat[3][3], const uint8_t vec[3], float *result);
void float2colour(float* colour_raw, uint8_t* colour_processed, int len);
void adjust_spectrum(const float blind_mat[3][3]);
