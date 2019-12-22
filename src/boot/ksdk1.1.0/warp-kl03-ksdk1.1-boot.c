/*
    Authored 2016-2018. Phillip Stanley-Marbell.
    
    Additional contributions, 2018: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.
    All rights reserved.
    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    *    Redistributions of source code must retain the above
        copyright notice, this list of conditions and the following
        disclaimer.
    *    Redistributions in binary form must reproduce the above
        copyright notice, this list of conditions and the following
        disclaimer in the documentation and/or other materials
        provided with the distribution.
    *    Neither the name of the author nor the names of its
        contributors may be used to endorse or promote products
        derived from this software without specific prior written
        permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#define WARP_FRDMKL03


#ifndef WARP_FRDMKL03

#else
#    include "devSSD1331.h"
#    include "devTCS34725.h"
#endif

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
//#define WARP_BUILD_BOOT_TO_CSVSTREAM


/*
*    BTstack includes WIP
*/
// #include "btstack_main.h"


#define                        kWarpConstantStringI2cFailure        "\rI2C failed, reg 0x%02x, code %d\n"
#define                        kWarpConstantStringErrorInvalidVoltage    "\rInvalid supply voltage [%d] mV!"
#define                        kWarpConstantStringErrorSanity        "\rSanity check failed!"


#ifdef WARP_BUILD_ENABLE_DEVTCS34725
volatile WarpI2CDeviceState            deviceTCS34725State;
#endif

/*
 *    TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t            i2cMasterState;
volatile spi_master_state_t            spiMasterState;
volatile spi_master_user_config_t        spiUserConfig;
volatile lpuart_user_config_t             lpuartUserConfig;
volatile lpuart_state_t             lpuartState;

/*
 *    TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t            gWarpI2cBaudRateKbps        = 200;
volatile uint32_t            gWarpUartBaudRateKbps        = 1;
volatile uint32_t            gWarpSpiBaudRateKbps        = 200;
volatile uint32_t            gWarpSleeptimeSeconds        = 0;
volatile WarpModeMask            gWarpMode            = kWarpModeDisableAdcOnSleep;
volatile uint32_t            gWarpI2cTimeoutMilliseconds    = 5;
volatile uint32_t            gWarpSpiTimeoutMicroseconds    = 5;
volatile uint32_t            gWarpMenuPrintDelayMilliseconds    = 10;
volatile uint32_t            gWarpSupplySettlingDelayMilliseconds = 1;


void                    lowPowerPinStates(void);
void                    disableTPS82740A(void);
void                    disableTPS82740B(void);
void                    enableTPS82740A(uint16_t voltageMillivolts);
void                    enableTPS82740B(uint16_t voltageMillivolts);
void                    setTPS82740CommonControlLines(uint16_t voltageMillivolts);
void                    getRawData(volatile WarpI2CDeviceState *  i2cDeviceState,
                                WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister,  int numberOfBytes),
                                uint8_t *  r, uint8_t *  g, uint8_t *  b);
void                    enableSssupply(uint16_t voltageMillivolts);
void                    disableSssupply(void);


uint16_t SssupplyMillivolts = 1800;

uint8_t state = 1;
uint8_t prev_state = 1;
uint8_t r_contrast = 90;
uint8_t g_contrast = 40;
uint8_t b_contrast = 40;
uint8_t std_contrast = 127;
uint8_t all_contrast = 127;
uint8_t next_state = 2;

float blind_type[3][3];

uint8_t sensor_raw[3];
float sensor_adjusted_float[3];
uint8_t sensor_adjusted[3];

uint8_t width = 96;

WarpStatus TCS34725ConfigStatus;

/*
 *    From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *    static clock callback table.
 */
clock_manager_callback_user_config_t        clockManagerCallbackUserlevelStructure =
                                    {
                                        .callback    = clockManagerCallbackRoutine,
                                        .callbackType    = kClockManagerCallbackBeforeAfter,
                                        .callbackData    = NULL
                                    };

static clock_manager_callback_user_config_t *    clockCallbackTable[] =
                                    {
                                        &clockManagerCallbackUserlevelStructure
                                    };

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
    clock_manager_error_code_t result = kClockManagerSuccess;

    switch (notify->notifyType)
    {
        case kClockManagerNotifyBefore:
            break;
        case kClockManagerNotifyRecover:
        case kClockManagerNotifyAfter:
            break;
        default:
            result = kClockManagerError;
        break;
    }

    return result;
}


void
enableSPIpins(void)
{
    CLOCK_SYS_EnableSpiClock(0);

    /*    Warp KL03_SPI_MISO    --> PTA6    (ALT3)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

    /*    Warp KL03_SPI_MOSI    --> PTA7    (ALT3)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

    /*    Warp KL03_SPI_SCK    --> PTB0    (ALT3)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);


    /*
     *    Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
     *
     */
    uint32_t            calculatedBaudRate;
    spiUserConfig.polarity        = kSpiClockPolarity_ActiveHigh;
    spiUserConfig.phase        = kSpiClockPhase_FirstEdge;
    spiUserConfig.direction        = kSpiMsbFirst;
    spiUserConfig.bitsPerSec    = gWarpSpiBaudRateKbps * 1000;
    SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
    SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
disableSPIpins(void)
{
    SPI_DRV_MasterDeinit(0);


    /*    Warp KL03_SPI_MISO    --> PTA6    (GPI)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

    /*    Warp KL03_SPI_MOSI    --> PTA7    (GPIO)        */
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

    /*    Warp KL03_SPI_SCK    --> PTB0    (GPIO)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);


    CLOCK_SYS_DisableSpiClock(0);
}



void
enableI2Cpins(uint16_t pullupValue)
{
    CLOCK_SYS_EnableI2cClock(0);

    /*    Warp KL03_I2C0_SCL    --> PTB3    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (ALT2 == I2C)        */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);


    I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);


    /*
     *    TODO: need to implement config of the DCP
     */
    //...
}



void
disableI2Cpins(void)
{
    I2C_DRV_MasterDeinit(0 /* I2C instance */);


    /*    Warp KL03_I2C0_SCL    --> PTB3    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

    /*    Warp KL03_I2C0_SDA    --> PTB4    (GPIO)            */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);


    /*
     *    TODO: need to implement clearing of the DCP
     */
    //...

    /*
     *    Drive the I2C pins low
     */
    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);


    CLOCK_SYS_DisableI2cClock(0);
}


// TODO: add pin states for pan1326 lp states
void
lowPowerPinStates(void)
{
    /*
     *    Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
     *    we configure all pins as output and set them to a known state. We choose
     *    to set them all to '0' since it happens that the devices we want to keep
     *    deactivated (SI4705, PAN1326) also need '0'.
     */

    /*
     *            PORT A
     */
    /*
     *    For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
     */
    /*
    PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
    */

    /*
     *    PTA3 and PTA4 are the EXTAL/XTAL
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

    PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
    
    /*
     *    NOTE: The KL03 has no PTA10 or PTA11
     */

    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);



    /*
     *            PORT B
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
    
    /*
     *    PTB1 is connected to KL03_VDD. We have a choice of:
     *        (1) Keep 'disabled as analog'.
     *        (2) Set as output and drive high.
     *
     *    Pin state "disabled" means default functionality (ADC) is _active_
     */
    if (gWarpMode & kWarpModeDisableAdcOnSleep)
    {
        PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
    }
    else
    {
        PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
    }

    PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

    /*
     *    PTB3 and PTB3 (I2C pins) are true open-drain
     *    and we purposefully leave them disabled.
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);


    PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

    /*
     *    NOTE: The KL03 has no PTB8 or PTB9
     */

    PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

    /*
     *    NOTE: The KL03 has no PTB12
     */
    
    PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);



    /*
     *    Now, set all the pins (except kWarpPinKL03_VDD_ADC, the SWD pins, and the XTAL/EXTAL) to 0
     */
    
    
    
    /*
     *    If we are in mode where we disable the ADC, then drive the pin high since it is tied to KL03_VDD
     */
    if (gWarpMode & kWarpModeDisableAdcOnSleep)
    {
        GPIO_DRV_SetPinOutput(kWarpPinKL03_VDD_ADC);
    }
#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_nSHUTD);
#endif
#endif

    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

#ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
    GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
#endif

    GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

    /*
     *    Drive these chip selects high since they are active low:
     */
    #ifndef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
    GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);
#endif
#ifdef WARP_BUILD_ENABLE_DEVADXL362
    GPIO_DRV_SetPinOutput(kWarpPinADXL362_CS);
#endif

    /*
     *    When the PAN1326 is installed, note that it has the
     *    following pull-up/down by default:
     *
     *        HCI_RX / kWarpPinI2C0_SCL    : pull up
     *        HCI_TX / kWarpPinI2C0_SDA    : pull up
     *        HCI_RTS / kWarpPinSPI_MISO    : pull up
     *        HCI_CTS / kWarpPinSPI_MOSI    : pull up
     *
     *    These I/Os are 8mA (see panasonic_PAN13xx.pdf, page 10),
     *    so we really don't want to be driving them low. We
     *    however also have to be careful of the I2C pullup and
     *    pull-up gating. However, driving them high leads to
     *    higher board power dissipation even when SSSUPPLY is off
     *    by ~80mW on board #003 (PAN1326 populated).
     *
     *    In revB board, with the ISL23415 DCP pullups, we also
     *    want I2C_SCL and I2C_SDA driven high since when we
     *    send a shutdown command to the DCP it will connect
     *    those lines to 25570_VOUT.
     *
     *    For now, we therefore leave the SPI pins low and the
     *    I2C pins (PTB3, PTB4, which are true open-drain) disabled.
     */

    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
    GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
    GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

    /*
     *    HCI_RX / kWarpPinI2C0_SCL is an input. Set it low.
     */
    //GPIO_DRV_SetPinOutput(kWarpPinI2C0_SCL);

    /*
     *    HCI_TX / kWarpPinI2C0_SDA is an output. Set it high.
     */
    //GPIO_DRV_SetPinOutput(kWarpPinI2C0_SDA);

    /*
     *    HCI_RTS / kWarpPinSPI_MISO is an output. Set it high.
     */
    //GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO);

    /*
     *    From PAN1326 manual, page 10:
     *
     *        "When HCI_CTS is high, then CC256X is not allowed to send data to Host device"
     */
    //GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI);
}



void
disableTPS82740A(void)
{
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
}

void
disableTPS82740B(void)
{
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
}


void
enableTPS82740A(uint16_t voltageMillivolts)
{
    setTPS82740CommonControlLines(voltageMillivolts);
    GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_CTLEN);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);

    /*
     *    Select the TS5A3154 to use the output of the TPS82740
     *
     *        IN = high selects the output of the TPS82740B:
     *        IN = low selects the output of the TPS82740A:
     */
    GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
}


void
enableTPS82740B(uint16_t voltageMillivolts)
{
    setTPS82740CommonControlLines(voltageMillivolts);
    GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
    GPIO_DRV_SetPinOutput(kWarpPinTPS82740B_CTLEN);

    /*
     *    Select the TS5A3154 to use the output of the TPS82740
     *
     *        IN = high selects the output of the TPS82740B:
     *        IN = low selects the output of the TPS82740A:
     */
    GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);
}


void
setTPS82740CommonControlLines(uint16_t voltageMillivolts)
{
    /*
     *     From Manual:
     *
     *        TPS82740A:    VSEL1 VSEL2 VSEL3:    000-->1.8V, 111-->2.5V
     *        TPS82740B:    VSEL1 VSEL2 VSEL3:    000-->2.6V, 111-->3.3V
     */

    switch(voltageMillivolts)
    {
        case 2600:
        case 1800:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        case 2700:
        case 1900:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        case 2800:
        case 2000:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        case 2900:
        case 2100:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        case 3000:
        case 2200:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        case 3100:
        case 2300:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        case 3200:
        case 2400:
        {
            GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        case 3300:
        case 2500:
        {
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
            GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);
            
            break;
        }

        /*
         *    Should never happen, due to previous check in enableSssupply()
         */
        default:
        {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
            SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
#endif
        }
    }

    /*
     *    Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
     */
    OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}



void
enableSssupply(uint16_t voltageMillivolts)
{
    if (voltageMillivolts >= 1800 && voltageMillivolts <= 2500)
    {
        enableTPS82740A(voltageMillivolts);
    }
    else if (voltageMillivolts >= 2600 && voltageMillivolts <= 3300)
    {
        enableTPS82740B(voltageMillivolts);
    }
    else
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
#endif
    }
}

void
disableSssupply(void)
{
    disableTPS82740A();
    disableTPS82740B();

    /*
     *    Clear the pin. This sets the TS5A3154 to use the output of the TPS82740B,
     *    which shouldn't matter in any case. The main objective here is to clear
     *    the pin to reduce power drain.
     *
     *        IN = high selects the output of the TPS82740B:
     *        IN = low selects the output of the TPS82740A:
     */
    GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);

    /*
     *    Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
     */
    OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}



int
main(void)
{
    //I2C pullup value
    uint16_t                PullupValue = 32768;

    /*
     *    Enable clock for I/O PORT A and PORT B
     */
    CLOCK_SYS_EnablePortClock(0);
    CLOCK_SYS_EnablePortClock(1);

    /*
     *    Setup board clock source.
     */
    g_xtal0ClkFreq = 32768U;

    /*
     *    Initialize KSDK Operating System Abstraction layer (OSA) layer.
     */
    OSA_Init();

    /*
     *    Setup SEGGER RTT to output as much as fits in buffers.
     *
     *    Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
     *    we might have SWD disabled at time of blockage.
     */
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);


    SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Colour Blindness Adjuster, in 3... ");
    OSA_TimeDelay(200);
    SEGGER_RTT_WriteString(0, "2... ");
    OSA_TimeDelay(200);
    SEGGER_RTT_WriteString(0, "1...\n\r");
    OSA_TimeDelay(200);

    /*
     *    Configure Clock Manager to default, and set callback for Clock Manager mode transition.
     *
     *    See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
     */
    
    CLOCK_SYS_Init(    g_defaultClockConfigurations,
            CLOCK_CONFIG_NUM,
            &clockCallbackTable,
            ARRAY_SIZE(clockCallbackTable)
            );
    CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

    /*
     *    Initialize RTC Driver
     */
    RTC_DRV_Init(0);

    /*
     *    Initialize the GPIO pins with the appropriate pull-up, etc.,
     *    defined in the inputPins and outputPins arrays (gpio_pins.c).
     *
     *    See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
     */
    GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
    
    /*
     *    Note that it is lowPowerPinStates() that sets the pin mux mode,
     *    so until we call it pins are in their default state.
     */
    lowPowerPinStates();

    /*
     *    Toggle LED3 (kWarpPinSI4705_nRST)
     */
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
    OSA_TimeDelay(200);
    GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);



    /*
     *    Initialize the sensor and OLED
     */

#ifdef WARP_BUILD_ENABLE_DEVTCS34725
    initTCS34725(    0x29    /* i2cAddress */,    &deviceTCS34725State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVSSD1331
    devSSD1331init();
#endif

    
    SEGGER_RTT_printf(0, "\nConfiguring sensor");
    
    //Enable I2C to write configuaration commands to sensor
    enableI2Cpins(PullupValue);
    
    //Set supply voltage (to default 1800mV)
    enableSssupply(SssupplyMillivolts);

    //Configuration
    TCS34725ConfigStatus = configureSensorTCS34725();
    if (TCS34725ConfigStatus != kWarpStatusOK)
    {
        SEGGER_RTT_printf(0, "\nError when reading from TCS34725 device");
    }

    //Initialisation of button read values
    uint32_t button_up, button_down, button_next, button_prev, button_sel;
    
    
    SEGGER_RTT_printf(0, "\nDevice Ready");

    while (1)
    {
        
        switch(state)
        {
            /*
             *      Case 1: normal vision
             *      Prev: no response
             *      Next: Go to Deuteranopia vision
             *      Up: Increase overall screen brightness
             *      Down: Decrease overall screen brightness
             *      Select: Selects 'normal vision' colour filter (i.e. doesn't adjust colour)
             */
            case 1:
            {
                set_contrast(&all_contrast, &all_contrast, &all_contrast);
                adjust_spectrum(normal);
                while(state == 1)
                {
                    OSA_TimeDelay(100);
                    button_up = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 6)); //normally high
                    button_down = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 2));//normally high
                    button_next = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 1)); //normally high
                    button_prev = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 7)); //normally high
                    button_sel = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 5)); //normally low
                    if (button_next == 0)
                    {
                        state = 2;
                        SEGGER_RTT_printf(0, "\r\t Next: Deuteranopia\n");
                    }
                    if (button_prev == 0)
                    {
                        state = 4;
                        SEGGER_RTT_printf(0, "\r\t Prev: Tritanopia\n");
                    }
                    else if (button_up == 0)
                    {
                        all_contrast += 10;
                        set_contrast(&all_contrast, &all_contrast, &all_contrast);
                        SEGGER_RTT_printf(0, "\r\t Contrast all increase: %d\n", all_contrast);
                    }
                    else if (button_down == 0)
                    {
                        all_contrast -= 10;
                        set_contrast(&all_contrast, &all_contrast, &all_contrast);
                        SEGGER_RTT_printf(0, "\r\t Contrast all decrease: %d\n", all_contrast);
                    }
                    else if (button_sel == 1)
                    {
                        state = 5;
                        prev_state = 1;
                        memcpy(blind_type, normal, sizeof(normal));
                        SEGGER_RTT_printf(0, "\r\t Selected normal mode\n");
                    }
                        
                }
                break;
            }
            /*
            *      Case 2: Deuteranopia vision (green deficiency)
            *      Prev: Go to Normal Vision
            *      Next: Go to Protanopia vision
            *      Up: Increase Green contrast only (for callibration)
            *      Down: Decrease green contrast only
            *      Select: Selects 'Deuteranopia' colour filter for sensor
            */
                
            case 2:
            {
                set_contrast(&std_contrast, &g_contrast, &std_contrast);
                adjust_spectrum(deuteranopia);
                
                while(state == 2)
                {
                    OSA_TimeDelay(100);
                    button_up = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 6)); //normally high
                    button_down = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 2));//normally high
                    button_next = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 1)); //normally high
                    button_prev = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 7)); //normally high
                    button_sel = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 5)); //normally low
                    if (button_next == 0)
                    {
                        state = 3;
                        SEGGER_RTT_printf(0, "\r\t Next: Protanopia\n");
                    }
                    if (button_prev == 0)
                    {
                        state = 1;
                        SEGGER_RTT_printf(0, "\r\t Prev: Normal\n");
                    }
                    else if (button_up == 0)
                    {
                        g_contrast += 10;
                        set_contrast(&std_contrast, &g_contrast, &std_contrast);
                        SEGGER_RTT_printf(0, "\r\t Increase green contrast to %d\n", g_contrast);
                    }
                    else if (button_down == 0)
                    {
                        g_contrast -= 10;
                        set_contrast(&std_contrast, &g_contrast, &std_contrast);
                        SEGGER_RTT_printf(0, "\r\t Decrease green contrast to %d\n", g_contrast);
                    }
                    else if (button_sel == 1)
                    {
                        state = 5;
                        prev_state = 2;
                        memcpy(blind_type, deuteranopia, sizeof(deuteranopia));
                        SEGGER_RTT_printf(0, "\r\t Select Deuteranopia mode \n");
                    }
                        
                }
                break;
            }
            
            /*
            *      Case 3: Protanopia vision (red deficiency)
            *      Prev: Go to Deuteranopia Vision
            *      Next: Go to Tritanopia vision
            *      Up: Increase Red contrast only (for callibration)
            *      Down: Decrease Red contrast only
            *      Select: Selects 'Protanopianopia' colour filter for sensor
            */
                
            case 3:
            {
                set_contrast(&r_contrast, &std_contrast, &std_contrast);
                adjust_spectrum(protanopia);
                
                while(state == 3)
                {
                    OSA_TimeDelay(100);
                    button_up = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 6)); //normally high
                    button_down = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 2));//normally high
                    button_next = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 1)); //normally high
                    button_prev = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 7)); //normally high
                    button_sel = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 5)); //normally low
                    if (button_next == 0)
                    {
                        state = 4;
                        SEGGER_RTT_printf(0, "\r\t Next: Tritanopia\n");
                    }
                    if (button_prev == 0)
                    {
                        state = 2;
                        SEGGER_RTT_printf(0, "\r\t Prev: Deutranopia\n");
                    }
                    else if (button_up == 0)
                    {
                        r_contrast += 10;
                        set_contrast(&r_contrast, &std_contrast, &std_contrast);
                        SEGGER_RTT_printf(0, "\r\t Increase red contrast to %d\n", r_contrast);
                    }
                    else if (button_down == 0)
                    {
                        r_contrast -= 10;
                        set_contrast(&r_contrast, &std_contrast, &std_contrast);
                        SEGGER_RTT_printf(0, "\r\t Decrease red contrast to %d\n", r_contrast);
                    }
                    else if (button_sel == 1)
                    {
                        state = 5;
                        prev_state = 3;
                        memcpy(blind_type, protanopia, sizeof(protanopia));
                        SEGGER_RTT_printf(0, "\r\t Select Protanopia mode \n");
                    }
                        
                }
                break;
            }
                
            /*
            *      Case 4: Tritanopia vision (blue deficiency)
            *      Prev: Go to Protanopia Vision
            *      Next: Go to Normal Vision
            *      Up: Increase Blue contrast only (for callibration)
            *      Down: Decrease Blue contrast only
            *      Select: Selects 'Tritanopia' colour filter for sensor
            */
                
            case 4:
            {
                set_contrast(&std_contrast, &std_contrast, &b_contrast);
                adjust_spectrum(tritanopia);
                
                while(state == 4)
                {
                    OSA_TimeDelay(100);
                    button_up = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 6)); //normally high
                    button_down = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 2));//normally high
                    button_next = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 1)); //normally high
                    button_prev = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 7)); //normally high
                    button_sel = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 5)); //normally low
                    if (button_next == 0)
                    {
                        state = 1;
                        SEGGER_RTT_printf(0, "\r\t Next: Normal\n");
                    }
                    if (button_prev == 0)
                    {
                        state = 3;
                        SEGGER_RTT_printf(0, "\r\t Prev: Protanopia\n");
                    }
                    else if (button_up == 0)
                    {
                        b_contrast += 10;
                        set_contrast(&std_contrast, &std_contrast, &b_contrast);
                        SEGGER_RTT_printf(0, "\r\t Increase blue contrast to %d\n", b_contrast);
                    }
                    else if (button_down == 0)
                    {
                        b_contrast -= 10;
                        set_contrast(&std_contrast, &std_contrast, &b_contrast);
                        SEGGER_RTT_printf(0, "\r\t Decrease blue contrast to %d\n", b_contrast);
                    }
                    else if (button_sel == 1)
                    {
                        state = 5;
                        prev_state = 4;
                        memcpy(blind_type, tritanopia, sizeof(tritanopia));
                        SEGGER_RTT_printf(0, "\r\t Selected Tritanopia mode \n");
                    }
                        
                }
                break;
            }
            
                /*
                *      Case 5: Colour sensor mode
                *      Prev: Go back to calibration of selected mode
                *      Next: Go to Normal vision selection
                *      Up: does nothing
                *      Down: does nothing
                *      Select: Detects and adjusts colour at colour sensor
                */
                
            case 5:
            {
                while(state == 5)
                {
                    OSA_TimeDelay(100);
                    button_next = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 1)); //normally high
                    button_sel = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 5)); //normally low
                    button_prev = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 7)); //normally high
                    if (button_sel == 1)
                    {
                        SEGGER_RTT_printf(0, "\r\t Initiated colour conversion \n");
                        getRawData(&deviceTCS34725State, &readSensorRegisterTCS34725,
                                   &sensor_raw[0], &sensor_raw[1], &sensor_raw[2]);
                        matrix_vector_mult(blind_type, sensor_raw, sensor_adjusted_float);
                        float2colour(sensor_adjusted_float, sensor_adjusted, 3);
                        SEGGER_RTT_printf(0, "\n\r\t Filtered OLED values\n\r\t Red: %d\n\r\t Green: %d\n\r\t Blue: %d\n\r\t", sensor_adjusted[0],  sensor_adjusted[1], sensor_adjusted[2]);
                        rectangle(&sensor_adjusted[0], &sensor_adjusted[1], &sensor_adjusted[2], &width, 0);
                    }
                    else if (button_prev == 0)
                    {
                        state = prev_state;
                        SEGGER_RTT_printf(0, "\r\t Going back to calibration of selected mode\n");
                    }
                    else if (button_next == 0)
                    {
                        state = 1;
                        SEGGER_RTT_printf(0, "\r\t Returning to normal vision\n");
                    }
                }
            }
        }
    }
                
    return 0;
}

void
getRawData(volatile WarpI2CDeviceState *  i2cDeviceState,
           WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister,  int numberOfBytes),
            uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t clear, r_raw, b_raw, g_raw;
   
    WarpStatus status;
    
    enableI2Cpins(32768);
    
    status = readSensorRegisterFunction(0x14, 2 /* numberOfBytes */);
    if (status == kWarpStatusOK)
    {
        clear = (i2cDeviceState->i2cBuffer[1]<<8) + i2cDeviceState->i2cBuffer[0];
    }
    
    status = readSensorRegisterFunction(0x16, 2 /* numberOfBytes */);
    if (status == kWarpStatusOK)
    {
        r_raw = (i2cDeviceState->i2cBuffer[1]<<8) + i2cDeviceState->i2cBuffer[0];
        
    }
    
    status = readSensorRegisterFunction(0x18, 2 /* numberOfBytes */);
    if (status == kWarpStatusOK)
    {
        g_raw = (i2cDeviceState->i2cBuffer[1]<<8) + i2cDeviceState->i2cBuffer[0];
    }
    
    status = readSensorRegisterFunction(0x1a, 2 /* numberOfBytes */);
    if (status == kWarpStatusOK)
    {
        b_raw = (i2cDeviceState->i2cBuffer[1]<<8) + i2cDeviceState->i2cBuffer[0];
    }
    uint32_t sum;
    sum = r_raw + b_raw + g_raw;
    
    float r_ = ceil( ((2*(float)r_raw / (float)sum) - 0.333) * 63.0 );
    float g_ = ceil( ((2*(float)g_raw / (float)sum) - 0.333) * 63.0 );
    float b_ = ceil( ((2*(float)b_raw / (float)sum) - 0.333) * 63.0 );
    
    float2colour(&r_, r, 1);
    float2colour(&g_, g, 1);
    float2colour(&b_, b, 1);
    
    SEGGER_RTT_printf(0, "\n\r\t Unfiltered OLED values\n\r\t Red: %d\n\r\t Green: %d\n\r\t Blue: %d\n", *r, *g, *b);
}

