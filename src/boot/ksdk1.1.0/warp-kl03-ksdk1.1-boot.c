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



#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
volatile WarpI2CDeviceState            deviceMMA8451QState;
#endif




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
kWarpPinButton1                 = GPIO_MAKE_PIN(HW_GPIOB, 10);       //button pins for final project

kWarpPinButton2                 = GPIO_MAKE_PIN(HW_GPIOB, 7);

kWarpPinButton3                 = GPIO_MAKE_PIN(HW_GPIOB, 6);
*/


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

void                    sleepUntilReset(void);
void                    lowPowerPinStates(void);
void                    disableTPS82740A(void);
void                    disableTPS82740B(void);
void                    enableTPS82740A(uint16_t voltageMillivolts);
void                    enableTPS82740B(uint16_t voltageMillivolts);
void                    setTPS82740CommonControlLines(uint16_t voltageMillivolts);
void                    printPinDirections(void);
void                    dumpProcessorState(void);
void                    repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
                                uint16_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty,
                                int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
                                uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
void                    getRawData(volatile WarpI2CDeviceState *  i2cDeviceState,
                                WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister,  int numberOfBytes),
                                uint8_t *  r, uint8_t *  g, uint8_t *  b);
int                    char2int(int character);
void                    enableSssupply(uint16_t voltageMillivolts);
void                    disableSssupply(void);
uint8_t                    readHexByte(void);
int                    read4digits(void);
void                    printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, int i2cPullupValue);


/*
 *    TODO: change the following to take byte arrays
 */
WarpStatus                writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus                writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void                    warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);

uint8_t state = 1;
uint8_t r_contrast = 127;
uint8_t g_contrast = 127;
uint8_t b_contrast = 127;
uint8_t std_contrast = 127;
uint8_t all_contrast = 127;
uint8_t next_state = 2;

float blind_type[3][3];

uint8_t sensor_raw[3];
float sensor_adjusted_float[3];
uint8_t sensor_adjusted[3];

uint8_t width = 96;

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

/*
 Function for processing colour values
 */


/*
 *    Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
    if (RTC_DRV_IsAlarmPending(0))
    {
        RTC_DRV_SetAlarmIntCmd(0, false);
    }
}

/*
 *    Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
    gWarpSleeptimeSeconds++;
}

/*
 *    Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
                    power_manager_callback_data_t *  dataPtr)
{
    WarpPowerManagerCallbackStructure *        callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
    power_manager_error_code_t            status = kPowerManagerError;

    switch (notify->notifyType)
    {
        case kPowerManagerNotifyBefore:
            status = kPowerManagerSuccess;
            break;
        case kPowerManagerNotifyAfter:
            status = kPowerManagerSuccess;
            break;
        default:
            callbackUserData->errorCount++;
            break;
    }

    return status;
}

/*
 *    From KSDK power_manager_demo.c <<END>>>
 */



void
sleepUntilReset(void)
{
    while (1)
    {
#ifdef WARP_BUILD_ENABLE_DEVSI4705
        GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
#endif
        warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);
#ifdef WARP_BUILD_ENABLE_DEVSI4705
        GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
#endif
        warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
    }
}


void
enableLPUARTpins(void)
{
    /*    Enable UART CLOCK */
    CLOCK_SYS_EnableLpuartClock(0);

    /*
    *    set UART pin association
    *    see page 99 in https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
    */

#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    /*    Warp KL03_UART_HCI_TX    --> PTB3 (ALT3)    --> PAN1326 HCI_RX */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
    /*    Warp KL03_UART_HCI_RX    --> PTB4 (ALT3)    --> PAN1326 HCI_RX */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

    /* TODO: Partial Implementation */
    /*    Warp PTA6 --> PAN1326 HCI_RTS */
    /*    Warp PTA7 --> PAN1326 HCI_CTS */
#endif

    /*
     *    Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
     *
     */
    lpuartUserConfig.baudRate = 115;
    lpuartUserConfig.parityMode = kLpuartParityDisabled;
    lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
    lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;

    LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);

}


void
disableLPUARTpins(void)
{
    /*
     *    LPUART deinit
     */
    LPUART_DRV_Deinit(0);

    /*    Warp KL03_UART_HCI_RX    --> PTB4 (GPIO)    */
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);
    /*    Warp KL03_UART_HCI_TX    --> PTB3 (GPIO) */
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_HCI_CTS);
    GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_HCI_CTS);
#endif

    GPIO_DRV_ClearPinOutput(kWarpPinLPUART_HCI_TX);
    GPIO_DRV_ClearPinOutput(kWarpPinLPUART_HCI_RX);

    /* Disable LPUART CLOCK */
    CLOCK_SYS_DisableLpuartClock(0);

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



void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
    /*
     *    Set all pins into low-power states. We don't just disable all pins,
     *    as the various devices hanging off will be left in higher power draw
     *    state. And manuals say set pins to output to reduce power.
     */
    if (forceAllPinsIntoLowPowerState)
    {
        lowPowerPinStates();
    }

    warpSetLowPowerMode(kWarpPowerModeVLPR, 0);
    warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
}



void
printPinDirections(void)
{
    /*
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
    SEGGER_RTT_printf(0, "KL03_VDD_ADC:%d\n", GPIO_DRV_GetPinDir(kWarpPinKL03_VDD_ADC));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_VSEL2:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740_VSEL2));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "kWarpPinPAN1326_nSHUTD:%d\n", GPIO_DRV_GetPinDir(kWarpPinPAN1326_nSHUTD));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_CTLEN:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740A_CTLEN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740B_CTLEN:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740B_CTLEN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_VSEL1:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740_VSEL1));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TPS82740A_VSEL3:%d\n", GPIO_DRV_GetPinDir(kWarpPinTPS82740_VSEL3));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "CLKOUT32K:%d\n", GPIO_DRV_GetPinDir(kWarpPinCLKOUT32K));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "TS5A3154_IN:%d\n", GPIO_DRV_GetPinDir(kWarpPinTS5A3154_IN));
    OSA_TimeDelay(100);
    SEGGER_RTT_printf(0, "SI4705_nRST:%d\n", GPIO_DRV_GetPinDir(kWarpPinSI4705_nRST));
    OSA_TimeDelay(100);
#endif
    */
}



void
dumpProcessorState(void)
{
/*
    uint32_t    cpuClockFrequency;
    CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
    SEGGER_RTT_printf(0, "\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
    SEGGER_RTT_printf(0, "\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
    SEGGER_RTT_printf(0, "\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
    SEGGER_RTT_printf(0, "\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
    SEGGER_RTT_printf(0, "\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
    SEGGER_RTT_printf(0, "\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
#endif
*/
}

#ifdef WARP_BUILD_ENABLE_THERMALCHAMBERANALYSIS
void
addAndMultiplicationBusyLoop(long iterations)
{
    int value;
    for (volatile long i = 0; i < iterations; i++)
    {
        value = kWarpThermalChamberBusyLoopAdder + value * kWarpThermalChamberBusyLoopMutiplier;
    }
}

uint8_t
checkSum(uint8_t *  pointer, uint16_t length) /*    Adapted from https://stackoverflow.com/questions/31151032/writing-an-8-bit-checksum-in-c    */
{
    unsigned int sum;
    for ( sum = 0 ; length != 0 ; length-- )
    {
        sum += *(pointer++);
    }
    return (uint8_t)sum;
}
#endif

int
main(void)
{
    uint8_t                    key;
    WarpSensorDevice            menuTargetSensor = kWarpSensorBMX055accel;
    volatile WarpI2CDeviceState *        menuI2cDevice = NULL;
    uint16_t                menuI2cPullupValue = 32768;
    uint8_t                    menuRegisterAddress = 0x00;
    uint16_t                menuSupplyVoltage = 0;


    rtc_datetime_t                warpBootDate;

    power_manager_user_config_t        warpPowerModeWaitConfig;
    power_manager_user_config_t        warpPowerModeStopConfig;
    power_manager_user_config_t        warpPowerModeVlpwConfig;
    power_manager_user_config_t        warpPowerModeVlpsConfig;
    power_manager_user_config_t        warpPowerModeVlls0Config;
    power_manager_user_config_t        warpPowerModeVlls1Config;
    power_manager_user_config_t        warpPowerModeVlls3Config;
    power_manager_user_config_t        warpPowerModeRunConfig;

    const power_manager_user_config_t    warpPowerModeVlprConfig = {
                            .mode            = kPowerManagerVlpr,
                            .sleepOnExitValue    = false,
                            .sleepOnExitOption    = false
                        };

    power_manager_user_config_t const *    powerConfigs[] = {
                            /*
                             *    NOTE: This order is depended on by POWER_SYS_SetMode()
                             *
                             *    See KSDK13APIRM.pdf Section 55.5.3
                             */
                            &warpPowerModeWaitConfig,
                            &warpPowerModeStopConfig,
                            &warpPowerModeVlprConfig,
                            &warpPowerModeVlpwConfig,
                            &warpPowerModeVlpsConfig,
                            &warpPowerModeVlls0Config,
                            &warpPowerModeVlls1Config,
                            &warpPowerModeVlls3Config,
                            &warpPowerModeRunConfig,
                        };

    WarpPowerManagerCallbackStructure            powerManagerCallbackStructure;

    /*
     *    Callback configuration structure for power manager
     */
    const power_manager_callback_user_config_t callbackCfg0 = {
                            callback0,
                            kPowerManagerCallbackBeforeAfter,
                            (power_manager_callback_data_t *) &powerManagerCallbackStructure};

    /*
     *    Pointers to power manager callbacks.
     */
    power_manager_callback_user_config_t const *    callbacks[] = {
                                &callbackCfg0
                        };



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


    SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp, in 3... ");
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
     *    Set initial date to 1st January 2016 00:00, and set date via RTC driver
     */
    warpBootDate.year    = 2016U;
    warpBootDate.month    = 1U;
    warpBootDate.day    = 1U;
    warpBootDate.hour    = 0U;
    warpBootDate.minute    = 0U;
    warpBootDate.second    = 0U;
    RTC_DRV_SetDatetime(0, &warpBootDate);


    //Setup gpio input pin
    //GPIO_DRV_InputPinInit(&inputpin);

    /*
     *    Setup Power Manager Driver
     */
    memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));


    warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;
    
    warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpsConfig.mode = kPowerManagerVlps;
    
    warpPowerModeWaitConfig = warpPowerModeVlprConfig;
    warpPowerModeWaitConfig.mode = kPowerManagerWait;
    
    warpPowerModeStopConfig = warpPowerModeVlprConfig;
    warpPowerModeStopConfig.mode = kPowerManagerStop;

    warpPowerModeVlls0Config = warpPowerModeVlprConfig;
    warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

    warpPowerModeVlls1Config = warpPowerModeVlprConfig;
    warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

    warpPowerModeVlls3Config = warpPowerModeVlprConfig;
    warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

    warpPowerModeRunConfig.mode = kPowerManagerRun;

    POWER_SYS_Init(    &powerConfigs,
            sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
            &callbacks,
            sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
            );



    /*
     *    Switch CPU to Very Low Power Run (VLPR) mode
     */
    warpSetLowPowerMode(kWarpPowerModeVLPR, 0);



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
     *    Initialize all the sensors
     */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
    initBMX055accel(0x18    /* i2cAddress */,    &deviceBMX055accelState    );
    initBMX055gyro(    0x68    /* i2cAddress */,    &deviceBMX055gyroState    );
    initBMX055mag(    0x10    /* i2cAddress */,    &deviceBMX055magState    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
    initMMA8451Q(    0x1C    /* i2cAddress */,    &deviceMMA8451QState    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVLPS25H
    initLPS25H(    0x5C    /* i2cAddress */,    &deviceLPS25HState    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVHDC1000
    initHDC1000(    0x43    /* i2cAddress */,    &deviceHDC1000State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVMAG3110
    initMAG3110(    0x0E    /* i2cAddress */,    &deviceMAG3110State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVSI7021
    initSI7021(    0x40    /* i2cAddress */,    &deviceSI7021State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVL3GD20H
    initL3GD20H(    0x6A    /* i2cAddress */,    &deviceL3GD20HState    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVBME680
    initBME680(    0x77    /* i2cAddress */,    &deviceBME680State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVTCS34725
    initTCS34725(    0x29    /* i2cAddress */,    &deviceTCS34725State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVSI4705
    initSI4705(    0x11    /* i2cAddress */,    &deviceSI4705State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVCCS811
    initCCS811(    0x5A    /* i2cAddress */,    &deviceCCS811State    );
#endif
    
#ifdef WARP_BUILD_ENABLE_DEVAMG8834
    initAMG8834(    0x68    /* i2cAddress */,    &deviceAMG8834State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVAS7262
    initAS7262(    0x49    /* i2cAddress */,    &deviceAS7262State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVAS7263
    initAS7263(    0x49    /* i2cAddress */,    &deviceAS7263State    );
#endif

#ifdef WARP_BUILD_ENABLE_DEVSSD1331
    devSSD1331init();
#endif

#ifdef WARP_BUILD_ENABLE_DEVRV8803C7
  initRV8803C7(0x32 /* i2cAddress */, &deviceRV8803C7State);
  enableI2Cpins(menuI2cPullupValue);
  setRTCCountdownRV8803C7(0, TD_1HZ, false);
  disableI2Cpins();
#endif

    /*
     *    Initialization: Devices hanging off SPI
     */
#ifdef WARP_BUILD_ENABLE_DEVADXL362
    initADXL362(&deviceADXL362State);
#endif


    /*
     *    Initialization: the PAN1326, generating its 32k clock
     */
#ifdef WARP_BUILD_ENABLE_DEVPAN1326
    initPAN1326B(&devicePAN1326BState);
#endif



    /*
     *    Make sure SCALED_SENSOR_SUPPLY is off.
     *
     *    (There's no point in calling activateAllLowPowerSensorModes())
     */
    disableSssupply();


    /*
     *    TODO: initialize the kWarpPinKL03_VDD_ADC, write routines to read the VDD and temperature
     */




#ifdef WARP_BUILD_BOOT_TO_CSVSTREAM
    /*
     *    Force to printAllSensors
     */
    gWarpI2cBaudRateKbps = 300;
    warpSetLowPowerMode(kWarpPowerModeRUN, 0 /* sleep seconds : irrelevant here */);
    enableSssupply(3000);
    enableI2Cpins(menuI2cPullupValue);
    printAllSensors(false /* printHeadersAndCalibration */, false /* hexModeFlag */, 0 /* menuDelayBetweenEachRun */, menuI2cPullupValue);
    /*
     *    Notreached
     */
#endif




    while (1)
    {
        /*
         *    Do not, e.g., lowPowerPinStates() on each iteration, because we actually
         *    want to use menu to progressiveley change the machine state with various
         *    commands.
         */

        /*
         *    We break up the prints with small delays to allow us to use small RTT print
         *    buffers without overrunning them when at max CPU speed.
         */
        SEGGER_RTT_WriteString(0, "\r\n\n\n\n[ *\t\t\t\tW\ta\tr\tp\t(rev. b)\t\t\t* ]\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
                                menuSupplyVoltage, menuRegisterAddress);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_printf(0, "\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%dkb/s,\tI2C Pull-Up=%d\n\n",
                                gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateKbps, menuI2cPullupValue);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_printf(0, "\r\tSIM->SCGC6=0x%02x\t\tRTC->SR=0x%02x\t\tRTC->TSR=0x%02x\n", SIM->SCGC6, RTC->SR, RTC->TSR);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_printf(0, "\r\tMCG_C1=0x%02x\t\t\tMCG_C2=0x%02x\t\tMCG_S=0x%02x\n", MCG_C1, MCG_C2, MCG_S);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_printf(0, "\r\tMCG_SC=0x%02x\t\t\tMCG_MC=0x%02x\t\tOSC_CR=0x%02x\n", MCG_SC, MCG_MC, OSC_CR);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_printf(0, "\r\tSMC_PMPROT=0x%02x\t\t\tSMC_PMCTRL=0x%02x\t\tSCB->SCR=0x%02x\n", SMC_PMPROT, SMC_PMCTRL, SCB->SCR);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_printf(0, "\r\tPMC_REGSC=0x%02x\t\t\tSIM_SCGC4=0x%02x\tRTC->TPR=0x%02x\n\n", PMC_REGSC, SIM_SCGC4, RTC->TPR);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

        SEGGER_RTT_printf(0, "\r\t%ds in RTC Handler to-date,\t%d Pmgr Errors\n", gWarpSleeptimeSeconds, powerManagerCallbackStructure.errorCount);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
#else
        SEGGER_RTT_WriteString(0, "\r\n\n\t\tWARNING: SEGGER_RTT_printf disabled in this firmware build.\n\t\tOnly showing output that does not require value formatting.\n\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
#endif

        SEGGER_RTT_WriteString(0, "\rSelect:\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        SEGGER_RTT_WriteString(0, "\r- 'a': set default sensor.\n");
        SEGGER_RTT_WriteString(0, "\r- 'g': set default SSSUPPLY.\n");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, "\r- 'j': repeat read reg 0x%02x on sensor #%d.\n", menuRegisterAddress, menuTargetSensor);
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
#endif

        SEGGER_RTT_WriteString(0, "\rEnter selection> ");
        OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
        key = SEGGER_RTT_WaitKey();

        switch (key)
        {
            /*
             *        Select sensor
             */
            case 'a':
            {
                SEGGER_RTT_WriteString(0, "\r\tSelect:\n");


#ifdef WARP_BUILD_ENABLE_DEVTCS34725
                SEGGER_RTT_WriteString(0, "\r\t- 'd' TCS34725            (0x00--0x1D): 2.7V -- 3.3V\n");
                #else
                SEGGER_RTT_WriteString(0, "\r\t- 'd' TCS34725            (0x00--0x1D): 2.7V -- 3.3V (compiled out) \n");
#endif
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

                SEGGER_RTT_WriteString(0, "\r\tEnter selection> ");
                OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

                key = SEGGER_RTT_WaitKey();

                switch(key)
                {


#ifdef WARP_BUILD_ENABLE_DEVTCS34725
                    case 'd':
                    {
                        menuTargetSensor = kWarpSensorTCS34725;
                        menuI2cDevice = &deviceTCS34725State;
                        break;
                    }
#endif
                    default:
                    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                        SEGGER_RTT_printf(0, "\r\tInvalid selection '%c' !\n", key);
#endif
                    }
                }

                break;
            }

            /*
             *    Configure default TPS82740 voltage
             */
            case 'g':
            {
                SEGGER_RTT_WriteString(0, "\r\n\tOverride SSSUPPLY in mV (e.g., '1800')> ");
                menuSupplyVoltage = read4digits();
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                SEGGER_RTT_printf(0, "\r\n\tOverride SSSUPPLY set to %d mV", menuSupplyVoltage);
#endif

                break;
            }

            /*
             *    Start repeated read
             */
            case 'j':
            {
                bool        autoIncrement, chatty;
                int        spinDelay, repetitionsPerAddress, chunkReadsPerAddress;
                int        adaptiveSssupplyMaxMillivolts;
                uint8_t        referenceByte;

#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                SEGGER_RTT_printf(0, "\r\n\tAuto-increment from base address 0x%02x? ['0' | '1']> ", menuRegisterAddress);
                #else
                SEGGER_RTT_WriteString(0, "\r\n\tChunk reads per address (e.g., '1')> ");
#endif

                autoIncrement = SEGGER_RTT_WaitKey() - '0';

                SEGGER_RTT_WriteString(0, "\r\n\tChunk reads per address (e.g., '1')> ");
                chunkReadsPerAddress = SEGGER_RTT_WaitKey() - '0';

                SEGGER_RTT_WriteString(0, "\r\n\tChatty? ['0' | '1']> ");
                chatty = SEGGER_RTT_WaitKey() - '0';

                SEGGER_RTT_WriteString(0, "\r\n\tInter-operation spin delay in milliseconds (e.g., '0000')> ");
                spinDelay = read4digits();

                SEGGER_RTT_WriteString(0, "\r\n\tRepetitions per address (e.g., '0000')> ");
                repetitionsPerAddress = read4digits();

                SEGGER_RTT_WriteString(0, "\r\n\tMaximum voltage for adaptive supply (e.g., '0000')> ");
                adaptiveSssupplyMaxMillivolts = read4digits();

                SEGGER_RTT_WriteString(0, "\r\n\tReference byte for comparisons (e.g., '3e')> ");
                referenceByte = readHexByte();

#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                SEGGER_RTT_printf(0, "\r\n\tRepeating dev%d @ 0x%02x, reps=%d, pull=%d, delay=%dms:\n\n",
                    menuTargetSensor, menuRegisterAddress, repetitionsPerAddress, menuI2cPullupValue, spinDelay);
#endif

                repeatRegisterReadForDeviceAndAddress(    menuTargetSensor /*warpSensorDevice*/,
                                    menuRegisterAddress /*baseAddress */,
                                    menuI2cPullupValue,
                                    autoIncrement /*autoIncrement*/,
                                    chunkReadsPerAddress,
                                    chatty,
                                    spinDelay,
                                    repetitionsPerAddress,
                                    menuSupplyVoltage,
                                    adaptiveSssupplyMaxMillivolts,
                                    referenceByte
                                );

                break;
            }

            case ' ':
            {
                uint32_t button_up, button_down, button_next, button_prev, button_sel;
                
                while(1)
                {
                    switch(state)
                    {
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
                                button_sel = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 5)); //normally low
                                if (button_next == 0)
                                {
                                    state = 2;
                                    SEGGER_RTT_printf(0, "\r\t State 1 --> State 2\n");
                                }
                                else if (button_up == 0)
                                {
                                    all_contrast += 10;
                                    set_contrast(&all_contrast, &all_contrast, &all_contrast);
                                    SEGGER_RTT_printf(0, "\r\t Contrast all: %d\n", all_contrast);
                                }
                                else if (button_down == 0)
                                {
                                    all_contrast -= 10;
                                    set_contrast(&all_contrast, &all_contrast, &all_contrast);
                                    SEGGER_RTT_printf(0, "\r\t Contrast all: %d\n", all_contrast);
                                }
                                else if (button_sel == 1)
                                {
                                    state = 5;
                                    memcpy(blind_type, normal, sizeof(normal));
                                    SEGGER_RTT_printf(0, "\r\t Selected normal mode\n");
                                }
                                    
                            }
                            break;
                        }
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
                                    SEGGER_RTT_printf(0, "\r\t Next: State 2 --> 3\n");
                                }
                                if (button_prev == 0)
                                {
                                    state = 1;
                                    SEGGER_RTT_printf(0, "\r\t Prev: State 2 --> 1\n");
                                }
                                else if (button_up == 0)
                                {
                                    g_contrast += 10;
                                    set_contrast(&r_contrast, &g_contrast, &b_contrast);
                                    SEGGER_RTT_printf(0, "\r\t increase green contrast --> %d\n", g_contrast);
                                }
                                else if (button_down == 0)
                                {
                                    g_contrast -= 10;
                                    set_contrast(&r_contrast, &g_contrast, &b_contrast);
                                    SEGGER_RTT_printf(0, "\r\t decrease green contrast --> %d\n", g_contrast);
                                }
                                else if (button_sel == 1)
                                {
                                    state = 5;
                                    memcpy(blind_type, deuteranopia, sizeof(deuteranopia));
                                    SEGGER_RTT_printf(0, "\r\t Select deuteranopia mode \n");
                                }
                                    
                            }
                            break;
                        }
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
                                    SEGGER_RTT_printf(0, "\r\t Next: State 3 --> 4\n");
                                }
                                if (button_prev == 0)
                                {
                                    state = 2;
                                    SEGGER_RTT_printf(0, "\r\t Prev: State 3 --> 2\n");
                                }
                                else if (button_up == 0)
                                {
                                    r_contrast += 10;
                                    set_contrast(&r_contrast, &g_contrast, &b_contrast);
                                    SEGGER_RTT_printf(0, "\r\t increase red contrast --> %d\n", r_contrast);
                                }
                                else if (button_down == 0)
                                {
                                    r_contrast -= 10;
                                    set_contrast(&r_contrast, &g_contrast, &b_contrast);
                                    SEGGER_RTT_printf(0, "\r\t decrease red contrast --> %d\n", r_contrast);
                                }
                                else if (button_sel == 1)
                                {
                                    state = 5;
                                    memcpy(blind_type, protanopia, sizeof(protanopia));
                                    SEGGER_RTT_printf(0, "\r\t Select protanopia mode \n");
                                }
                                    
                            }
                            break;
                        }
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
                                    SEGGER_RTT_printf(0, "\r\t Next: State 2 --> 1\n");
                                }
                                if (button_prev == 0)
                                {
                                    state = 3;
                                    SEGGER_RTT_printf(0, "\r\t Prev: State 2 --> 1\n");
                                }
                                else if (button_up == 0)
                                {
                                    b_contrast += 10;
                                    set_contrast(&r_contrast, &g_contrast, &b_contrast);
                                    SEGGER_RTT_printf(0, "\r\t increase blue contrast --> %d\n", b_contrast);
                                }
                                else if (button_down == 0)
                                {
                                    b_contrast -= 10;
                                    set_contrast(&r_contrast, &g_contrast, &b_contrast);
                                    SEGGER_RTT_printf(0, "\r\t decrease blue contrast --> %d\n", b_contrast);
                                }
                                else if (button_sel == 1)
                                {
                                    state = 5;
                                    memcpy(blind_type, tritanopia, sizeof(tritanopia));
                                    SEGGER_RTT_printf(0, "\r\t Select tritanopia mode \n");
                                }
                                    
                            }
                            break;
                        }
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
                                    SEGGER_RTT_printf(0, "\r\t Collecting sensor data \n");
                                    getRawData(&deviceTCS34725State, &readSensorRegisterTCS34725,
                                               &sensor_raw[0], &sensor_raw[1], &sensor_raw[2]);
                                    matrix_vector_mult(blind_type, sensor_raw, sensor_adjusted_float);
                                    float2colour(sensor_adjusted_float, sensor_adjusted, 3);
                                    rectangle(&sensor_adjusted[0], &sensor_adjusted[1], &sensor_adjusted[2], &width, 0);
                                }
                                else if (button_prev == 0)
                                {
                                    state = 2;
                                    SEGGER_RTT_printf(0, "\r\t State 3 --> State 2\n");
                                }
                                else if (button_next == 0)
                                {
                                    state = 1;
                                    SEGGER_RTT_printf(0, "\r\t State 3 --> State 1\n");
                                }
                            }
                        }
                    }
                }
                //colourblock(255, 0, 0);
                //OSA_TimeDelay(1000);
                //colourblock(0, 255, 0);
                //OSA_TimeDelay(1000);
                //colourblock(0, 0, 255);
                //OSA_TimeDelay(1000);
                //colourblock(0,0,0);
                //OSA_TimeDelay(2000);
                
                
                
                /*
                while(1)
                {
                    
                
                    getRawData(&deviceTCS34725State, &readSensorRegisterTCS34725, &r, &g, &b);
                    
                    //disableI2Cpins();
                    
                    //SEGGER_RTT_printf(0, "\n\r\t Red --> %05d\n", r);
                    //SEGGER_RTT_printf(0, "\r\t Green --> %05d\n", g);
                    //SEGGER_RTT_printf(0, "\r\t Blue --> %05d\n", b);
                    
                    
                    colourblock(r, g, b);
                    
                    OSA_TimeDelay(400);
                
                }
                 */
                /*
                int spec = spectrum(red, orange, yellow, green, blue, purple);
                
                OSA_TimeDelay(3000);
                
                
                
                matrix_vector_mult(*deuteranopia, red, red_res, 3, 3);
                float2colour(red_res, red2, 3);
                matrix_vector_mult(*deuteranopia, orange, orange_res, 3, 3);
                float2colour(orange_res, orange2, 3);
                matrix_vector_mult(*deuteranopia, yellow, yellow_res, 3, 3);
                float2colour(yellow_res, yellow2, 3);
                matrix_vector_mult(*deuteranopia, green, green_res, 3, 3);
                float2colour(green_res, green2, 3);
                matrix_vector_mult(*deuteranopia, blue, blue_res, 3, 3);
                float2colour(blue_res, blue2, 3);
                matrix_vector_mult(*deuteranopia, purple, purple_res, 3, 3);
                float2colour(purple_res, purple2, 3);
                
                int spec2 = spectrum(red2, orange2, yellow2, green2, blue2, purple2);
                OSA_TimeDelay(3000);
                
                SEGGER_RTT_printf(0, "\n\r\t Purple Segment:");
                SEGGER_RTT_printf(0, "\n\r\t Red --> %05d\n", purple2[0]);
                SEGGER_RTT_printf(0, "\r\t Green --> %05d\n", purple2[1]);
                SEGGER_RTT_printf(0, "\r\t Blue --> %05d\n", purple2[2]);
                
               uint32_t button_1, button_2, button_3, button_4, button_5;
                
                
                
                int add_sub = 1;
                
                
                while(1)
                {
                    
                    button_1 = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 7));
                    button_2 = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 6));
                    button_3 = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 1));
                    button_4 = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 2));
                    button_5 = GPIO_DRV_ReadPinInput(GPIO_MAKE_PIN(HW_GPIOB, 5));
                    
                    OSA_TimeDelay(500);
                    
                    SEGGER_RTT_printf(0, "\n\r\t Button 5 --> %d\n", button_5);
                    
                    
                    if ((button_4 == 0) && (button_1 == 0))
                    {
                        getRawData(&deviceTCS34725State, &readSensorRegisterTCS34725, &r, &g, &b);
                        SEGGER_RTT_printf(0, "\n\r\t Red --> %05d\n", r);
                        SEGGER_RTT_printf(0, "\r\t Green --> %05d\n", g);
                        SEGGER_RTT_printf(0, "\r\t Blue --> %05d\n", b);
                        
                        rectangle(&r, &g, &b, &width, 0);
                        
                        OSA_TimeDelay(1000);
                    }
                    else if (button_2 == 0)
                    {
                        g = (g + add_sub*1) & 0x3F;
                        rectangle(&r, &g, &b, &width, 0);
                        SEGGER_RTT_printf(0, "\n\r\t Button 2 --> %d\n r = %d, g = %d, b = %d", button_2, r, g, b);
                        OSA_TimeDelay(100);
                    }
                    else if (button_3 == 0)
                    {
                        b = (b + add_sub*1) & 0x3F;
                        rectangle(&r, &g, &b, &width, 0);
                        SEGGER_RTT_printf(0, "\n\r\t Button 3 --> %d\n r = %d, g = %d, b = %d", button_3, r, g, b);
                        OSA_TimeDelay(100);
                    }
                    else if (button_4 == 0)
                    {
                        add_sub *= -1;
                        SEGGER_RTT_printf(0, "\n\r\t Button 4 --> %d\n r = %d, g = %d, b = %d", button_4, r, g, b);
                        OSA_TimeDelay(500);
                    }
                    else if (button_1 == 0)
                    {
                        r = (r + add_sub*1) & 0x3F;
                        rectangle(&r, &g, &b, &width, 0);
                        SEGGER_RTT_printf(0, "\n\r\t Button 1 --> %d\n r = %d, g = %d, b = %d", button_1, r, g, b);
                        OSA_TimeDelay(100);
                    }
                    
                }
                
                */
                //enableI2Cpins(menuI2cPullupValue);
                
                
               /* while(1)
                {
                    //pins_in = GPIO_DRV_ReadPinInput(kWarpPinButton1);
                    //OSA_TimeDelay(1);
                    SEGGER_RTT_printf(0, "\r\t Pins in --> %32b\n", pins_in);
                }
                */
                   
            }

            /*
             *    Ignore naked returns.
             */
            case '\n':
            {
                SEGGER_RTT_WriteString(0, "\r\tPayloads make rockets more than just fireworks.");
                break;
            }

            default:
            {
                #ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                SEGGER_RTT_printf(0, "\r\tInvalid selection '%c' !\n", key);
                #endif
            }
        }
    }

    return 0;
}



void
printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, int i2cPullupValue)
{
    /*
     *    A 32-bit counter gives us > 2 years of before it wraps, even if sampling at 60fps
     */
    uint32_t    readingCount = 0;
    uint32_t    numberOfConfigErrors = 0;
    
    

    while(1)
    {

        if (menuDelayBetweenEachRun > 0)
        {
            OSA_TimeDelay(menuDelayBetweenEachRun);
        }

        readingCount++;
    }
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
        SEGGER_RTT_printf(0, "\n\r\t Clear --> %05d\n", clear);
    }
    
    status = readSensorRegisterFunction(0x16, 2 /* numberOfBytes */);
    if (status == kWarpStatusOK)
    {
        r_raw = (i2cDeviceState->i2cBuffer[1]<<8) + i2cDeviceState->i2cBuffer[0];
        SEGGER_RTT_printf(0, "\r\t Red --> %05d\n", r_raw);
        
    }
    
    status = readSensorRegisterFunction(0x18, 2 /* numberOfBytes */);
    if (status == kWarpStatusOK)
    {
        g_raw = (i2cDeviceState->i2cBuffer[1]<<8) + i2cDeviceState->i2cBuffer[0];
        SEGGER_RTT_printf(0, "\r\t Green --> %05d\n", g_raw);
    }
    
    status = readSensorRegisterFunction(0x1a, 2 /* numberOfBytes */);
    if (status == kWarpStatusOK)
    {
        b_raw = (i2cDeviceState->i2cBuffer[1]<<8) + i2cDeviceState->i2cBuffer[0];
        SEGGER_RTT_printf(0, "\r\t Blue --> %05d\n", b_raw);
    }
    uint32_t sum;
    sum = r_raw + b_raw + g_raw;
    
    float r_ = ceil( ((2*(float)r_raw / (float)sum) - 0.333) * 63.0 );
    float g_ = ceil( ((2*(float)g_raw / (float)sum) - 0.333) * 63.0 );
    float b_ = ceil( ((2*(float)b_raw / (float)sum) - 0.333) * 63.0 );
    
    float2colour(&r_, r, 1);
    float2colour(&g_, g, 1);
    float2colour(&b_, b, 1);
}

void
loopForSensor(    const char *  tagString,
        WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister, int numberOfBytes),
        volatile WarpI2CDeviceState *  i2cDeviceState,
        volatile WarpSPIDeviceState *  spiDeviceState,
        uint8_t  baseAddress,
        uint8_t  minAddress,
        uint8_t  maxAddress,
        int  repetitionsPerAddress,
        int  chunkReadsPerAddress,
        int  spinDelay,
        bool  autoIncrement,
        uint16_t  sssupplyMillivolts,
        uint8_t  referenceByte,
        uint16_t adaptiveSssupplyMaxMillivolts,
        bool  chatty
        )
{
    WarpStatus        status, TCS34725ConfigStatus;
    uint8_t            address = min(minAddress, baseAddress);
    int            readCount = repetitionsPerAddress + 1;
    int            nSuccesses = 0;
    int            nFailures = 0;
    int            nCorrects = 0;
    int            nBadCommands = 0;
    uint16_t        actualSssupplyMillivolts = sssupplyMillivolts;


    if (    (!spiDeviceState && !i2cDeviceState) ||
        (spiDeviceState && i2cDeviceState) )
    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
        SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
#endif
    }

    enableSssupply(actualSssupplyMillivolts);
    SEGGER_RTT_WriteString(0, tagString);

    /*
     *    Keep on repeating until we are above the maxAddress, or just once if not autoIncrement-ing
     *    This is checked for at the tail end of the loop.
     */
    
    
    TCS34725ConfigStatus = configureSensorTCS34725();//&deviceTCS34725State);
    if (TCS34725ConfigStatus != kWarpStatusOK)
    {
        SEGGER_RTT_printf(0, "\nError when reading from TCS34725 device");
    }
       
    while (true)
    {
        for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
        {
            status = readSensorRegisterFunction(address+j, 1 /* numberOfBytes */);
            if (status == kWarpStatusOK)
            {
                nSuccesses++;
                if (actualSssupplyMillivolts > sssupplyMillivolts)
                {
                    actualSssupplyMillivolts -= 100;
                    enableSssupply(actualSssupplyMillivolts);
                }

                if (spiDeviceState)
                {
                    if (referenceByte == spiDeviceState->spiSinkBuffer[2])
                    {
                        nCorrects++;
                    }

                    if (chatty)
                    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                        SEGGER_RTT_printf(0, "\r\t0x%02x --> [0x%02x 0x%02x 0x%02x]\n",
                            address+j,
                            spiDeviceState->spiSinkBuffer[0],
                            spiDeviceState->spiSinkBuffer[1],
                            spiDeviceState->spiSinkBuffer[2]);
#endif
                    }
                }
                else
                {
                    if (referenceByte == i2cDeviceState->i2cBuffer[0])
                    {
                        nCorrects++;
                    }

                    if (chatty)
                    {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                        switch (address+j)
                        {
                            case 0x14: case 0x16: case 0x18: case 0x1a:
                            {
                                SEGGER_RTT_printf(0, "\r\t0x%02x --> 0x%04x\n",
                                address+j,
                                (i2cDeviceState->i2cBuffer[1]<<8)
                                              + i2cDeviceState->i2cBuffer[0]);
                                break;
                            }
                            case 0x12:
                            {
                                if (i2cDeviceState->i2cBuffer[0] != 0x44)
                                {
                                    SEGGER_RTT_printf(0, "\r\t0x%02x --> 0x%02x (Device recognition Failed)\n",
                                    address+j,
                                    i2cDeviceState->i2cBuffer[0]);
                                }
                                else if (i2cDeviceState->i2cBuffer[0] == 0x44)
                                {
                                    SEGGER_RTT_printf(0, "\r\t0x%02x --> 0x%02x (Device recognition Success)\n",
                                    address+j,
                                    i2cDeviceState->i2cBuffer[0]);
                                }
                                break;
                            }
                                
                            default:
                                SEGGER_RTT_printf(0, "\r\t0x%02x --> 0x%02x\n",
                                address+j,
                                i2cDeviceState->i2cBuffer[0]);
                        }
                        
                        
#endif
                    }
                }
            }
            else if (status == kWarpStatusDeviceCommunicationFailed)
            {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
                SEGGER_RTT_printf(0, "\r\t0x%02x --> ----\n",
                    address+j);
#endif

                nFailures++;
                if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
                {
                    actualSssupplyMillivolts += 100;
                    enableSssupply(actualSssupplyMillivolts);
                }
            }
            else if (status == kWarpStatusBadDeviceCommand)
            {
                nBadCommands++;
            }

            if (spinDelay > 0)
            {
                OSA_TimeDelay(spinDelay);
            }
        }

        if (autoIncrement)
        {
            address++;
        }

        if (address > maxAddress || !autoIncrement)
        {
            /*
             *    We either iterated over all possible addresses, or were asked to do only
             *    one address anyway (i.e. don't increment), so we're done.
             */
            break;
        }
    }

    /*
     *    We intersperse RTT_printfs with forced delays to allow us to use small
     *    print buffers even in RUN mode.
     */
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
    SEGGER_RTT_printf(0, "\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));
    OSA_TimeDelay(50);
    SEGGER_RTT_printf(0, "\r\t%d/%d successes matched ref. value of 0x%02x.\n", nCorrects, nSuccesses, referenceByte);
    OSA_TimeDelay(50);
    SEGGER_RTT_printf(0, "\r\t%d bad commands.\n\n", nBadCommands);
    OSA_TimeDelay(50);
#endif


    return;
}



void
repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, uint16_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty, int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts, uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte)
{
    if (warpSensorDevice != kWarpSensorADXL362)
    {
        enableI2Cpins(pullupValue);
    }

    switch (warpSensorDevice)
    {
        
        case kWarpSensorTCS34725:
        {
            /*
             *    TCS34725: VDD 2.7V -- 3.3V
             */
#ifdef WARP_BUILD_ENABLE_DEVTCS34725
            loopForSensor(    "\r\nTCS34725:\n\r",        /*    tagString            */
                    &readSensorRegisterTCS34725,    /*    readSensorRegisterFunction    */
                    &deviceTCS34725State,        /*    i2cDeviceState            */
                    NULL,                /*    spiDeviceState            */
                    baseAddress,            /*    baseAddress            */
                    0x00,                /*    minAddress            */
                    0x1D,                /*    maxAddress            */
                    repetitionsPerAddress,        /*    repetitionsPerAddress        */
                    chunkReadsPerAddress,        /*    chunkReadsPerAddress        */
                    spinDelay,            /*    spinDelay            */
                    autoIncrement,            /*    autoIncrement            */
                    sssupplyMillivolts,        /*    sssupplyMillivolts        */
                    referenceByte,            /*    referenceByte            */
                    adaptiveSssupplyMaxMillivolts,    /*    adaptiveSssupplyMaxMillivolts    */
                    chatty                /*    chatty                */
                    );
            #else
            SEGGER_RTT_WriteString(0, "\r\n\tTCS34725 Read Aborted. Device Disabled :( ");
#endif
            break;
        }
        default:
        {
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
            SEGGER_RTT_printf(0, "\r\tInvalid warpSensorDevice [%d] passed to repeatRegisterReadForDeviceAndAddress.\n", warpSensorDevice);
#endif
        }
    }

    if (warpSensorDevice != kWarpSensorADXL362)
    {
        disableI2Cpins();
    }
}



int
char2int(int character)
{
    if (character >= '0' && character <= '9')
    {
        return character - '0';
    }

    if (character >= 'a' && character <= 'f')
    {
        return character - 'a' + 10;
    }

    if (character >= 'A' && character <= 'F')
    {
        return character - 'A' + 10;
    }

    return 0;
}



uint8_t
readHexByte(void)
{
    uint8_t        topNybble, bottomNybble;

    topNybble = SEGGER_RTT_WaitKey();
    bottomNybble = SEGGER_RTT_WaitKey();

    return (char2int(topNybble) << 4) + char2int(bottomNybble);
}



int
read4digits(void)
{
    uint8_t        digit1, digit2, digit3, digit4;
    
    digit1 = SEGGER_RTT_WaitKey();
    digit2 = SEGGER_RTT_WaitKey();
    digit3 = SEGGER_RTT_WaitKey();
    digit4 = SEGGER_RTT_WaitKey();

    return (digit1 - '0')*1000 + (digit2 - '0')*100 + (digit3 - '0')*10 + (digit4 - '0');
}



WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
    i2c_status_t    status;
    uint8_t        commandBuffer[1];
    uint8_t        payloadBuffer[1];
    i2c_device_t    i2cSlaveConfig =
            {
                .address = i2cAddress,
                .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    commandBuffer[0] = commandByte;
    payloadBuffer[0] = payloadByte;

    status = I2C_DRV_MasterSendDataBlocking(
                        0    /* instance */,
                        &i2cSlaveConfig,
                        commandBuffer,
                        (sendCommandByte ? 1 : 0),
                        payloadBuffer,
                        (sendPayloadByte ? 1 : 0),
                        gWarpI2cTimeoutMilliseconds);

    return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
    uint8_t        inBuffer[payloadLength];
    spi_status_t    status;
    
    enableSPIpins();
    status = SPI_DRV_MasterTransferBlocking(0        /* master instance */,
                        NULL        /* spi_master_user_config_t */,
                        payloadBytes,
                        inBuffer,
                        payloadLength    /* transfer size */,
                        1000        /* timeout in microseconds (unlike I2C which is ms) */);
    disableSPIpins();

    return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}



