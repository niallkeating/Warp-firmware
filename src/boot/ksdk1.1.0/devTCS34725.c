/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
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
#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceTCS34725State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
initTCS34725(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
    deviceStatePointer->i2cAddress    = i2cAddress;
    deviceStatePointer->signalType    = (kWarpTypeMaskColor | kWarpTypeMaskTemperature);

    return;
}

WarpStatus
writeSensorRegisterTCS34725(uint8_t deviceRegister, uint8_t payload)
{
    uint8_t        payloadByte[1], commandByte[1];
    i2c_status_t    status1, status2;

    if (deviceRegister > 0x1D)
    {
        return kWarpStatusBadDeviceCommand;
    }

    i2c_device_t slave =
    {
        .address = deviceTCS34725State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    
    commandByte[0] = 0x80 + deviceRegister;
    payloadByte[0] = payload;

    status1 = I2C_DRV_MasterSendDataBlocking(
                            0 /* I2C peripheral instance */,
                            &slave,
                            commandByte,
                            1,
                            payloadByte,
                            1,
                            gWarpI2cTimeoutMilliseconds);

    if (status1 != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}


/*
 This can be called from the full implementation of the Warp Firmware boot file to read sensor data and configurations
 */

WarpStatus
readSensorRegisterTCS34725(uint8_t deviceRegister)
{
    /*
     *    From manual, page 17 (bottom): First write to specify register address, then read.
     *
     *    See fields of COMMAND register on page 18. We write the bit pattern 1000 0000
     *    to specify that this is a command write for subsequent repeated byte protocol
     *    transactions (and special function flags in lower nybble ignored).
     */
    
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status1, status2;
    
    i2c_device_t slave =
    {
        .address = deviceTCS34725State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    cmdBuf[0] = 0x80 + deviceRegister;
    
	switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x03: case 0x04:
        case 0x05: case 0x06: case 0x07: case 0x0c:
        case 0x0d: case 0x0f: case 0x12: case 0x13:
        {
            cmdBuf[0] = 0x80 + deviceRegister; //3 MSBs indicate only look at 'deviceregister'
            
            status1 = I2C_DRV_MasterSendDataBlocking(
                                    0 /* I2C peripheral instance */,
                                    &slave,
                                    cmdBuf,
                                    1,
                                    NULL,
                                    0,
                                    gWarpI2cTimeoutMilliseconds);

            status2 = I2C_DRV_MasterReceiveDataBlocking(
                                    0 /* I2C peripheral instance */,
                                    &slave,
                                    NULL,//cmdBuf,
                                    0,//1,
                                    (uint8_t *)deviceTCS34725State.i2cBuffer,
                                    1,
                                    gWarpI2cTimeoutMilliseconds);


            if ((status1 != kStatus_I2C_Success) || (status2 != kStatus_I2C_Success))
            {
                return kWarpStatusDeviceCommunicationFailed;
            }
            
            break;
        }
        /*
         cases to read clear, red, green, blue registers.
         Get 16 bit precision so need case to read twice
         */
        case 0x14: case 0x16: case 0x18: case 0x1a:
        
        {
            cmdBuf[0] = 0x80 + deviceRegister; //now autoincrement the registers (so MSBs relating to LSBs are always read)
            
            status1 = I2C_DRV_MasterSendDataBlocking(
                                    0 /* I2C peripheral instance */,
                                    &slave,
                                    cmdBuf,
                                    1,
                                    NULL,
                                    0,
                                    gWarpI2cTimeoutMilliseconds);

            status2 = I2C_DRV_MasterReceiveDataBlocking(
                                    0 /* I2C peripheral instance */,
                                    &slave,
                                    NULL,
                                    0,
                                    (uint8_t *)deviceTCS34725State.i2cBuffer,
                                    2,
                                    gWarpI2cTimeoutMilliseconds);


            if ((status1 != kStatus_I2C_Success) || (status2 != kStatus_I2C_Success))
            {
                return kWarpStatusDeviceCommunicationFailed;
            }
            
            break;
        }
        
        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

	

	return kWarpStatusOK;
}

WarpStatus
configureSensorTCS34725()//WarpI2CDeviceState volatile *  TCS34725DeviceState)
{
    //WarpStatus    i2cReadStatus;
    WarpStatus    i2cWriteStatus1, i2cWriteStatus2, i2cWriteStatus3, i2cWriteStatus4 ;
    
    //set integration time: 154ms = 0xC0 ; 700ms = 0x00 ; 400ms = 0x59
    //See TCS34725.pdf page 8 for how to calculate
    i2cWriteStatus3 = writeSensorRegisterTCS34725(0x01 /* register address ATIME */,
                            0x59 /* ATIME value */);
    
    
    i2cWriteStatus4 = writeSensorRegisterTCS34725(0x0F /* register address AGAIN */,
                            0x00 /* AGAIN value */);
    
    //Enable PON
    i2cWriteStatus1 = writeSensorRegisterTCS34725(0x00 /* register address ENABLE */,
                            0x01 /* ENABLE value */);
    
    OSA_TimeDelay(3); //2.4ms delay for warm up after PON enabled
    
    //Enable PON and AEN (ADC turn on)
    i2cWriteStatus2 = writeSensorRegisterTCS34725(0x00 /* register address ENABLE */,
    0x03 /* ENABLE value */);
    
    OSA_TimeDelay(700); //Maximum integration time such that no readings can be taken before these are set
    
    
    return (i2cWriteStatus1 | i2cWriteStatus2 | i2cWriteStatus3 | i2cWriteStatus4);
}
