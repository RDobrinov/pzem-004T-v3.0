/*
 * MIT License
 * 
 * Copyright (c) 2020 Rossen Dobrinov
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/*
 * pzem-004T-v3.0.h
 *
 * Interface library for PEACEFAIR PZEM-004T v3.0
 * 
 * Author: Rossen Dobrinov https://github.com/RDobrinov
 *
*/

#ifndef _PZEM004T_V3.0_H_
#define _PZEM004T_V3.0_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define _PZEM004T_USE_SS_               0       //Never have success to work with SoftwareSerial *** DISABLED ***

#ifndef _PZEM004T_DIAG_STATS_
#define _PZEM004T_DIAG_STATS_           0       //Collect diagnostic data
#endif

#ifndef _PZEM0004_USE_RTU_MODBUS_ADDR_
#define _PZEM0004_USE_RTU_MODBUS_ADDR_  1       //Use PZEM-004T RTU address for communication. Do not use device global address 0xF8
#endif

#ifndef _PZEM0004_WRITE_PROTECTION_
#define _PZEM0004_WRITE_PROTECTION_     1       //Write protected Holding Registers. More info in docs
#endif

#ifndef _PZEM0004_UPDATE_ON_GET_VALUE_
#define _PZEM0004_UPDATE_ON_GET_VALUE_  0       //Do not update values on get
#endif

#if _PZEM004T_USE_SS_
#include <SoftwareSerial.h>
#endif


#define PZEM_MODBUS_GLOBAL_ADDRESS  0xF8        // Global device address 0xF8
#define PZEM_MAX_FRAME              28          // Max response frame length
#define PZEM_MAX_CMD_FRAME_LENGTH   8           // Max command frame length
#define PZEM_FRAME_HEADER           3           // Response header frame length
#define PZEM_BYTES_IN_FRAME_POS     2           // Byte possition in received frame for bytes count in frame
#define PZEM_ERROR_POS_IN_FRAME     2           // Byte possition in received frame for error frame flag
#define PZEM_FRAME_TAIL             2           // Frame tail length ( CRC bytes )
#define PZEM_FRAME_OVERHEAD         (PZEM_FRAME_HEADER + PZEM_FRAME_TAIL)
#define PZEM_MAX_ALARM_THRESHOLD    23000       // Default alarm threshold value
#define PZEM_UINT32_VALUE           2           // Nubmer of input registers holding 32bit value
#define PZEM_UINT16_VALUE           1           // Nubmer of input registers holding 16bit value

//PZEM Input register addresses - Read only
#define PZEM_VOLTAGE_REG_ADDR       0x0000      // Voltage register
#define PZEM_CURRENT_REG_ADDR_LOW   0x0001      // Current register low
#define PZEM_CURRENT_REG_ADDR_HIGH  0x0002      // Current register high
#define PZEM_POWER_REG_ADDR_LOW     0x0003      // Power register low
#define PZEM_POWER_REG_ADDR_HIGH    0x0004      // Power register high
#define PZEM_ENERGY_REG_ADDR_LOW    0x0005      // Energy register low
#define PZEM_ENERGY_REG_ADDR_HIGH   0x0006      // Energy register high
#define PZEM_FREQUENCY_REG_ADDR     0x0007      // Frequency register
#define PZEM_POWERFACTOR_REG_ADDR   0x0008      // PF register
#define PZEM_ALARM_REG_ADDR         0x0009      // Alarm state register

//PZEM Holding register addresses - Writable. !!!Realy writable!!! 
//*** Keep in mind and read docs ***
#define PZEM_UNKNOWN_REG_ADDR       0x0000      // Unknown. Default value 0x0001
#define PZEM_ALARM_THRS_REG_ADDR    0x0001      // Power alarm threshold value - 0x59D8 default
#define PZEM_MODBUS_ADDR_REG_ADDR   0x0002      // PZEM Modbus RTU Address - 0x0001 default
#define PZEM_BAUDRATE_REG_ADDR      0x0003      // Communication interface baudrate - 0x2580 
#define PZEM_STD_VOLTAGE_REG_ADDR   0x0004      // STD Voltage - 0x0898 |
#define PZEM_STD_CURRENT_REG_ADDR   0x0005      // STD Current - 0x2710 |--> Values for calibration? Who knows...
#define PZEM_STD_POWER_REG_ADDR     0x0006      // STD Power - 0x55F0   |

//PZEM Commands
#define PZEM_CMD_READ_HOLD_REG      0x03        // Read holding register
#define PZEM_CMD_READ_INPUT_REG     0x04        // Read input register
#define PZEM_CMD_WRITE_HOLD_REG     0x06        // Write holding register
#define PZEM_CMD_CALIBRATION        0x41        // Device calibration
#define PZEM_CMD_RESET_ENERGY       0x42        // Reset energy walue
#define PZEM_CMD_NOP_COMMAND        0x80        // No command send to device. Not really PZEM command.

//PZEM Error states
#define PZEM_NO_ERROR               0x00
#define PZEM_ILLEGAL_FUNC_ERR       0x01
#define PZEM_ILLEGAL_ADDR_ERR       0x02
#define PZEM_ILLEGAL_DATA_ERR       0x03
#define PZEM_SLAVE_ERR              0x04
#define PZEM_SLAVE_CRC_ERR          0x10
#define PZEM_SLAVE_TIMEOUT_ERR      0x11
#define PZEM_DATA_NOT_UPDATED       0x12
#define PZEM_PARAMS_NOT_UPDATED     0x13

#define PZEM_UPDATE_INTERVAL        150         // Minimal time for update values from input registers
#define PZEM_READ_TIMEOUT           50          // Initial time for response
#define PZEM_BAUD_RATE              9600        // Communication speed for interface

using void_callback_f = void (*)();

typedef struct rawRegisters_t {
    uint16_t  InputReg[10];                     // Input register array
    uint16_t  HoldingReg[7];                    // Holding register array
};

typedef struct errorState_t {
    uint8_t rtuError;                           //Error state returned from RTU by error frame
    uint8_t inputError;                         //Error state after Input register reading
    uint8_t holdError;                          //Error state after Hold register reading
    uint8_t lastCMD;                            //Last executed command from RTU
    uint16_t lastRegAddr;                       //Last used RTU register address
};

typedef struct rcvCtl_t {
    uint8_t Bytes;                              // Expected number in response frame
    uint8_t receiveIndex;                       // Pointer to current receive buffer
    uint8_t receiveBuffer[PZEM_MAX_FRAME];      // Receive buffer
    uint64_t lastRcv;                           // Timestamp for last received byte from PZEM
    uint64_t lastUpdate;                        // Timestamp for last rawRegisters_t update
};

#if _PZEM004T_DIAG_STATS_                       // For error stats colection - default: Disabled
typedef struct diagStats_t {
    uint32_t  RTUTimeout;                       // Number of RTU timeouts
    uint32_t  RTUCRCFail;                       // Number of CRC failed frames
    uint32_t  RTUErrorFrame;                    // Number of received error frames from device
    uint32_t  RTUIllegalFunction;               // Number of invalid command send
    uint32_t  RTUIllegalAddress;                // Number of invalid address read/write
    uint32_t  RTUIllegalData;                   // Number of invalid data for register has send
    uint32_t  RTUSlaveError;                    // No idea how to test - send from PZEM in error frame
    uint8_t   lastCommandFrame[PZEM_MAX_CMD_FRAME_LENGTH];  // Last command frame to PZEM
    uint8_t   lastCommandFrameLength;           // Last comman frame length
    uint8_t   lastRTUResponce[PZEM_MAX_FRAME];  // Last response frame send from PZEM
    uint8_t   lastRTUResponceLength;            // Last response frame length
};
#endif //_PZEM004T_DIAG_STATS_

enum txStates { NOT_EXPIRED, NOT_NOP_COMMAND, READY };  // Transmit states

class PZEM004Tv30
{
    public:

        PZEM004Tv30(HardwareSerial* hwSerial, uint8_t modbus_addr=PZEM_MODBUS_GLOBAL_ADDRESS);
        #if _PZEM004T_USE_SS_
        PZEM004Tv30(uint8_t rxPin, uint8_t txPin, uint8_t modbus_addr=PZEM_MODBUS_GLOBAL_ADDRESS);
        #endif
        ~PZEM004Tv30();

        float getVoltage();
        float getCurrent();
        float getPower();
        float getEnergy(bool kwh=true);
        float getFrequency();
        float getPowerFactor(bool percentage=true);
        bool  isOverloaded();
        uint8_t getModbusRTUAddress();
        rawRegisters_t getRTURegisters();
        errorState_t getErrorState();
        
        bool setModbusRTUAddress(uint8_t modbus_addr);
        bool setPowerAlarmThreshold(uint16_t watts, void_callback_f cb = nullptr);
        bool resetEnergyValue();

        bool readInputRegister(uint16_t regAddress = 0x0000, uint16_t regCount = 0x000A);
        bool readHoldingRegister(uint16_t regAddress = 0x0000, uint16_t regCount = 0x0007);
    
        #if !(_PZEM0004_WRITE_PROTECTION_)
        bool setHoldRegister(uint16_t regAddress, uint16_t regValue, bool enable_writing = false);
        #endif
        
        void rtuCalibration();

        
        

    private:

        Stream* _serial;
        bool _softSerial;
        rawRegisters_t _rtuReg;
        errorState_t   _err;
        void_callback_f _alarmCallback;
      
        rcvCtl_t _rx;

        #if _PZEM004T_DIAG_STATS_
        diagStats_t _rtuDiag;
        #endif

        void _initDrv(uint8_t modbusRTUAddress);
        void _resetRx();
        txStates _getTxState();
        
        void _sendCommand(uint8_t modbusRTUCommand, uint16_t regAddress, uint16_t regValue, bool fullFrame=true);
        void _parseRxData();
        void _setError();
        
        bool _setHoldRegister(uint16_t regAddress, uint16_t regValue);
        
        bool _modbusCRC16(uint8_t *rtuFrame, uint8_t bytes, bool set=false);
};

#endif // _PZEM004T_V3.0_H_
