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
 * pzem-004T-v3.0.cpp
 *
*/

#include "pzem-004T-v3.0.h"

/*!
 * PZEM004Tv30::PZEM004Tv30
 *
 * Hardware serial constructor
 *
 * @param hwSerial Hardware serial to use
 * @param modbus_addr RTU address of pzem device
*/
#if defined(ARDUINO_ARCH_ESP32)
PZEM004Tv30::PZEM004Tv30(HardwareSerial* hwSerial, int8_t rx_pin, int8_t tx_pin, uint8_t modbus_addr)
#else
PZEM004Tv30::PZEM004Tv30(HardwareSerial* hwSerial, uint8_t modbus_addr)
#endif
{
    this->_serial = hwSerial;
    this->_softSerial = false;
    #if defined(ARDUINO_ARCH_ESP32)
    if((rx_pin<0) || (tx_pin<0))
        static_cast<HardwareSerial*>(_serial)->begin(PZEM_BAUD_RATE);
    else
        static_cast<HardwareSerial*>(_serial)->begin(PZEM_BAUD_RATE, PZEM_UART_CONFIG, rx_pin, tx_pin);
    #else
    static_cast<HardwareSerial*>(_serial)->begin(PZEM_BAUD_RATE);
    #endif
    _initDrv(modbus_addr);
}

/*!
 * PZEM004Tv30::PZEM004Tv30
 *
 * Software serial constructor
 * ( not used ) No successful communication with RTU
 *
 * @param rxPin Hardware serial to use
 * @param txPin Hardware serial to use
 * @param modbus_addr RTU address of pzem device
*/
#if _PZEM004T_USE_SS_
PZEM004Tv30::PZEM004Tv30(uint8_t rxPin, uint8_t txPin, uint8_t modbus_addr)
{
    SoftwareSerial *port = new SoftwareSerial(rxPin, txPin);
    this->_serial = port;
    this->_softSerial = true;
    static_cast<SoftwareSerial*>(_serial)->begin(PZEM_BAUD_RATE);
    static_cast<SoftwareSerial*>(_serial)->listen();
    _initDrv(modbus_addr);
}
#endif

/*!
 * PZEM004Tv30::PZEM004Tv30
 *
 * Destrutor
 * Delete software serial.
 * ( not used ) No successful communication with RTU
*/
PZEM004Tv30::~PZEM004Tv30()
{
    if(_softSerial) delete this->_serial;
}

/*!
 * PZEM004Tv30::getVoltage
 *
 * Get last measured Voltage
 *
 * @return last measured voltage value in Volts with 0.1V resolution
*/
float PZEM004Tv30::getVoltage()
{
    #if _PZEM0004_UPDATE_ON_GET_VALUE_
    readInputRegister(PZEM_VOLTAGE_REG_ADDR, PZEM_UINT16_VALUE);
    #endif
    return (_err.inputError == PZEM_DATA_NOT_UPDATED) ? 0.0 : _rtuReg.InputReg[PZEM_VOLTAGE_REG_ADDR] / 10.0;
}

/*!
 * PZEM004Tv30::getCurrent
 *
 * Get last measured Current
 *
 * @return last measured power value in Amps with 0.001A resolution
*/
float PZEM004Tv30::getCurrent()
{
    #if _PZEM0004_UPDATE_ON_GET_VALUE_
    readInputRegister(PZEM_CURRENT_REG_ADDR_LOW, PZEM_UINT32_VALUE);
    #endif
    return (_err.inputError == PZEM_DATA_NOT_UPDATED) ? 0.0 : ((uint32_t)_rtuReg.InputReg[PZEM_CURRENT_REG_ADDR_HIGH]<<16 | (uint32_t)_rtuReg.InputReg[PZEM_CURRENT_REG_ADDR_LOW]) / 1000.0;
}

/*!
 * PZEM004Tv30::getPower
 *
 * Get last measured active power value in Watt with 0.1W resolution
 *
 * @return success
*/
float PZEM004Tv30::getPower()
{
    #if _PZEM0004_UPDATE_ON_GET_VALUE_
    readInputRegister(PZEM_POWER_REG_ADDR_LOW, PZEM_UINT32_VALUE);
    #endif
    return (_err.inputError == PZEM_DATA_NOT_UPDATED) ? 0.0 : ((uint32_t)_rtuReg.InputReg[PZEM_POWER_REG_ADDR_HIGH]<<16 | (uint32_t)_rtuReg.InputReg[PZEM_POWER_REG_ADDR_LOW]) / 10.0;
}

/*!
 * PZEM004Tv30::getEnergy
 *
 * Get last enegry value
 *
 * @param kwh Return value in kW/h - default: true
 * @return Last measured energy value in kWh or Wh with 1Wh resolution
*/
float PZEM004Tv30::getEnergy(bool kwh)
{
    #if _PZEM0004_UPDATE_ON_GET_VALUE_
    readInputRegister(PZEM_ENERGY_REG_ADDR_LOW, PZEM_UINT32_VALUE);
    #endif
    return (_err.inputError == PZEM_DATA_NOT_UPDATED) ? 0.0 : ((uint32_t)_rtuReg.InputReg[PZEM_ENERGY_REG_ADDR_HIGH]<<16 | (uint32_t)_rtuReg.InputReg[PZEM_ENERGY_REG_ADDR_LOW]) / (kwh ? 1000.0 : 1.0);
}

/*!
 * PZEM004Tv30::getFrequency
 *
 * Get last measured Frequency
 *
 * @return Last measured Frequency in Hz with 0.1Hz resolution
*/
float PZEM004Tv30::getFrequency()
{
    #if _PZEM0004_UPDATE_ON_GET_VALUE_
    readInputRegister(PZEM_FREQUENCY_REG_ADDR, PZEM_UINT16_VALUE);
    #endif
    return (_err.inputError == PZEM_DATA_NOT_UPDATED) ? 0.0 : _rtuReg.InputReg[PZEM_FREQUENCY_REG_ADDR] / 10.0;
}

/*!
 * PZEM004Tv30::getPowerFactor
 *
 * Get last measured power vctor
 *
 * @param percentage Return value in percents
 * @return Last measured power factor with 1% resolution
*/
float PZEM004Tv30::getPowerFactor(bool percentage)
{
    #if _PZEM0004_UPDATE_ON_GET_VALUE_
    readInputRegister(PZEM_POWERFACTOR_REG_ADDR, PZEM_UINT16_VALUE);
    #endif
    return (_err.inputError == PZEM_DATA_NOT_UPDATED) ? 0.0 : _rtuReg.InputReg[PZEM_POWERFACTOR_REG_ADDR] / (percentage ? 1.0 : 100.0);
}

/*!
 * PZEM004Tv30::isOverloaded
 *
 * Get last alarm status
 *
 * @return Alarm threshold reached true/false
*/
bool PZEM004Tv30::isOverloaded()
{
    #if _PZEM0004_UPDATE_ON_GET_VALUE_
    readInputRegister(PZEM_ALARM_REG_ADDR, PZEM_UINT16_VALUE);
    #endif
    return (_rtuReg.InputReg[PZEM_ALARM_REG_ADDR] != 0x0000);
}

/*!
 * PZEM004Tv30::getModbusRTUAddress
 *
 * Get current Modbus RTU address
 *
 * @return Current Modbus RTU address set/read
*/
uint8_t PZEM004Tv30::getModbusRTUAddress()
{
    return _rtuReg.HoldingReg[PZEM_MODBUS_ADDR_REG_ADDR];
}

/*!
 * PZEM004Tv30::getRTURegisters
 *
 * Get last register values read from PZEM device
 *
 * @return rawRegisters_t structure containing current PZEM register values 
*/
rawRegisters_t PZEM004Tv30::getRTURegisters()
{
    return _rtuReg;
}

/*!
 * PZEM004Tv30::getErrorState
 *
 * Get error states
 *
 * @return errorState_t structure containing internal error state 
*/
errorState_t PZEM004Tv30::getErrorState()
{
    return _err;
}

/*!
 * PZEM004Tv30::setModbusRTUAddress
 *
 * Set a new device RTU address
 * Update address in RTU, new address became active
 *
 * @param addr New device address 0x01-0xF7
 * @return success
*/
bool PZEM004Tv30::setModbusRTUAddress(uint8_t modbus_addr)
{
    if((modbus_addr < 0x01) || (modbus_addr > 0xF7)) return false;
    if(_rtuReg.HoldingReg[PZEM_MODBUS_ADDR_REG_ADDR] == modbus_addr) return true;
    if( !_setHoldRegister(PZEM_MODBUS_ADDR_REG_ADDR, (uint16_t)modbus_addr) ) return false;
    _rtuReg.HoldingReg[PZEM_MODBUS_ADDR_REG_ADDR] = modbus_addr;
    return true;
}

/*!
 * PZEM004Tv30::setPowerAlarmThreshold
 *
 * Set a new device Power alarm threshold 
 * and/or set/change callback function pointer
 *
 * @param watts New alarm threshold in Watts
 * @param Pointer to callback function - default: nullptr
 * @return success
*/
bool PZEM004Tv30::setPowerAlarmThreshold(uint16_t watts, void_callback_f cb)
{
    if( watts > PZEM_MAX_ALARM_THRESHOLD ) return false;
    if(cb) _alarmCallback = cb;
    if( !_setHoldRegister(PZEM_ALARM_THRS_REG_ADDR, watts) ) return false;
    _rtuReg.HoldingReg[PZEM_ALARM_THRS_REG_ADDR] = watts;
    return true;
}

/*!
 * PZEM004Tv30::resetEnergyValue
 *
 * Reset energy counter
 *
 * @return success
*/
bool PZEM004Tv30::resetEnergyValue()
{
    _sendCommand(PZEM_CMD_RESET_ENERGY, 0x00, 0x00, false);
    return (_err.inputError == PZEM_NO_ERROR);
}

/*!
 * PZEM004Tv30::readInputRegister
 *
 * Read input register value and parse received data
 * Default: Read whole input registers
 * Use carefully! regAddress+regCount must be less than 0x000B
 *
 * @param regAddress Register address where read start from - default: 0x0000
 * @param regCount Number of registers to read - default: 0x000A
 * @return success
*/
bool PZEM004Tv30::readInputRegister(uint16_t regAddress, uint16_t regCount)
{
    txStates state = _getTxState();
    if ( ( state != READY) ) return (state == NOT_EXPIRED);
    _sendCommand(PZEM_CMD_READ_INPUT_REG, regAddress, regCount);
    return (_err.inputError == PZEM_NO_ERROR);
}

/*!
 * PZEM004Tv30::readInputRegister
 *
 * Read holding register value and parse received data
 * Default: Read whole holding registers
 * Use carefully! regAddress+regCount must be less than 0x0008
 *
 * @param regAddress Register address where read start from - default: 0x0000
 * @param regCount Number of registers to read - default: 0x0007
 * @return success
*/
bool PZEM004Tv30::readHoldingRegister(uint16_t regAddress, uint16_t regCount)
{
    if ( _getTxState() == NOT_NOP_COMMAND ) return false;
    _sendCommand(PZEM_CMD_READ_HOLD_REG, regAddress, regCount);
    return (_err.holdError == PZEM_NO_ERROR);
}

/*!
 * PZEM004Tv30::setHoldRegister
 *
 * Set single holding register value - read docs!
 * *********** WRITE PROTECTED *************
 * *** Use carefully and don't be stupid ***
 * This function is only available wnen _PZEM0004_WRITE_PROTECTION_ is set to zero
 *
 * @param regAddress Register address where to write
 * @param regValue Value to write
 * @param enable_writing Override protection - defalut: false
 * @return success
*/
#if !(_PZEM0004_WRITE_PROTECTION_)
bool PZEM004Tv30::setHoldRegister(uint16_t regAddress, uint16_t regValue, bool enable_writing)
{
    if( ( ( regAddress < 0x0001 ) || ( regAddress > 0x0002 ) ) && ( !enable_writing ) ) return false;
    _sendCommand(PZEM_CMD_WRITE_HOLD_REG, regAddress, regValue);
    return (_err.inputError == PZEM_NO_ERROR);    
}
#endif //_PZEM0004_WRITE_PROTECTION_

/*!
 * PZEM004Tv30::rtuCalibration
 *
 * Not implemented due lack of information
*/
void PZEM004Tv30::rtuCalibration()
{
    return;
}

//Private

/*!
 * PZEM004Tv30::_initDrv
 *
 * Init driver values
 *
 * @param modbusRTUAddress Modbus RTU address
*/
void PZEM004Tv30::_initDrv(uint8_t modbusRTUAddress)
{
    memset(_rtuReg.InputReg, 0x00, sizeof(_rtuReg.InputReg));
    memset(_rtuReg.HoldingReg, 0x00, sizeof(_rtuReg.HoldingReg));
    
    if(!setModbusRTUAddress(modbusRTUAddress)) _rtuReg.HoldingReg[PZEM_MODBUS_ADDR_REG_ADDR]=PZEM_MODBUS_GLOBAL_ADDRESS;
    
    _err = {
        .rtuError = PZEM_NO_ERROR,
        .inputError = PZEM_DATA_NOT_UPDATED,
        .holdError = PZEM_PARAMS_NOT_UPDATED,
        .lastCMD = PZEM_CMD_NOP_COMMAND,
        .lastRegAddr = 0x0000
    };

    _alarmCallback = nullptr;
    _rx.lastUpdate = 0;
    _resetRx();
    
    #if _PZEM004T_DIAG_STATS_
    _rtuDiag = {
        .RTUTimeout = 0,
        .RTUCRCFail = 0,
        .RTUErrorFrame = 0,
        .RTUIllegalFunction = 0,
        .RTUIllegalAddress = 0,
        .RTUIllegalData = 0,
        .RTUSlaveError = 0,
        .lastCommandFrame = {0},
        .lastCommandFrameLength = 0,
        .lastRTUResponce = {0},
        .lastRTUResponceLength = 0,
    };
    #endif

    #if _PZEM0004_USE_RTU_MODBUS_ADDR_
    readHoldingRegister();
    #endif
}

/*!
 * PZEM004Tv30::_resetRx
 *
 * Reset RX queue
*/
void PZEM004Tv30::_resetRx()
{
    _err.lastCMD = PZEM_CMD_NOP_COMMAND;
    _rx.Bytes = 0;
    _rx.receiveIndex = 0;
    _rx.lastUpdate = millis();
}

/*!
 * PZEM004Tv30::_getTxState
 *
 * Get current TX state
 * @return TX state
*/
txStates PZEM004Tv30::_getTxState()
{
    #if !_PZEM0004_UPDATE_ON_GET_VALUE_
    if((millis()-_rx.lastUpdate) < PZEM_UPDATE_INTERVAL) return NOT_EXPIRED;
    #endif
    if(_err.lastCMD != PZEM_CMD_NOP_COMMAND) return NOT_NOP_COMMAND;               //*** Should never be here *** [Timeout expired but command is not in reset state]
    return READY;
}

/*!
 * PZEM004Tv30::_sendCommand
 *
 * Prepare 8 Bytes command frame and send it to slave device
 * Wait and receive response frame from RTU. 
 * Answer from RTU is received after 40 msec, except for Reset Energy command. Reset energy takes approx. 43-44 msec.
 *
 * Command Frame format ( full frame ). Total 8 bytes
 * +---------------+---------------+----------------------+----------------------+--------------------+--------------------+-----------+-----------+
 * |  RTU address  |  RTU Command  | Register Address MSB | Register Address LSB | Register Value MSB | Register Value LSB | CRC16 MSB | CRC16 LSB |
 * +---------------+---------------+----------------------+----------------------+--------------------+--------------------+-----------+-----------+
 *
 * Command Frame format ( short frame for calibration ). Total 6 bytes
 * +---------------+---------------+------+------+-----------+-----------+
 * |  RTU address  |  RTU Command  | 0x37 | 0x21 | CRC16 MSB | CRC16 LSB |
 * +---------------+---------------+------+------+-----------+-----------+
 *
 * Command Frame format ( short frame for energy reset ). Total 4 bytes
 * +---------------+---------------+-----------+-----------+
 * |  RTU address  |  RTU Command  | CRC16 MSB | CRC16 LSB |
 * +---------------+---------------+-----------+-----------+
 *
 * @param modbusRTUCommand Command to PZEM device
 * @param regAddress Internal PZEM register address
 * @param regValue Value to write in internal PZEM register
 * @param fullFrame Prepare full or short frame - default: full.
*/

void PZEM004Tv30::_sendCommand(uint8_t modbusRTUCommand, uint16_t regAddress, uint16_t regValue, bool fullFrame )
{
    uint8_t cmdFrame[PZEM_MAX_CMD_FRAME_LENGTH];
    uint8_t cmdFrameLength = PZEM_MAX_CMD_FRAME_LENGTH;

    uint8_t bytesAvailable;

    cmdFrame[0] = 
    #if _PZEM0004_USE_RTU_MODBUS_ADDR_
      _rtuReg.HoldingReg[PZEM_MODBUS_ADDR_REG_ADDR];
    #else
      PZEM_MODBUS_GLOBAL_ADDRESS;
    #endif
    
    cmdFrame[1] = modbusRTUCommand;

    cmdFrame[2] = ((regAddress >> 8) & 0xFF );
    cmdFrame[3] = ( regAddress & 0xFF );
    
    if(!fullFrame)
    {
        cmdFrameLength = (modbusRTUCommand == PZEM_CMD_RESET_ENERGY) ? 4 : 6;
    }
    else
    {
        cmdFrame[4] = ((regValue >> 8) & 0xFF );
        cmdFrame[5] = ( regValue & 0xFF );
    }

    _modbusCRC16(cmdFrame, cmdFrameLength, true);

    _serial->write(cmdFrame, cmdFrameLength);
    
    #if _PZEM004T_DIAG_STATS_
        memcpy(_rtuDiag.lastCommandFrame, cmdFrame, cmdFrameLength);
        _rtuDiag.lastCommandFrameLength = cmdFrameLength;
    #endif
    
    _rx.Bytes = (modbusRTUCommand==PZEM_CMD_READ_INPUT_REG) ? 25 : (modbusRTUCommand==PZEM_CMD_READ_HOLD_REG) ? 19 : (modbusRTUCommand==PZEM_CMD_RESET_ENERGY) ? 4 : 8;
    _err.lastCMD = modbusRTUCommand;
    _err.lastRegAddr = regAddress;

    _rx.lastRcv = millis();
    if( modbusRTUCommand == PZEM_CMD_READ_INPUT_REG ) 
        _rx.lastUpdate = millis();

    while( (_rx.receiveIndex < _rx.Bytes) && !((millis()-_rx.lastRcv) > PZEM_READ_TIMEOUT))
    {
        bytesAvailable = _serial->available();
        if( bytesAvailable > 0)
        {
            for( int bytesIndex=0; bytesIndex<bytesAvailable; bytesIndex++)
            {
                _rx.receiveBuffer[_rx.receiveIndex++] = (uint8_t)_serial->read();
            }
            _rx.lastRcv = millis();         //Reset timeout counter
        }
        if(_rx.receiveIndex > 2)
        {
            if( (0b10000000 & _rx.receiveBuffer[1]) != 0 )      //Responce is error frame
            {
                _rx.Bytes = 5;
            }
        } 
    }
    
    #if _PZEM004T_DIAG_STATS_
        memcpy(_rtuDiag.lastRTUResponce, _rx.receiveBuffer, _rx.receiveIndex);
        _rtuDiag.lastRTUResponceLength = _rx.receiveIndex;
    #endif
    
    if( (_rx.receiveIndex == _rx.Bytes) || (_rx.receiveIndex == (_rx.receiveBuffer[PZEM_BYTES_IN_FRAME_POS]+PZEM_FRAME_OVERHEAD)) )
    {
        if(_rx.receiveIndex != _rx.Bytes) _rx.Bytes = _rx.receiveBuffer[PZEM_BYTES_IN_FRAME_POS]+PZEM_FRAME_OVERHEAD;
        _parseRxData();
        return ;
    }

    if((millis()-_rx.lastRcv) > PZEM_READ_TIMEOUT)
    {
        #if _PZEM004T_DIAG_STATS_
        _rtuDiag.RTUTimeout++;
        #endif
        _resetRx();
        _err.rtuError = PZEM_SLAVE_TIMEOUT_ERR;
        _setError();
    }
}

/*!
 * PZEM004Tv30::_parseRxData
 *
 * Parse and populate received data from PZEM device
 *   
 * Response Frame format from input and holding registers. Total 2xNumber of registers + 5 Bytes
 * +---------------+---------------+------------------+-------------------+-------------------+~...~+-------------------+-------------------+-----------+-----------+
 * |  RTU address  |  RTU Command  | Payload in Bytes | Reg[+0] Value MSB | Reg[+0] Value LSB |~...~| Reg[+n] Value MSB | Reg[+n] Value LSB | CRC16 MSB | CRC16 LSB |
 * +---------------+---------------+------------------+-------------------+-------------------+~...~+-------------------+-------------------+-----------+-----------+
 *
 * Response confirmation frame format. Total 8 bytes
 * +---------------+---------------+----------------------+----------------------+--------------------+--------------------+-----------+-----------+
 * |  RTU address  |  RTU Command  | Register Address MSB | Register Address LSB | Register Value MSB | Register Value LSB | CRC16 MSB | CRC16 LSB |
 * +---------------+---------------+----------------------+----------------------+--------------------+--------------------+-----------+-----------+
 *
 * Response confirmation frame for energy reset. Total 4 bytes
 * +---------------+---------------+-----------+-----------+
 * |  RTU address  |  RTU Command  | CRC16 MSB | CRC16 LSB |
 * +---------------+---------------+-----------+-----------+
 *
 * Response error frame format. Second byte is Command Bitwise OR 0x80. Total 5 bytes
 * +---------------+----------------------+------------+-----------+-----------+
 * |  RTU address  |  RTU Command OR 0x80 | Error code | CRC16 MSB | CRC16 LSB |
 * +---------------+----------------------+------------+-----------+-----------+
 *
*/
void PZEM004Tv30::_parseRxData()
{
    if(!_modbusCRC16( _rx.receiveBuffer, _rx.Bytes))
    {
        _setError();
        _err.rtuError = PZEM_SLAVE_CRC_ERR;
        _resetRx();
        
        #if _PZEM004T_DIAG_STATS_
          _rtuDiag.RTUCRCFail++;
        #endif
        
        return;
    }

    if(_rx.receiveBuffer[1] & 0b10000000)
    {
        _setError();
        _err.rtuError = _rx.receiveBuffer[PZEM_ERROR_POS_IN_FRAME];
        
        #if _PZEM004T_DIAG_STATS_
          _rtuDiag.RTUErrorFrame++;
  
          switch (_err.rtuError)
          {
              case PZEM_ILLEGAL_FUNC_ERR:
                  _rtuDiag.RTUIllegalFunction++;
                  break;
              case PZEM_ILLEGAL_ADDR_ERR:
                  _rtuDiag.RTUIllegalAddress++;
                  break;
              case PZEM_ILLEGAL_DATA_ERR:
                  _rtuDiag.RTUIllegalData++;
                  break;
              case PZEM_SLAVE_ERR:
                  _rtuDiag.RTUSlaveError++;
                  break;
              default:
                  break;
          }
        #endif
        
        _resetRx();
        return;
    }

    _err.rtuError = PZEM_NO_ERROR;      //Speculative execution...
    _resetRx();

    if( (_rx.receiveBuffer[1] == PZEM_CMD_READ_INPUT_REG) || (_rx.receiveBuffer[1] == PZEM_CMD_READ_HOLD_REG) )
    {
        for(uint8_t bufIndex=0; bufIndex<(_rx.receiveBuffer[PZEM_BYTES_IN_FRAME_POS]); bufIndex += 2)
        {
            if(_rx.receiveBuffer[1] == PZEM_CMD_READ_INPUT_REG) 
            {
                _rtuReg.InputReg[(_err.lastRegAddr+(bufIndex/2))] = ((uint16_t)_rx.receiveBuffer[bufIndex+3]<<8 | (uint16_t)_rx.receiveBuffer[bufIndex+4]);  
            }
            else
            {
                _rtuReg.HoldingReg[(_err.lastRegAddr+(bufIndex/2))] = ((uint16_t)_rx.receiveBuffer[bufIndex+3]<<8 | (uint16_t)_rx.receiveBuffer[bufIndex+4]);
            }
        }
        
        if(_rx.receiveBuffer[1] == PZEM_CMD_READ_INPUT_REG)
        {
            _err.inputError = PZEM_NO_ERROR;
            if( (_alarmCallback) && (_rtuReg.InputReg[PZEM_ALARM_REG_ADDR] != 0x0000) ) _alarmCallback();
        }
        else
        {
            _err.holdError = PZEM_NO_ERROR;  
        }
        return;
    }
    
    if( (_rx.receiveBuffer[1] == PZEM_CMD_WRITE_HOLD_REG) || (_rx.receiveBuffer[1] == PZEM_CMD_RESET_ENERGY) )
    {
        _err.inputError = PZEM_NO_ERROR;
    }
}

/*!
 * PZEM004Tv30::_setError
 *
 * Set error flags
*/
void PZEM004Tv30::_setError()
{
    if(_err.lastCMD == PZEM_CMD_READ_INPUT_REG )
    {
        _err.inputError = PZEM_DATA_NOT_UPDATED;
    }

    if(_err.lastCMD == PZEM_CMD_READ_HOLD_REG)
    {
        _err.holdError = PZEM_PARAMS_NOT_UPDATED;
    }

    _err.lastCMD = PZEM_CMD_NOP_COMMAND;
    return;
}

/*!
 * PZEM004Tv30::_setHoldRegister
 *
 * Set single holding register value
 *
 * @param regAddress Register address where to write
 * @param regValue Value to write
 * @return success
*/
bool PZEM004Tv30::_setHoldRegister(uint16_t regAddress, uint16_t regValue)
{
    _sendCommand(PZEM_CMD_WRITE_HOLD_REG, regAddress, regValue);
    return (_err.inputError == PZEM_NO_ERROR);    
}

/*!
 * PZEM004Tv30::_modbusCRC16
 *
 * Calculate CRC16_MODBUS with lookup table
 * Polynomial: 0x8005, Initial value: 0xFFFF, Input reflected, Result reflected
 *
 * Lookup table generator http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
 *
 * @param rtuFrame Pointer to buffer
 * @param regValue Number of bytes in buffer
 * @param set Set or check CRC value
 * @return success
*/
bool PZEM004Tv30::_modbusCRC16(uint8_t *rtuFrame, uint8_t bytes, bool set)
{
    static const uint16_t modbusCRCTable[] PROGMEM = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };

    //uint8_t tblIndex;
    uint16_t modbusCRC = 0xFFFF;
    bytes -= ( set ? 2 : 0 );
    while(bytes--)
    {
        modbusCRC = ( modbusCRC >> 8 ) ^ (uint16_t)pgm_read_word(&modbusCRCTable[( (modbusCRC ^ *rtuFrame++) & 0xFF ) ]);
    }

    if(set)
    {
        *rtuFrame++ = (modbusCRC & 0xFF);       //Reflected CRC Value
        *rtuFrame = ((modbusCRC >> 8) & 0xFF);
        return true;
    }
    return (modbusCRC == 0x0000);
}

//end pzem-004T-v3.0.cpp
