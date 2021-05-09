/******************************************************************************
bq27531.cpp
BQ27441 Arduino Library Main Source File
Rohan Hajare
May 9, 2021

Implementation of all features of the BQ27531 Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- BQ27531 Development Board

Development environment specifics:
Arduino 1.8.9
******************************************************************************/

#include "Arduino.h"
#include <Wire.h>
#include "bq27531.h"

/** Default constructor, uses default I2C address. Assumes the device is 
 *  BQ27531
 */
BQ27531::BQ27531()
{
	// set Default BQ27531 I2C slave device address
    devAddr = BQ27531_ADDRES;
	sealFlag = true;
}

/** Specific address constructor.
 * @param address 7 bits of the slave address should be provided here
 */
BQ27531::BQ27531(uint8_t address)
{
	// set user given BQ27531 I2C slave device address
    devAddr = address;
	sealFlag = true;
}

/** Prepare I2C communication interface.
 */
void BQ27531::begin()
{
	Wire.begin();
}

/** Power on and prepare for general usage.
 */
void BQ27531::initialize()
{
  // Nothing to do here
  // TBD
}

/** Verify the I2C connection. read the temperature value from fual gauge
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::testConnection()
{
    if (i2cReadBytes(BQ27531_CMD_TEMP, (uint8_t*)&buffer, 2) == 1) {
        return true;
    }
    return false;
}


/*************************** Control Sub-Commands ****************************/
/** read BQ27531 Device Type.
 * @param data Buffer to store read data in
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readControlDeviceType(uint16_t *data)
{
  
    if (readControlCommandRegister(BQ27531_CNTL_DEVICE_INFO, data) == 0)
	{
        return false;
    }
    return true;
}

/** read BQ27531 Firmware Version.
 * @param data Buffer to store read data in
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readControlFirmwareVersion(uint16_t *data)
{
  
    if (readControlCommandRegister(BQ27531_CNTL_FW_VERSION, data) == 0)
	{
        return false;
    }
    return true;
}

/** read BQ27531 Hardware Version.
 * @param data Buffer to store read data in
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readControlHardwareVersion(uint16_t *data)
{
  
    if (readControlCommandRegister(BQ27531_CNTL_HW_VERSION, data) == 0) {
        return false;
    }
    return true;
}

/** read BQ27531 flags.
 * @return data 16bit flag register value
 */
uint16_t BQ27531::flags(void)
{
	uint16_t data = 0;
	readCommandRegister(BQ27531_CMD_FLAGS, (uint16_t *)&data);
	return data;
}

/** read BQ27531 CONTROL_STATUS subcommand of control.
 * @return data 16bit status register value
 */
uint16_t BQ27531::status(void)
{
	uint16_t data = 0;
	readControlCommandRegister(BQ27531_CNTL_CONTROL_STATUS, (uint16_t *)&data);
	return data;
}


/**************************** Fual Gauge Commands ****************************/
/** read BQ27531 Temperature.
 * @param data Buffer to store read data in
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readTemperature(temp_measure type, uint16_t *temperature)
{
	bool status = false;
	uint8_t cmd;
	switch (type)
	{
	case BATTERY:
		cmd = BQ27531_CMD_VOLTAGE;
		break;
	case INTERNAL_TEMP:
		cmd = BQ27531_CMD_PROG_CHARGING_VOLTAGE;
		break;
	}
	status = readCommandRegister(cmd, temperature);
	return status;
}

/** read BQ27531 Voltage.
 * @param type type of voltage to read from BQ27531
 * @param data Buffer to store read data in
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readVoltage(voltage_measure type, uint16_t *voltage) 
{
	bool status = false;
	uint8_t cmd;
	switch (type)
	{
	case MEAS:
		cmd = BQ27531_CMD_VOLTAGE;
		break;
	case PROGV:
		cmd = BQ27531_CMD_PROG_CHARGING_VOLTAGE;
		break;
	case CALCV:
		cmd = BQ27531_CMD_CALC_CHARGING_VOLTAGE;
		break;
	}
	status = readCommandRegister(cmd, voltage);
	return status;
}

/** read BQ27531 current.
 * @param type type of current to read from BQ27531
 * @param current signed curret value
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readCurrent(current_measure type, int16_t *current) 
{
	bool status = false;
	uint8_t cmd;
	switch (type)
	{
	case AVG:
		cmd = BQ27531_CMD_AVERAGE_CURRENT;
		break;
	case STBY:
		cmd = BQ27531_CMD_STANDBY_CURRENT;
		break;
	case PROGI:
		cmd = BQ27531_CMD_PROG_CHARGING_CURRENT;
		break;
	case INST:
		cmd = BQ27531_CMD_INSTANTANEOUS_CURRENT_READING;
		break;
	case TAPER:
		cmd = BQ27531_CMD_LEVEL_TAPER_CURRENT;
		break;
	case CALCI:
		cmd = BQ27531_CMD_CALC_CHARGING_CURRENT;
		break;
	}
	status = readCommandRegister(cmd, (uint16_t *)current);
	return status;
}

/** read BQ27531 charge capacity.
 * @param type type of capacity to read from BQ27531
 * @param capacity capacity value
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readCapacity(capacity_measure type, uint16_t *capacity)
{
	bool status = false;
	uint8_t cmd;
	switch (type)
	{
	case REMAIN:
		cmd = BQ27531_CMD_REMAINING_CAPACITY;
		break;
	case FULL:
		cmd = BQ27531_CMD_FULL_CAPACITY;
		break;
	case AVAIL:
		cmd = BQ27531_CMD_NOMINAL_CAPACITY;
		break;
	case FULL_CHAR:
		cmd = BQ27531_CMD_FULL_CHARGE_CAPACITY;
		break;
	case REMAIN_UF:
		cmd = BQ27531_CMD_REMAINING_CAPACITY_UNFIL;
		break;
	case REMAIN_F:
		cmd = BQ27531_CMD_REMAINING_CAPACITY_FILTERED;
		break;
	case FULL_UF:
		cmd = BQ27531_CMD_FULL_CHARGE_CAPACITY_UNFIL;
		break;
	case FULL_F:
		cmd = BQ27531_CMD_FULL_CHARGE_CAPACITY_FIL;
		break;
	}
	status = readCommandRegister(cmd, capacity);
	return status;
}

/** read BQ27531 Average Power.
 * @param power Average power value pointer
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readPower(uint16_t *power)
{
	bool status;
	status = readCommandRegister(BQ27531_CMD_AVERAGE_POWER, power);
	return status;
}

/** read BQ27531 state of charge measurement.
 * @param type type of state of charge to read from BQ27531
 * @param soc soc value pointer
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readStateOfCharge(soc_measure type, uint16_t *soc)
{
	bool status = false;
	uint8_t cmd;
	switch (type)
	{
	case FILTERED:
		cmd = BQ27531_CMD_STATE_OF_CHARGE;
		break;
	case UNFILTERED:
		cmd = BQ27531_CMD_TRUE_SOC;
		break;
	}
	status = readCommandRegister(cmd, soc);
	return status;
}

/** read BQ27531 state of health measurement.
 * @param soh state of health value pointer
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readStateOfHealth(uint16_t *soh)
{
	bool status;
	status = readCommandRegister(BQ27531_CMD_STATE_OF_HEALTH, soh);
	return status;
}


/*************************** Flag register Functions ************************/
/** get DSG bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagDsg(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_DSG);
}

/** get SYSDOWN bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagSysdown(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_SYSDOWN);
}

/** get SOC1 bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagSoc1(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_SOC1);
}

/** get BAT_DET bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagBatDet(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_BAT_DET);
}

/** get WAIT_ID bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagWaitId(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_WAIT_ID);
}

/** get OCV_GD bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagOcvGd(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_OCV_GD);
}

/** get CHG bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagChg(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_CHG);
}

/** get FC bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagFc(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_FC);
}

/** get GG_CHGRCTL_EN bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagGgChgrctlEn(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_GG_CHGRCTL_EN);
}

/** get DIV_CUR bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagDivCur(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_DIV_CUR);
}

/** get CALMODE bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagCalmode(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_CALMODE);
}

/** get UT bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagUt(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_UT);
}

/** get OT bit status in flag.
 * @return bit status
 */
bool BQ27531::getFlagOt(void)
{
	uint16_t flagState = flags();
	return (flagState & BQ27531_FLAG_OT);
}


/****************************** Private Functions ***************************/
/** Enter configuration mode.
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::enterConfig()
{	
	if (sealed())
	{
		sealFlag = true;
		unseal(); // Must be unsealed before making changes
	}
	
	return true;
}

/** Exit configuration mode.
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::exitConfig()
{
	if (!sealed())
	{
		sealFlag = false;
		seal();
	}
	return true;
}

/** read BQ27531 seal status.
 * @return status return seal bit value from control status register
 */
bool BQ27531::sealed(void)
{
	uint16_t stat = status();
	return (stat & BQ27531_STATUS_SS);
}

/** Seal the BQ27531-G1.
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::seal(void)
{
	return executeControlWord(BQ27531_CNTL_SEALED);
}

/** Unseal the BQ27531-G1. write the key to the control command.
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::unseal(void)
{
	if (!executeControlWord(BQ27531_UNSEAL_KEY0))
		return false;
	if (!executeControlWord(BQ27531_UNSEAL_KEY1))
		return false;
	return true;
}

/** read BQ27531 Control Command registers.
 * @param cmd BQ27531 Control subcommand code
 * @param data Buffer to store read data in
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readControlCommandRegister(uint8_t cmd, uint16_t *data) {
	uint16_t subcommand = (uint16_t)cmd;
    if (i2cWriteBytes(BQ27531_CMD_CNTL, (uint8_t*)&subcommand, 2) == 0) {
        return false;
    } 
    if (i2cReadBytes(BQ27531_CMD_CNTL, (uint8_t*)data, 2) == 0) {
        return false;
    }
    return true;
}

/** read BQ27531 Command registers.
 * @param cmd BQ27531 command code
 * @param data Buffer to store read data in
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::readCommandRegister(uint8_t cmd, uint16_t *data) {
  
    if (i2cReadBytes((uint8_t)cmd, (uint8_t*)data, 2) == 0) {
        return false;
    }
    return true;
}

/** Execute a subcommand from the BQ27531-G1's control.
 * @param cmd BQ27531 command code
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::executeControlWord(uint16_t function)
{
	uint16_t subcommand = (uint16_t)function;
    if (i2cWriteBytes(BQ27531_CMD_CNTL, (uint8_t*)&subcommand, 2) == 0) {
        return false;
    }
	return true;
}


/************************** Extended Data Commands ***************************/
/** Enable BlockData access.
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::blockDataControl(void)
{
	uint8_t enableByte = 0x00;
	return i2cWriteBytes(BQ27531_EXTENDED_CONTROL, &enableByte, 1);
}

/** Set the block data class to be accessed.
 * @param id BQ27531 Extended data class ID
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::blockDataClass(uint8_t id)
{
	return i2cWriteBytes(BQ27531_EXTENDED_DATACLASS, &id, 1);
}

/** Set the block data offset to be accessed.
 * @param id BQ27531 Extended data class offset
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::blockDataOffset(uint8_t offset)
{
	/* To access data located at offset 0 to 31 use offset = 0x00. 
	 * To access data located at offset 32 to 41 use offset = 0x01
	 */
	offset = (offset >> 5);
	return i2cWriteBytes(BQ27531_EXTENDED_DATABLOCK, &offset, 1);
}

/** Read the current checksum.
 * @return csum old checksum value
 */
uint8_t BQ27531::blockDataChecksum(void)
{
	uint8_t csum;
	i2cReadBytes(BQ27531_EXTENDED_CHECKSUM, &csum, 1);
	return csum;
}

/** Read a byte from the loaded extended data flash.
 * @param offset BQ27531 Extended data offset
 * @return ret byte value read from externed block data offset
 */
uint8_t BQ27531::readBlockData(uint8_t offset)
{
	uint8_t ret;
	uint8_t address;
	offset = (offset & 0x1F);
	address = offset + BQ27531_EXTENDED_BLOCKDATA;
	i2cReadBytes(address, &ret, 1);
	return ret;
}

/** Wrie a byte to the loaded extended data flash.
 * @param offset BQ27531 Extended data offset
 * @param data byte to write in Extended data offset location
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::writeBlockData(uint8_t offset, uint8_t data)
{
	uint8_t address;
	offset = (offset & 0x1F);
	address = offset + BQ27531_EXTENDED_BLOCKDATA;
	return i2cWriteBytes(address, &data, 1);
}

/** Compute checksum of 32 bytes of the loaded extended data.
 * @return csum return the computed checksum value
 */
uint8_t BQ27531::computeBlockChecksum(void)
{
	uint8_t data[32];
	i2cReadBytes(BQ27531_EXTENDED_BLOCKDATA, data, 32);

	uint8_t csum = 0;
	for (int i=0; i<32; i++)
	{
		csum += data[i];
	}
	csum = 255 - csum;
	
	return csum;
}

/** Write computed checksum of the loaded extended data.
 * @param csum new checksum value for loaded extended data
 * @return True if connection is valid, false otherwise
 */
bool BQ27531::writeBlockChecksum(uint8_t csum)
{
	return i2cWriteBytes(BQ27531_EXTENDED_CHECKSUM, &csum, 1);	
}

/** Read a byte from extended data specifying a class ID and position offset.
 * @param classID extended data class ID
 * @param offset subclass offset in extended data class ID
 * @return retData byte read from extended class ID and Offset location
 */
uint8_t BQ27531::readExtendedData(uint8_t classID, uint8_t offset)
{
	uint8_t retData = 0;
	enterConfig();
		
	if (!blockDataControl()) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
		return false;
	
	blockDataOffset(offset); // block offset
	retData = readBlockData(offset); // Read from offset
	
	exitConfig();
	
	return retData;
}

/** Write a specified number of bytes to extended data specifying a class ID, position offset.
 * @param classID extended data class ID
 * @param offset subclass offset in extended data class ID
 * @param data Buffer to copy new data to extended data class memory
 * @param len write number of data byte to extended data class memory
 * @return true if requested number of bytes received, false otherwise
 */
bool BQ27531::writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len)
{
	if (len > 32)
		return false;
	
	enterConfig();
	
	if (!blockDataControl()) // // enable block data memory control
		return false; // Return false if enable fails
	if (!blockDataClass(classID)) // Write class ID using DataBlockClass()
		return false;
	
	blockDataOffset(offset); // Write 32-bit block offset (usually 0)

	// Write data bytes to extended bockdata memory
	for (int i = 0; i < len; i++)
	{
		writeBlockData((offset + i), data[i]);
	}

	uint8_t newCsum = computeBlockChecksum(); // Compute the new checksum
	writeBlockChecksum(newCsum);

	exitConfig();
	
	return true;
}


/*********************** I2C Read and Write Routines *************************/

/** Read multiple bytes from an 8-bit device register.
 * @param regAddr First register regAddr to read from
 * @param dest Buffer to store read data in
 * @param length Number of bytes to read
 * @return true if requested number of bytes received, false otherwise
 */
bool BQ27531::i2cReadBytes(uint8_t regAddr, uint8_t * dest, uint8_t length)
{
	uint8_t count = 0;	
	Wire.beginTransmission(devAddr);
	Wire.write(regAddr);
	Wire.endTransmission(false);
	Wire.requestFrom(devAddr, length);
	
	for (; Wire.available(); count++)
	{
		dest[count] = Wire.read();
	}
	return (count == length);
}

/** Write multiple bytes to an 8-bit device register.
 * @param regAddr First register address to write to
 * @param src Buffer to copy new data from
 * @param length Number of bytes to write
 * @return Status of operation (true = success)
 */
bool BQ27531::i2cWriteBytes(uint8_t regAddr, uint8_t * src, uint8_t length)
{
	uint8_t status = 0;
	Wire.beginTransmission(devAddr);
	Wire.write(regAddr);
	for (int i=0; i<length; i++)
	{
		Wire.write(src[i]);
	}	
	status = Wire.endTransmission(true);
	
	return (status == 0);	
}

