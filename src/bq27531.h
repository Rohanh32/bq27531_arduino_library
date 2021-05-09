/******************************************************************************
bq27531.h
BQ27441 Arduino Library Main Header File
Rohan Hajare
May 9, 2021

Implementation of all features of the BQ27531 Fuel Gauge.

Hardware Resources:
- Arduino Development Board
- BQ27531 Development Board

Development environment specifics:
Arduino 1.8.9
******************************************************************************/

#ifndef _BQ27531_H_
#define _BQ27531_H_

#include "Arduino.h"

#define BQ27531_ADDRES 				(0x55) 		/* BQ27531 slave address ADDR[6-0] */
#define BQ27531_UNSEAL_KEY0			(0x0414)	/* Secret code to unseal the BQ27531-G1 */
#define BQ27531_UNSEAL_KEY1			(0x3672)	/* Secret code to unseal the BQ27531-G1 */
#define BQ27531_DEVICE_ID			(0x0531)	/* Default device ID */

/**
 * Control Status Word - Bit Definitions
 * Bit positions for the 16-bit data of CONTROL_STATUS.
 * CONTROL_STATUS instructs the fuel gauge to return status information to 
 * Control() addresses 0x00 and 0x01. The read-only status word contains status
 * bits that are set or cleared either automatically as conditions warrant or
 * through using specified subcommands.
 */
#define BQ27531_STATUS_SHUTDOWNEN	(1<<15)
#define BQ27531_STATUS_WDRESET		(1<<14)
#define BQ27531_STATUS_SS			(1<<13)
#define BQ27531_STATUS_CALMODE		(1<<12)
#define BQ27531_STATUS_CCA			(1<<11)
#define BQ27531_STATUS_BCA			(1<<10)
#define BQ27531_STATUS_QMAX_UP		(1<<9)
#define BQ27531_STATUS_RES_UP		(1<<8)
#define BQ27531_STATUS_INITCOMP		(1<<7)
#define BQ27531_STATUS_HIBERNATE	(1<<6)
#define BQ27531_STATUS_SLEEP		(1<<4)
#define BQ27531_STATUS_LDMD			(1<<3)
#define BQ27531_STATUS_RUP_DIS		(1<<2)
#define BQ27531_STATUS_VOK			(1<<1)

/**
 * Flag Command - Bit Definitions
 * Bit positions for the 16-bit data of Flags()
 * This read-word function returns the contents of the fuel gauging status
 * register, depicting the current operating status.
 */
#define BQ27531_FLAG_OT				(1<<15)
#define BQ27531_FLAG_UT				(1<<14)
#define BQ27531_FLAG_CALMODE		(1<<12)
#define BQ27531_FLAG_DIV_CUR		(1<<11)
#define BQ27531_FLAG_GG_CHGRCTL_EN	(1<<10)
#define BQ27531_FLAG_FC				(1<<9)
#define BQ27531_FLAG_CHG			(1<<8)
#define BQ27531_FLAG_OCV_GD			(1<<5)
#define BQ27531_FLAG_WAIT_ID		(1<<4)
#define BQ27531_FLAG_BAT_DET		(1<<3)
#define BQ27531_FLAG_SOC1			(1<<2)
#define BQ27531_FLAG_SYSDOWN		(1<<1)
#define BQ27531_FLAG_DSG			(1<<0)

/**
 * Extended Data Commands
 * Extended data commands offer additional functionality beyond the standard
 * set of commands. They are used in the same manner; however, unlike standard
 * commands, extended commands are not limited to 2-byte words.
 */
#define BQ27531_EXTENDED_CAPACITY	(0x3C)	// DesignCapacity()
#define BQ27531_EXTENDED_DATACLASS	(0x3E)	// DataClass()
#define BQ27531_EXTENDED_DATABLOCK	(0x3F)	// DataBlock()
#define BQ27531_EXTENDED_BLOCKDATA	(0x40)	// BlockData()
#define BQ27531_EXTENDED_CHECKSUM	(0x60)	// BlockDataCheckSum()
#define BQ27531_EXTENDED_CONTROL	(0x61)	// BlockDataControl()
#define BQ27531_EXTENDED_STATUS		(0x6A)	// ApplicationStatus()()

/**
 * Configuration Class, Subclass ID's
 * To access a subclass of the extended data, set the DataClass() function
 * with one of these values.
 */
/* Configuration Classes */
#define BQ27531_ID_SAFETY			(2)		// Safety
#define BQ27531_ID_CHG_TERMINATION	(36)	// Charge Termination
#define BQ27531_ID_CONFIG_DATA		(48)	// Data
#define BQ27531_ID_DISCHARGE		(49)	// Discharge
#define BQ27531_ID_REGISTERS		(64)	// Registers
#define BQ27531_ID_POWER			(68)	// Power
/* Gas Gauging Classes */
#define BQ27531_ID_IT_CFG			(80)	// IT Cfg
#define BQ27531_ID_CURRENT_THRESH	(81)	// Current Thresholds
#define BQ27531_ID_STATE			(82)	// State
#define BQ27531_ID_LAST_RUN			(95)	// Last Run
/* OCV Tables Classes */
#define BQ27531_ID_OCV_A0			(83)	// OCVa0 Table
#define BQ27531_ID_OCV_A1			(84)	// OCVa1 Table
/* Ra Tables Classes */
#define BQ27531_ID_DEF0_RA			(87)	// Def0 Ra
#define BQ27531_ID_DEF1_RA			(88)	// Def1 Ra
#define BQ27531_ID_PACK0_RA			(91)	// Pack0 Ra
#define BQ27531_ID_PACK1_RA			(92)	// Pack1 Ra
#define BQ27531_ID_RAM_RA			(110)	// RAM Ra Table
/* Calibration Classes */
#define BQ27531_ID_CALIB_DATA		(104)	// Data
#define BQ27531_ID_TEMP_MODEL		(106)	// Temp Model
#define BQ27531_ID_CURRENT			(107)	// Current
/* Security Classes */
#define BQ27531_ID_CODES			(112)	// Codes
/* Charger Classes */
#define BQ27531_ID_TEMPERATURE		(74)	// Temperature Table
#define BQ27531_ID_CHARGER_INFO		(76)	// Charger info
#define BQ27531_ID_CHARGER_CONTROL	(78)	// Charger Control Configuration

/* BQ27531 fual guage commands
 * The fuel gauge uses a series of 2-byte standard commands to enable system 
 * reading and writing of battery information. Each command has an associated
 * sequential command-code pair.
 */
#define BQ27531_CMD_CNTL							(0x00)
#define BQ27531_CMD_AT_RATE							(0x02)
#define BQ27531_CMD_AT_RATE_TIME_TO_EMPTY			(0x04)
#define BQ27531_CMD_TEMP							(0x06)
#define BQ27531_CMD_VOLTAGE							(0x08)
#define BQ27531_CMD_FLAGS							(0x0A)
#define BQ27531_CMD_NOMINAL_CAPACITY				(0x0C)
#define BQ27531_CMD_FULL_CAPACITY					(0x0E)
#define BQ27531_CMD_REMAINING_CAPACITY				(0x10)
#define BQ27531_CMD_FULL_CHARGE_CAPACITY			(0x12)
#define BQ27531_CMD_AVERAGE_CURRENT					(0x14)
#define BQ27531_CMD_TIME_TO_EMPTY					(0x16)
#define BQ27531_CMD_REMAINING_CAPACITY_UNFIL		(0x18)
#define BQ27531_CMD_STANDBY_CURRENT					(0x1A)
#define BQ27531_CMD_REMAINING_CAPACITY_FILTERED		(0x1C)	
#define BQ27531_CMD_PROG_CHARGING_CURRENT			(0x1E)	
#define BQ27531_CMD_PROG_CHARGING_VOLTAGE			(0x20)
#define BQ27531_CMD_FULL_CHARGE_CAPACITY_UNFIL		(0x22)	
#define BQ27531_CMD_AVERAGE_POWER					(0x24)
#define BQ27531_CMD_FULL_CHARGE_CAPACITY_FIL		(0x26)
#define BQ27531_CMD_STATE_OF_HEALTH					(0x28)
#define BQ27531_CMD_CYCLE_COUNT						(0x2A)
#define BQ27531_CMD_STATE_OF_CHARGE					(0x2C)
#define BQ27531_CMD_TRUE_SOC						(0x2E)
#define BQ27531_CMD_INSTANTANEOUS_CURRENT_READING	(0x30)
#define BQ27531_CMD_INTERNAL_TEMPERATURE			(0x32)
#define BQ27531_CMD_CHARGING_LEVEL					(0x34)
#define BQ27531_CMD_LEVEL_TAPER_CURRENT				(0x6E)
#define BQ27531_CMD_CALC_CHARGING_CURRENT			(0x70)
#define BQ27531_CMD_CALC_CHARGING_VOLTAGE			(0x72)

/* BQ27531 fual guage control subcommands
 * Issuing a Control() command requires a subsequent 2-byte subcommand. These
 * additional bytes specify the particular control function desired. The 
 * Control() command allows the system to control specific features of the fuel
 * gauge during normal operation and additional features when the device is in 
 * different access modes.
 */
#define BQ27531_CNTL_CONTROL_STATUS		(0x0000)
#define BQ27531_CNTL_DEVICE_INFO		(0x0001)
#define BQ27531_CNTL_FW_VERSION			(0x0002)
#define BQ27531_CNTL_HW_VERSION			(0x0003)
#define BQ27531_CNTL_PREV_MACWRITE		(0x0007)
#define BQ27531_CNTL_CHEM_ID			(0x0008)
#define BQ27531_CNTL_BOARD_OFFSET		(0x0009)
#define BQ27531_CNTL_CC_OFFSET			(0x000A)
#define BQ27531_CNTL_CC_OFFSET_SAVE		(0x000B)
#define BQ27531_CNTL_OCV_CMD			(0x000C)
#define BQ27531_CNTL_BAT_INSERT			(0x000D)
#define BQ27531_CNTL_BAT_REMOVE			(0x000E)
#define BQ27531_CNTL_SET_HIBERNATE		(0x0011)
#define BQ27531_CNTL_CLEAR_HIBERNATE	(0x0012)
#define BQ27531_CNTL_SET_SLEEP_PLUS		(0x0013)
#define BQ27531_CNTL_CLEAR_SLEEP_PLUS	(0x0014)
#define BQ27531_CNTL_OTG_ENABLE			(0x0015)
#define BQ27531_CNTL_OTG_DISABLE		(0x0016)
#define BQ27531_CNTL_DIV_CUR_ENABLE		(0x0017)
#define BQ27531_CNTL_CHG_ENABLE			(0x001A)
#define BQ27531_CNTL_CHG_DISABLE		(0x001B)
#define BQ27531_CNTL_GG_CHGRCTL_ENABLE	(0x001C)
#define BQ27531_CNTL_GG_CHGRCTL_DISABLE	(0x001D)
#define BQ27531_CNTL_DIV_CUR_DISABLE	(0x001E)
#define BQ27531_CNTL_DF_VERSION			(0x001F)
#define BQ27531_CNTL_SEALED				(0x0020)
#define BQ27531_CNTL_IT_ENABLE			(0x0021)
#define BQ27531_CNTL_RESET				(0x0041)
#define BQ27531_CNTL_SHIPMODE_ENABLE	(0x0050)
#define BQ27531_CNTL_SHIPMODE_DISABLE	(0x0051)

/**
 * Parameters for the readCurrent() function, to specify which current to read
 */
typedef enum {
	AVG,	// Average Current (DEFAULT)
	STBY,	// Standby Current
	PROGI,	// Programed Charging Current
	INST,	// Instantaneous Current
	TAPER,	// Target Taper Current
	CALCI	// Calc/Recommended Charging Current
} current_measure;

/**
 * Parameters for the readVoltage() function, to specify which current to read
 */
typedef enum {
	MEAS,	// Average Current (DEFAULT)
	PROGV,	// Programed Charging Current
	CALCV	// CalcCharging/Recommended Charging Current
} voltage_measure;

/**
 * Parameters for the temperature() function
 */
typedef enum {
	BATTERY,		// Battery Temperature (DEFAULT)
	INTERNAL_TEMP	// Internal IC Temperature
} temp_measure;

/**
 * Parameters for the capacity() function, to specify which capacity to read
 */
typedef enum {
	REMAIN,		// Remaining Capacity (DEFAULT)
	FULL,		// Full Capacity
	AVAIL,		// Nominal Available Capacity
	FULL_CHAR,	// Full Available Capacity
	REMAIN_UF,	// Remaining Capacity Unfiltered
	REMAIN_F,	// Remaining Capacity Filtered
	FULL_UF,	// Full Capacity Unfiltered
	FULL_F,		// Full Capacity Filtered
} capacity_measure;

/**
 * Parameters for the soc() function
 */
typedef enum {
	FILTERED,	// State of Charge Filtered (DEFAULT)
	UNFILTERED	// State of Charge Unfiltered
} soc_measure;

class BQ27531 {
    public:
        BQ27531();
		BQ27531(uint8_t address);
		void begin();
		void initialize();
		bool testConnection();

		/********** Fual Gauge Commands ************/
		bool readTemperature(temp_measure type, uint16_t *temperature);
		bool readVoltage(voltage_measure type, uint16_t *voltage); 
		bool readCurrent(current_measure type, int16_t *current); 
		bool readCapacity(capacity_measure type, uint16_t *capacity);
		bool readPower(uint16_t *power);
		bool readStateOfCharge(soc_measure type, uint16_t *soc);
		bool readStateOfHealth(uint16_t *soh);
		
		/********** Control Sub-Commands ***********/
		bool readControlDeviceType(uint16_t *data);
		bool readControlFirmwareVersion(uint16_t *data);
		bool readControlHardwareVersion(uint16_t *data);
		uint16_t flags(void);
		uint16_t status(void);

		/********** Flag register Functions ********/
		bool getFlagDsg(void);
		bool getFlagSysdown(void);
		bool getFlagSoc1(void);
		bool getFlagBatDet(void);
		bool getFlagWaitId(void);
		bool getFlagOcvGd(void);
		bool getFlagChg(void);
		bool getFlagFc(void);
		bool getFlagGgChgrctlEn(void);
		bool getFlagDivCur(void);
		bool getFlagCalmode(void);
		bool getFlagUt(void);
		bool getFlagOt(void);

    private:
        uint8_t devAddr;		// Stores the BQ27531-G1's I2C address
        uint16_t buffer;
		bool sealFlag; 			// Global to identify that IC was previously sealed

		bool sealed(void);
		bool seal(void);
		bool unseal(void);
		bool readControlCommandRegister(uint8_t cmd, uint16_t *data);
		bool readCommandRegister(uint8_t cmd, uint16_t *data);
		bool executeControlWord(uint16_t function);
		bool enterConfig();
		bool exitConfig();

		/********** Extended Data Commands ********/
		bool blockDataControl(void);
		bool blockDataClass(uint8_t id);
		bool blockDataOffset(uint8_t offset);
		uint8_t blockDataChecksum(void);
		uint8_t readBlockData(uint8_t offset);
		bool writeBlockData(uint8_t offset, uint8_t data);
		uint8_t computeBlockChecksum(void);
		bool writeBlockChecksum(uint8_t csum);
		uint8_t readExtendedData(uint8_t classID, uint8_t offset);
		bool writeExtendedData(uint8_t classID, uint8_t offset, uint8_t * data, uint8_t len);

		/********** I2C Read and Write Routines ***/
		bool i2cReadBytes(uint8_t regAddr, uint8_t * dest, uint8_t length);
		bool i2cWriteBytes(uint8_t regAddr, uint8_t * src, uint8_t length);

};

#endif // End of _BQ27531_H_

