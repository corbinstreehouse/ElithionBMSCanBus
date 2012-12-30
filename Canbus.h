// Can bus reading for the elithion BMS - by corbin dunn, modified from other examples.
// Provided as-is. Use at your own risk.
// www.corbinstreehouse.com
// Nov 16, 2012

#ifndef CANBUS_H
#define CANBUS_H


#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps

#include <stdint.h>

typedef enum {
    CanSpeed500 = 1,
    CanSpeed250 = 3,
    CanSpeed125 = 7,
} CanSpeed;


// not yet sure if this is a bit being set (ie: multiple faults can happen at the same time), or if it is a single hex value
enum _FaultKindOptions {
    FaultKindDrivingOffWhilePluggedIn = 1 << 0, //	01h	Driving off while plugged in
    FaultKindInterlockTripped = 1 << 1, //	02h	Interlock is tripped
    FaultKindCommFault = 1 << 2, //	04h	Communication fault with a bank or cell
    FaultKindChargeOverCurrent = 1 << 3, //	08h	Charge overcurrent
    FaultKindDischargeOverCurrent = 1 << 4, //	10h	Discharge overcurrent
    FaultKindOverTemperature = 1 << 5, //	20h	Over-temperature fault
    FaultKindUnderVoltage = 1 << 6, //	40h	Under voltage
    FaultKindOverVoltage = 1 << 7, //	80h	Over voltage
    
    // Custom messages
    FaultKindCantFindBMSOnCanBus = 1 << 8,
//    FaultKindCanBusNotInitialized = 1 << 9, // would be nice
    FaultKindCanBusFailedInitialization = 1 << 9,

};

typedef uint16_t FaultKindOptions; // bitset of the above fault kinds

enum _StoredFaultKind {
    StoredFaultKindNone = 0, // 00h	No fault
    StoredFaultKindDrivingOffWhilePluggedIn = 1, //1	01h	Driving off while plugged in
    StoredFaultKindInterlockTripped = 2, //2	02h	Interlock is tripped
    StoredFaultKindCommFault = 3, //3	03h	Communication fault with a bank or cell
    StoredFaultKindChargeOverCurrent = 4, //4	04h	Charge overcurrent
    StoredFaultKindDischargeOverCurrent = 5, //5	05h	Discharge overcurrent
    StoredFaultKindOverTemperature = 6, //6	06h	Over-temperature fault
    StoredFaultKindUnderVoltage = 7, //7	07h	Under voltage
    StoredFaultKindOverVoltage = 8, //8	08h	Over voltage

    /////////////// NOTE: I only print the faults above in descriptions to save space...
    
    StoredFaultKindNoBatteryVoltage = 9, //9	09h	No battery voltage
    StoredFaultKindHighVoltageBMinusLeak = 10, //10	0Ah	High voltage B- leak to chassis
    StoredFaultKindHighVoltageBPlusLeak = 11, //11	0Bh	High voltage B+ leak to chassis
    StoredFaultKindContactorK1Shorted = 12, //12	0Ch	Contactor K1 shorted
    StoredFaultKindContactorK2Shorted = 13, //13	0Dh	Contactor K2 is shorted
    StoredFaultKindContactorK3Shorted = 14, //14	0Eh	Contactor K3 shorted
    StoredFaultKindNoPrecharge = 15, //15	12h	No precharge
    StoredFaultKindOpenK2 = 16, //16	10h	Open K2
    StoredFaultKindExcessivePrechageTime = 17, //17	11h	Excessive precharge time
    StoredFaultKindEEPROMStackOverflow = 18 //18	12h	EEPROM stack overflow
};

typedef uint8_t StoredFaultKind;


enum _IOFlags {
    IOFlagPowerFromSource = 1 << 0, //	01h	there is power from the source
    IOFlagPowerFromLoad = 1 << 1, //	02h	there is power from the load
    IOFlagInterlockTripped = 1 << 2, // 	04h	the interlock is tripped
    IOFlagHardWireContactorRequest = 1 << 3, //	08h	there is a hard wire contactor request
    IOFlagCANContactorRequest = 1 << 4, //	10h	there is a CAN contactor request
    IOFlagHighLimitSet = 1 << 5, //	20h	the HLIM is set
    IOFlagLowLimitSet = 1 << 6,//	40h	the LLIM is set
    IOFlagFanIsOn = 1 << 7, //	80h	the Fan is on
};

typedef uint8_t IOFlags;


enum _LimitCause {
    LimitCauseErrorReadingValue = -1,
    LimitCauseNone = 0, //	0	No limit
    LimitCausePackVoltageTooLow, // 1	1	Pack voltage too low
    LimitCausePackVoltageTooHigh, // 2	2	Pack voltage too high
    LimitCauseCellVoltageTooLow, // 3	3	Cell voltage too low
    LimitCauseCellVoltageTooHigh, //4	4	Cell voltage too high
    LimitCauseTempTooHighToCharge, // 5	5	Temperature too high for charging
    LimitCauseTempTooLowToCharge, // 6	6	Temperature too low for charging
    LimitCauseTempTooHighToDischarge, // 7	7	Temperature too high for discharging
    LimitCauseTempTooLowToDischarge, // 8	8	Temperature too low for discharging
    LimitCauseChargingCurrentPeakTooLong, // 9	9	Charging current peak lasted too long
    LimitCauseDischargingCurrentPeakTooLong, //10	A	Discharging current peak lasted too long
};
typedef int8_t LimitCause;

#define ERROR_READING_LIMIT_VALUE -1

class CanbusClass
{
private:
    bool _initialized;
public:
    CanbusClass();
    bool init(CanSpeed canSpeed);
  
    // Elithion BMS options
    uint8_t getStateOfCharge(); // Returns a value from 0 to 100
    uint16_t getDepthOfDischarge(); // [Ah]
    
    LimitCause getChargeLimitCause();
    LimitCause getDischargeLimitCause();
    
    int8_t getChargeLimitValue(); // 0-100 percent; returns ERROR_READING_LIMIT_VALUE on error
    int8_t getDischargeLimitValue(); // 0-100 percent; returns ERROR_READING_LIMIT_VALUE on error
    
    // Pack
    float getPackVoltage(); // in volts
    float getMinVoltage(); // volts, indivdual cell voltage
    float getAvgVoltage();
    float getMaxVoltage();
    uint8_t getMinVoltageCellNumber();
    uint8_t getAvgVoltageCellNumber();
    uint8_t getMaxVoltageCellNumber();
    
    int getNumberOfCells();
    float getVoltageForCell(int cell);
    
    // Current
    float getPackCurrent();  // amps
    float getAverageSourceCurrent(); // amps
    float getAverageLoadCurrent();  // amps
    float getSourceCurrent(); // amps
    float getLoadCurrent(); // amps
    
    void getFaults(FaultKindOptions *presentFaults, StoredFaultKind *storedFault, FaultKindOptions *presentWarnings);
    void clearStoredFault();
    
    IOFlags getIOFlags();
    

};

#endif
