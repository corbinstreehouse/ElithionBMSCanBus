// Can bus reading for the elithion BMS - by corbin dunn, modified from other examples.
// www.corbinstreehouse.com
// Nov 16, 2012

#if ARDUINO>=100
    #include <Arduino.h> // Arduino 1.0
#else
    #include <Wprogram.h> // Arduino 0022
#endif

#include "Canbus.h"

#include <stdint.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#include "pins_arduino.h"
#include <inttypes.h>
//#include "global.h"
#include "mcp2515.h"
//#include "defaults.h"

#include <HardwareSerial.h>

#define DEBUG 0
#define MOCK_DATA 0

#define TIMEOUT_DURATION 40 // Long enough? X milliseconds

const char *FaultKindMessages[MAX_FAULT_MESSAGES] = {
    "Driving and plugged in",
    "Interlock tripped",
    "Communication fault",
    "Charge over current",
    "Discharge over current",
    "Over Temperature",
    "Under voltage",
    "Over voltage",
    "BMS not found on CAN Bus",
    "CAN Bus init failed"
};


// http://lithiumate.elithion.com/php/menu_setup.php#Standard_output_messages
// TODO: this are all configurable and would be nice to set for the class
#define ELITHION_CAN_ID 0x620


#define ELITHION_PID_REQUEST 0x0745
#define ELITHION_PID_RESPONSE ELITHION_PID_REQUEST + 0x08 // 0x074D // The response ID is 08h more than the request ID
//Data length is 8 data bytes regardless of whether sme or all bytes are actually used (unused bytes are set at 0)
// PIDs: http://lithiumate.elithion.com/xls/Lithiumate_PIDs.xls
// EEPROM data that can be controlled: http://lithiumate.elithion.com/xls/eeprom_data.xls

// TODOO: maybe make these tables in memory instead of commands to save space? 
// Pack messages
#define ELITHION_PID_MODE_DEFAULT 0x10
// TODO: STATUS -> RESISTENCE
#define ELITHION_PID_PACK_SOC 0x50
#define ELITHION_PID_PACK_CAPACITY 0x51
#define ELITHION_PID_PACK_DOD 0x52
#define ELITHION_PID_PACK_POWER 0x53
#define ELITHION_PID_PACK_ENERGY_IN 0x54
#define ELITHION_PID_PACK_ENERGY_OUT 0x55
#define ELITHION_PID_PACK_SOH 0x56

#define ELITHION_PID_FAULT 0x62

#define ELITHION_PID_RESPONSE_MODE_DEFAULT 0x50

CanbusClass::CanbusClass() {
    // Initialize defaults
}

bool CanbusClass::init(CanSpeed canSpeed) {
    _initialized =  mcp2515_init(canSpeed);
    return _initialized;
}

// Standard offsets for elithion
#define NUM_BYTES_OFFSET 0
#define MODE_OFFSET 1
#define PID_HI_OFFSET 2
#define PID_LO_OFFSET 3


static bool sendAndReceiveMessage(tCAN *message, uint16_t pid_reply, uint8_t response_mode, uint8_t response_pid_hi, uint8_t response_pid_low) {
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
	if (mcp2515_send_message(message)) {
        long startTime = millis();
        bool timeout = false;
        while (!timeout) {
            if (mcp2515_check_message()) {
                if (mcp2515_get_message(message)) {
                    // See if we got the right response; making sure we got enough bytes (at least 3 to read the high and low
                    if ((message->id == pid_reply) && (message->data[NUM_BYTES_OFFSET] >= 3) && (message->data[MODE_OFFSET] == response_mode) && (message->data[PID_HI_OFFSET] == response_pid_hi) && (message->data[PID_LO_OFFSET] == response_pid_low)) {
                        return true;
                    } else {
//#if DEBUG
//                        Serial.print("reply id: 0x");
//                        Serial.print(message->id, HEX);
//                        Serial.print("expecting id: 0x");
//                        Serial.print(pid_reply, HEX);
//                        Serial.print(" bytes: ");
//                        Serial.print(message->data[NUM_BYTES_OFFSET], HEX);
//                        Serial.print(" mode: ");
//                        Serial.print(message->data[MODE_OFFSET], HEX);
//                        Serial.print("expecting mode: 0x");
//                        Serial.print(response_mode, HEX);
//                        
//                        Serial.print(" pid hi: ");
//                        Serial.print(message->data[PID_HI_OFFSET], HEX);
//                        
//                        Serial.print("expecting hi: 0x");
//                        Serial.print(response_pid_hi, HEX);
//                        
//                        
//                        Serial.print(" pid low: ");
//                        Serial.print(message->data[PID_LO_OFFSET], HEX);
//
//                        Serial.print("expecting low: 0x");
//                        Serial.print(response_pid_low, HEX);
//                        
//                        Serial.println("");
//#endif
                    }
                } else {
#if DEBUG
                    Serial.println("couldn't get the message even though one was available");
#endif
                }
            }
            
            // See if we hit the timeout duration
            long endTime = millis();
            if (endTime < startTime) {
                // roll over
                timeout = true;
            } else {
                if ((endTime - startTime) > TIMEOUT_DURATION) {
                    timeout = true;
                }
            }
        }
    } else {
#if DEBUG
        Serial.println("ERROR: message NOT sent");
#endif
    }
 	return false;
}

static void setupElithionCanMessage(tCAN *message, uint8_t mode, uint8_t pid_hi, uint8_t pid_low) {
	message->id = ELITHION_PID_REQUEST;
	message->header.rtr = 0; // not sure what this is for yet
	message->header.length = 8; // 8 bytes in the data
	message->data[NUM_BYTES_OFFSET] = 3; // additional bytes to follow (hardcoded for 3 -- everything except the settings use this)
	message->data[MODE_OFFSET] = mode; // See http://lithiumate.elithion.com/xls/Lithiumate_PIDs.xls
	message->data[PID_HI_OFFSET] = pid_hi;
	message->data[PID_LO_OFFSET] = pid_low;
    // The rest are unused
	message->data[4] = 0x00;
	message->data[5] = 0x00;
	message->data[6] = 0x00;
	message->data[7] = 0x00;
}

static bool readElithionDefaultMessageFromCanBus(tCAN *message, uint8_t pid_hi, uint8_t pid_low) {
    // most messages have a standard mode and standard response so make this commonized
    setupElithionCanMessage(message, ELITHION_PID_MODE_DEFAULT, pid_hi, pid_low);
    return sendAndReceiveMessage(message, ELITHION_PID_RESPONSE, ELITHION_PID_RESPONSE_MODE_DEFAULT, pid_hi, pid_low);
}

static int readElithionTwoByteValue(uint8_t pid_hi) {
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, pid_hi, 0)) {
        uint8_t msb = message.data[4];
        uint8_t lsb = message.data[5];
        int result = ((msb << 8) | lsb);
        return result;
    } else {
        return 0;
    }
}

static uint8_t readElithionSingleByteValue(uint8_t pid_hi) {
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, pid_hi, 0)) {
        return message.data[4];
    } else {
        return 0;
    }
}

uint8_t CanbusClass::getStateOfCharge() {
    return readElithionSingleByteValue(ELITHION_PID_PACK_SOC);
}

uint16_t CanbusClass::getDepthOfDischarge() {
#if MOCK_DATA
    return 10; // ah
#endif
    return readElithionTwoByteValue(ELITHION_PID_PACK_DOD);
}

float CanbusClass::getPackCurrent() {
    return readElithionTwoByteValue(0x68) * 100.0 / 1000.0; // Units returned is 100mA. Multiply by 100 to get mA. Then divide by 1000 to get amps.
}

float CanbusClass::getAverageSourceCurrent() {
#if MOCK_DATA
    return 30;
#endif
    return readElithionTwoByteValue(0x69) * 100.0 / 1000.0;
}

float CanbusClass::getAverageLoadCurrent() {
    return readElithionTwoByteValue(0x6A) * 100.0 / 1000.0;
}

float CanbusClass::getSourceCurrent() {
    return readElithionTwoByteValue(0x6B) * 100.0 / 1000.0;
}

float CanbusClass::getLoadCurrent() {
    return readElithionTwoByteValue(0x6C) * 100.0 / 1000.0;
}

LimitCause CanbusClass::getChargeLimitCause() {
#if MOCK_DATA
    static long lastTime = 0;
    if (lastTime != 0) {
        long endTime = millis();
        LimitCause r = LimitCauseNone;
        while ((endTime - lastTime) > 5000 && r < LimitCauseDischargingCurrentPeakTooLong) {
            r++;
            endTime -= 5000;
        }
        if (r == LimitCauseDischargingCurrentPeakTooLong) {
            r = 0;
        }
        return r;
    } else {
        lastTime = millis();
    }
#endif
 	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, 0x64, 0)) {
        return message.data[5];
    } else {
        return 0;
    }
}

LimitCause CanbusClass::getDischargeLimitCause() {
 	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, 0x65, 0)) {
        return message.data[5];
    } else {
        return 0;
    }
}

#define ROUND_255_AS_PERCENTAGE(v) round(100.0*(float)(v)/255.0)

int8_t CanbusClass::getChargeLimitValue() {
#if MOCK_DATA
    return 90;
#endif
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, 0x64, 0)) {
#if 0 // DEBUG
        Serial.print("charg limit:");
        Serial.println(message.data[4]);
        Serial.print("charg limit per:");
        Serial.println(ROUND_255_AS_PERCENTAGE(message.data[4]));
#endif
        return ROUND_255_AS_PERCENTAGE(message.data[4]);
    } else {
        return ERROR_READING_LIMIT_VALUE;
    }
}

int8_t CanbusClass::getDischargeLimitValue() {
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, 0x65, 0)) {
        return ROUND_255_AS_PERCENTAGE(message.data[4]);
    } else {
        return ERROR_READING_LIMIT_VALUE;
    }
}

float CanbusClass::getPackVoltage() {
#if MOCK_DATA
    static long lastTime = 0;
    long endTime = millis();
    if ((endTime - lastTime) > 2000) {
        lastTime = endTime;
        return 18;
    } else {
        return 132;
    }
#endif
    return readElithionTwoByteValue(0x46) * 100.0 / 1000.0; // in 100mV
}

#define CONVERT_ENCODED_MVOLT_TO_VOLT(v) (2.0 + (v)*10.0/1000.0) // in 10mv increments, plus 2.0v

float CanbusClass::getMinVoltage() {
#if MOCK_DATA
    return CONVERT_ENCODED_MVOLT_TO_VOLT(30);
#endif
    return CONVERT_ENCODED_MVOLT_TO_VOLT(readElithionSingleByteValue(0x43));
}

float CanbusClass::getAvgVoltage() {
#if MOCK_DATA
    return CONVERT_ENCODED_MVOLT_TO_VOLT(50);
#endif
    return CONVERT_ENCODED_MVOLT_TO_VOLT(readElithionSingleByteValue(0x44));
}

float CanbusClass::getMaxVoltage() {
#if MOCK_DATA
    return CONVERT_ENCODED_MVOLT_TO_VOLT(60);
#endif
    return CONVERT_ENCODED_MVOLT_TO_VOLT(readElithionSingleByteValue(0x45));
}

uint8_t CanbusClass::getMinVoltageCellNumber() {
    // this is racy...and min voltage + cell should be read at the same time
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, 0x43, 0)) {
        return message.data[5];
    } else {
        return 0;
    }
}

uint8_t CanbusClass::getAvgVoltageCellNumber() {
    // this is racy...and min voltage + cell should be read at the same time
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, 0x44, 0)) {
        return message.data[5];
    } else {
        return 0;
    }
}

uint8_t CanbusClass::getMaxVoltageCellNumber() {
    // this is racy...and min voltage + cell should be read at the same time
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, 0x45, 0)) {
        return message.data[5];
    } else {
        return 0;
    }
}


IOFlags CanbusClass::getIOFlags() {
#if MOCK_DATA
    return IOFlagPowerFromSource; // charging
#endif
    return readElithionSingleByteValue(0x66);
}

void CanbusClass::clearStoredFault() {
	tCAN message;
    setupElithionCanMessage(&message, 0x14, ELITHION_PID_FAULT, 0);
    // we ignore the result
    bool result = sendAndReceiveMessage(&message, 0x54, ELITHION_PID_RESPONSE_MODE_DEFAULT, ELITHION_PID_FAULT, 0);
#if DEBUG
    if (result) {
        Serial.println("faults should ahve been cleared");
    } else {
        Serial.println("failed to clear faults");
    }
#endif
}

void CanbusClass::getFaults(FaultKindOptions *presentFaults, StoredFaultKind *storedFault, FaultKindOptions *presentWarnings) {
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, ELITHION_PID_FAULT, 0/*pid_low*/)) {
        *presentFaults = message.data[4];
        *storedFault = message.data[5];
        *presentWarnings = message.data[6];
    } else {
        // Indicate a fault, in that we couldn't read it!
        if (!_initialized) {
            *presentFaults = FaultKindCanBusFailedInitialization;
        } else {
            *presentFaults = FaultKindCantFindBMSOnCanBus;
        }
        *storedFault = 0;
        *presentWarnings = 0;
        
#if MOCK_DATA
        static long lastTime = 0;
        if (lastTime != 0) {
            long endTime = millis();
            if ((endTime - lastTime) > 10000) {
//                lastTime = endTime;
                *presentFaults = 0;
//                *presentFaults = FaultKindCantFindBMSOnCanBus;
            } else if ((endTime - lastTime) > 5000) {
                *presentWarnings = FaultKindDischargeOverCurrent;
  //              *presentFaults = 0;
            }
        } else {
            lastTime = millis();
        }
#endif
    }
    
#if 0 // DEBUG
    Serial.print("faults:");
    Serial.println(*presentFaults, 16);
    Serial.print("stored:");
    Serial.println(*storedFault, 16);
    Serial.print("warnings:");
    Serial.println(*presentWarnings, 16);
#endif
    
}