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

#define DEBUG 1


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

#define ELITHION_PID_RESPONSE_MODE_DEFAULT 0x50

CanbusClass::CanbusClass() {
    // Initialize defaults
}

bool CanbusClass::init(CanSpeed canSpeed) {
    return mcp2515_init(canSpeed);
}

// Standard offsets for elithion
#define NUM_BYTES_OFFSET 0
#define MODE_OFFSET 1
#define PID_HI_OFFSET 2
#define PID_LO_OFFSET 3


static bool sendAndReceiveMessage(tCAN *message, uint16_t pid_reply, uint8_t response_mode, uint8_t response_pid_hi, uint8_t response_pid_low) {
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
	if (mcp2515_send_message(message)) {
        Serial.println("message sent");
        // Timeout should be smarter than a loop!
        int timeout = 0;
        while (timeout < 4000) {
            timeout++;
            if (mcp2515_check_message()) {
                Serial.println("message available");
                if (mcp2515_get_message(message)) {
                    // See if we got the right response; making sure we got enough bytes (at least 3 to read the high and low
                    if ((message->id == pid_reply) && (message->data[NUM_BYTES_OFFSET] >= 3) && (message->data[MODE_OFFSET] == response_mode) && (message->data[PID_HI_OFFSET] == response_pid_hi) && (message->data[PID_LO_OFFSET] == response_pid_low)) {
                        return true;
                    } else {
#if DEBUG
                        Serial.print("reply id: 0x");
                        Serial.print(message->id, HEX);
                        Serial.print("expecting id: 0x");
                        Serial.print(pid_reply, HEX);
                        Serial.print(" bytes: ");
                        Serial.print(message->data[NUM_BYTES_OFFSET], HEX);
                        Serial.print(" mode: ");
                        Serial.print(message->data[MODE_OFFSET], HEX);
                        Serial.print("expecting mode: 0x");
                        Serial.print(response_mode, HEX);
                        
                        Serial.print(" pid hi: ");
                        Serial.print(message->data[PID_HI_OFFSET], HEX);
                        
                        Serial.print("expecting hi: 0x");
                        Serial.print(response_pid_hi, HEX);
                        
                        
                        Serial.print(" pid low: ");
                        Serial.print(message->data[PID_LO_OFFSET], HEX);

                        Serial.print("expecting low: 0x");
                        Serial.print(response_pid_low, HEX);
                        
                        Serial.println("");
#endif
                    }
                } else {
#if DEBUG
                    Serial.println("couldn't get the message even though one was available");
#endif
                }
            }
        }
    } else {
#if DEBUG
        Serial.println("message NOT sent");
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

// readFromCanBusPID(ELITHION_PID_REQUEST, ELITHION_PID_MODE_PACK, ELITHION_PID_SOC, 0, ELITHION_PID_RESPONSE,

//static bool readElithionMessageFromCanBus(tCAN *message, uint8_t mode, uint8_t pid_hi, uint8_t pid_low, uint16_t pid_response_mode, uint16_t pid_response_hi, uint16_t pid_response_low)
//{
//    setupElithionCanMessage(message, mode, pid_hi, pid_low);
//    return sendAndReceiveMessage(message, ELITHION_PID_RESPONSE, pid_response_mode, pid_response_hi, pid_response_low);
//}

static bool readElithionDefaultMessageFromCanBus(tCAN *message, uint8_t pid_hi, uint8_t pid_low, uint16_t pid_response_hi, uint16_t pid_response_low)
{
    // most messages have a standard mode and standard response
    setupElithionCanMessage(message, ELITHION_PID_MODE_DEFAULT, pid_hi, pid_low);
    return sendAndReceiveMessage(message, ELITHION_PID_RESPONSE, ELITHION_PID_RESPONSE_MODE_DEFAULT, pid_response_hi, pid_response_low);
}

uint8_t CanbusClass::stateOfCharge() {
	tCAN message;
    if (readElithionDefaultMessageFromCanBus(&message, ELITHION_PID_PACK_SOC, 0/*pid_low*/, 0x50/*response pid hi*/, 0/*response pid low*/)) {
        Serial.println("got data!");
        return message.data[4]; // percentage 
    } else {
#if DEBUG
        Serial.println("could not read SOC");
#endif
        return 0;
    }
}


/*

char CanbusClass::message_rx(unsigned char *buffer) {
		tCAN message;
	
		if (mcp2515_check_message()) {
		
			
			// Lese die Nachricht aus dem Puffern des MCP2515
			if (mcp2515_get_message(&message)) {
			//	print_can_message(&message);
			//	PRINT("\n");
				buffer[0] = message.data[0];
				buffer[1] = message.data[1];
				buffer[2] = message.data[2];
				buffer[3] = message.data[3];
				buffer[4] = message.data[4];
				buffer[5] = message.data[5];
				buffer[6] = message.data[6];
				buffer[7] = message.data[7];								
//				buffer[] = message[];
//				buffer[] = message[];
//				buffer[] = message[];
//				buffer[] = message[];																												
			}
			else {
			//	PRINT("Kann die Nachricht nicht auslesen\n\n");
			}
		}

}

char CanbusClass::message_tx(void) {
	tCAN message;


	// einige Testwerte
	message.id = PID_REQUEST;
	message.header.rtr = 0;
	message.header.length = 8;
	message.data[0] = 0x02;
	message.data[1] = 0x01;
	message.data[2] = 0x05;
	message.data[3] = 0x00;
	message.data[4] = 0x00;
	message.data[5] = 0x00;
	message.data[6] = 0x00;
	message.data[7] = 0x00;						
	
	
	
	
//	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), (1<<REQOP1));	
		mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		
	if (mcp2515_send_message(&message)) {
		//	SET(LED2_HIGH);
		return 1;
	
	}
	else {
	//	PRINT("Fehler: konnte die Nachricht nicht auslesen\n\n");
	return 0;
	}
return 1;
 
}

 */
//     if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1)          // Request for engine RPM




