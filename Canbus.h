// Can bus reading for the elithion BMS - by corbin dunn, modified from other examples.
// www.corbinstreehouse.com
// Nov 16, 2012

#ifndef CANBUS_H
#define CANBUS_H


#define CANSPEED_125 	7		// CAN speed at 125 kbps
#define CANSPEED_250  	3		// CAN speed at 250 kbps
#define CANSPEED_500	1		// CAN speed at 500 kbps

typedef enum {
    CanSpeed500 = 1,
    CanSpeed250 = 3,
    CanSpeed125 = 7,
} CanSpeed;

class CanbusClass
{
  public:
    CanbusClass();
    bool init(CanSpeed canSpeed);
  
    // Elithion BMS options
    uint8_t stateOfCharge(); // Returns a value from 0 to 100
    
    
    
//	char message_tx(void);
//	char message_rx(unsigned char *buffer);
//	char ecu_req(unsigned char pid,  char *buffer);

};

#endif
