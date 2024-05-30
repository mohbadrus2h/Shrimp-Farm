#include "SimpleModbusMaster.h"

/**
 * CODE RS485
 * --------------------------------------
 */
#define baud 9600
#define timeout 1000
#define polling 200 // the scan rate
#define retry_count 10 

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 0 

unsigned int volt[2];
float tegangan = 0;
unsigned long timer;


enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET2,
  // leave this last entry
  TOTAL_NO_OF_PACKETS
};

// Create an array of Packets for modbus_update()
Packet packets[TOTAL_NO_OF_PACKETS];
packetPointer packet1 = &packets[PACKET1];

float f_2uint_float(unsigned int uint1, unsigned int uint2) {    // reconstruct the float from 2 unsigned integers
  union f_2uint {
    float f;
    uint16_t i[2];
  };
  union f_2uint f_number;
  f_number.i[0] = uint1;
  f_number.i[1] = uint2;
  return f_number.f;
}

void setup(){
  Serial.begin(115200);
  
  // CODE RS485
  //--------------------------------
  packet1->id = 1;
  packet1->function = READ_HOLDING_REGISTERS;
  packet1->address = 3109;
  packet1->no_of_registers = 2;
  packet1->register_array = volt;
  
  modbus_configure(baud, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS);
  
  timer = millis();
  
}

void loop(){
  unsigned int connection_status = modbus_update(packets);
  if (connection_status != TOTAL_NO_OF_PACKETS)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);
    
  long newTimer = millis();
  if(newTimer -  timer >= 1000){
    Serial.println();
    Serial.print("VOLTAGE : ");
    tegangan = f_2uint_float(volt[1],volt[0]);
    Serial.println(tegangan);
    timer = newTimer;
  }
}
