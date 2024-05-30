#include "SimpleModbusMaster.h"

/**
   CODE RS485
   --------------------------------------
*/
#define baud 9600
#define timeout 5000
#define polling 50 // the scan rate
#define retry_count 10

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 0

unsigned int VAN[2], VBN[2], VCN[2], VLN[2], AA[2], AB[2], AC[2], AAVG[2], PFTOT[2], FRQ[2];
float D_VAN, D_VBN, D_VCN, D_VLN, D_AA, D_AB, D_AC, D_AAVG, D_PFTOT, D_FRQ;
unsigned long timer;

enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,
  PACKET9,
  PACKET10,
  // leave this last entry
  TOTAL_NO_OF_PACKETS
};

// Create an array of Packets for modbus_update()
Packet packets[TOTAL_NO_OF_PACKETS];
packetPointer packet1 = &packets[PACKET1];
packetPointer packet2 = &packets[PACKET2];
packetPointer packet3 = &packets[PACKET3];
packetPointer packet4 = &packets[PACKET4];
packetPointer packet5 = &packets[PACKET5];
packetPointer packet6 = &packets[PACKET6];
packetPointer packet7 = &packets[PACKET7];
packetPointer packet8 = &packets[PACKET8];
packetPointer packet9 = &packets[PACKET9];
packetPointer packet10 = &packets[PACKET10];

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

void setup() {
  Serial.begin(115200);

  packet1->id = packet2->id = packet3->id = packet4->id = packet5->id =
                                packet6->id = packet7->id = packet8->id = packet9->id = packet10->id = 1;
  packet1->function = packet2->function = packet3->function = packet4->function = packet5->function =
      packet6->function = packet7->function = packet8->function = packet9->function = packet10->function = READ_HOLDING_REGISTERS;
  packet1->no_of_registers = packet2->no_of_registers = packet3->no_of_registers = packet4->no_of_registers = packet5->no_of_registers =
                               packet6->no_of_registers = packet7->no_of_registers = packet8->no_of_registers = packet9->no_of_registers = packet10->no_of_registers = 2;

  packet1->address = 3027;
  packet1->register_array = VAN;
  packet2->address = 3029;
  packet2->register_array = VBN;
  packet3->address = 3031;
  packet3->register_array = VCN;
  packet4->address = 3035;
  packet4->register_array = VLN;

  packet5->address = 2999;
  packet5->register_array = AA;
  packet6->address = 3001;
  packet6->register_array = AB;
  packet7->address = 3003;
  packet7->register_array = AC;
  packet8->address = 3009;
  packet8->register_array = AAVG;

  packet9->address = 3191;
  packet9->register_array = PFTOT;
  packet10->address = 3109;
  packet10->register_array = FRQ;

  modbus_configure(baud, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS);

  timer = millis();

}

void loop() {
  unsigned int connection_status = modbus_update(packets);
  if (connection_status != TOTAL_NO_OF_PACKETS)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);

  long newTimer = millis();
  if (newTimer -  timer >= 1000) {
    D_VAN = f_2uint_float(VAN[1], VAN[0]);
    D_VBN = f_2uint_float(VBN[1], VBN[0]);
    D_VCN = f_2uint_float(VCN[1], VCN[0]);
    D_VLN = f_2uint_float(VLN[1], VLN[0]);

    D_AA = f_2uint_float(AA[1], AA[0]);
    D_AB = f_2uint_float(AB[1], AB[0]);
    D_AC = f_2uint_float(AC[1], AC[0]);
    D_AAVG = f_2uint_float(AAVG[1], AAVG[0]);

    D_PFTOT = f_2uint_float(PFTOT[1], PFTOT[0]);
    D_FRQ = f_2uint_float(FRQ[1], FRQ[0]);
    
    Serial.println((String)
                   "VAN = " + D_VAN +
                   " || VBN = " + D_VBN +
                   " || VCN = " + D_VCN +
                   " || VLN = " + D_VLN +
                   " || AA = " + D_AA +
                   " || AB = " + D_AB +
                   " || AC = " + D_AC +
                   " || AAVG = " + D_AAVG +
                   " || PF = " + D_PFTOT +
                   " || FRQ = " + D_FRQ);
  
  //Serial.println((String)PFTOT[1] + "  " + PFTOT[0]); 
  timer = newTimer;
  }
}
