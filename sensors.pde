#include <OneWire.h>

// DS18S20 Temperature chip i/o

OneWire ds(10);  // on pin 10

void setup(void) {
  // initialize inputs/outputs
  // start serial port
  Serial.begin(9600);
}

void loop(void) {
  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];
  delay(1000);
  if ( !ds.search(addr)) {
 //   Serial.print("No more addresses.\n");
    ds.reset_search();
    return;
  }

  Serial.print("TempSensor=");
  for( i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.print("CRC is not valid!\n");
    return;
  }

  if ( addr[0] != 0x10) {
    Serial.print("Device is not a DS18S20 family device.\n");
    return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  //delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("P=");
  //Serial.print(present,HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  //  Serial.print(data[i], HEX);
  //  Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print( OneWire::crc8( data, 8), HEX);
  //Serial.println();
  printTemp(data[0], data[1]);
}

void printTemp(byte low, byte high)
{
  if (high == 0xFF)
  {
    Serial.print("Temp: -");
    Serial.print(low ^ 0xFF / 2);
    Serial.print(".");
    Serial.print(low % 2 * 5);
  }
  else
  {
    Serial.print("Temp: ");
    Serial.print(low/2);
    Serial.print(".");
    Serial.print(low % 2 * 5);
  }
  Serial.println("C");
}
