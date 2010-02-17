#include <OneWire.h>

// DS18S20 Temperature chip i/o

OneWire ds(2);  // on pin 10

//On what pins respective color is found on, pins needs to support PWM
#define RED 11
#define GREEN 10
#define BLUE 9 

void setup(void) {
	// initialize inputs/outputs
	pinMode(6, OUTPUT);
	pinMode(9, OUTPUT);
	// start serial port
	Serial.begin(115200);
}

#define READ_TEMP_EVERY 1000 // ms
void loop(void) {
	unsigned long lastRun = millis(); 
	while(lastRun + READ_TEMP_EVERY > millis())
	{
		readSerial();
	}	
	readTemp();
}

void readSerial()
{
	int inByte = Serial.read();
	switch (inByte)
	{
		case 'r':
			fade(RED);
			break;
		case 'g':
			fade(GREEN);
			break;
		case 'b':
			fade(BLUE);
			break;
	}
}

void fade(int PORT)
{
	delayMicroseconds(100); // delay so data have time to arrive
	int inByte = Serial.read(); // Get fade level for LED
	if (inByte != -1) // only change if data arrived
		analogWrite(PORT, inByte);
}

void readTemp(){
	byte i;
	byte present = 0;
	byte data[12];
	byte addr[8];
	if ( !ds.search(addr)) {
		ds.reset_search();
		return;
	}

	Serial.print("Sensor: Temp ID: ");
	for( i = 0; i < 8; i++) {
		Serial.print(addr[i], HEX);
		Serial.print(" ");
	}

	if ( OneWire::crc8( addr, 7) != addr[7]) {
		Serial.print("Sensor: Error CRC is not valid!\n");
		return;
	}

	if ( addr[0] != 0x10) {
		Serial.print("Sensor: Error Device is not a DS18S20 family device.\n");
		return;
	}

	ds.reset();
	ds.select(addr);
	ds.write(0x44,1);         // start conversion, with parasite power on at the end


	present = ds.reset();
	ds.select(addr);    
	ds.write(0xBE);         // Read Scratchpad

	for ( i = 0; i < 9; i++) {           // we need 9 bytes
		data[i] = ds.read();
	}
	printTemp(data[0], data[1], data[6]);
}

void printTemp(byte low, byte high, byte count_remain)
{
	if (high == 0xFF)
	{
		Serial.print("Temp: -");
		low = low ^ 0xFF;
	}
	else
	{
		Serial.print("Temp: ");
	}
	float temp = low/2 - 0.25 + float(16-count_remain)/16;
	Serial.print(int(temp));
	Serial.print(".");
	Serial.print(int(temp*10) % 10);
	Serial.println("C");
}
