/* Revised from Arduino-Temperature-Control-Library/examples/Simple/Simple.pde
   Use DS18B20 to measure the temperature and show through serial
   Editor: Sean, Lu
   Last update: 3/31, 2018
*/
/*
   Update: Revise wrong word
*/

// Include the libraries we need

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
// ----------------------
#define ONE_WIRE_BUS 2
// ----------------------

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)

OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire referance to Dallas Temperature
DallasTemperature sensors(&oneWire);

// Setup function. Initial sensors here
void setup(void)
{
	// Start Serial port
	Serial.begin(9600);
	// Start up the library
	sensors.begin();
}

// Loop function. Get and show temperature here
void loop(void)
{
	// call sensors.requestTemperatures() to issue a global temperature
	// request to all devices on the bus
	sensors.requestTemperatures();
	// Send the command to get temperature
	// After we got the temperature, we can print them here
	// We use the function ByIndex, and as an example get the temperature
	// from the first sensor only
	Serial.println(sensors.getTempCByIndex(0));
}
