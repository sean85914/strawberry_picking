// Download the library here and include to your Arduino IDE
// https://github.com/bogde/HX711

#include "HX711.h"

HX711 scale;

const double r = 0.100; // State noise, TBD
const double q = 0.205; // Measure noise, TBD
double var; // Initial variance
double mean; // Initial mean
double raw; // Raw data
char in; 
// One-dimension Kalman filter
double kalman(double measure, double last_mean, double last_var)
{
  double temp = last_var + r + q;
  double result = (last_mean * q + (last_var + r) * measure) / temp;
  result = (result <=0? 0 : result);
  var = (last_var + r) * q / temp;
  return result;
}

void setup() {
  
  Serial.begin(38400);
  // HX711 DOUT -> A1
  // HX711 SCK  -> A0
  scale.begin(A1, A0);
  // How to calibrate:
  // call scale.set_scale() and scale.tare()
  // Put known weight object onto your scale
  // and report the data from get_units()
  // devide the data by the known weight
  
  scale.set_scale(330.f); // After calibrarion
  scale.tare(); // Reset the scale
  mean = 0.0;
  var = 1.0;
}

void loop() {
  
  // Reset the scale if unsatisfied the offset value
  if(Serial.available()) {
    in = Serial.read();
    if(in == 'r') {
      scale.tare();
      Serial.println("Reset the scale...");
    }
  }
  // Get the raw data
  raw = scale.get_units(10);
  Serial.print("raw: "); Serial.print(raw); 
  // Weight should not less than zero, tune the value
  raw = (raw <=0 ? 0 : raw);
  // Kalman filter
  mean = kalman(raw, mean, var);
  Serial.print(" filtered: "); Serial.println(mean);
}
