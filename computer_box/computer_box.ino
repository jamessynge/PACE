#include <stdlib.h>
#include <OneWire.h>

#include <DHT.h>

#define DHTPIN 3 // DHT Temp & Humidity Pin
#define DHTTYPE DHT22   // DHT 22  (AM2302)

int ac_probe = 0;
int dc_probe = 1;

int led_pin = 13;
int led_value = LOW;

int fan_pin = 4;

int ds18_01_pin = 2;
const int num_ds18 = 3; // Number of DS18B20 Sensors
uint8_t sensors_address[num_ds18][8]; //here will store the sensors addresses for later use

//Temperature chip i/o
OneWire sensor_bus(ds18_01_pin); // on digital pin 2
float get_temperature (uint8_t *address);

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);

  pinMode(led_pin, OUTPUT);
  pinMode(ds18_01_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);

  dht.begin();
  Serial.println("DHT22found");

  // Turn on the fan
  digitalWrite(fan_pin, HIGH);
  Serial.println("Fan turned on");

  int x, y, c = 0;
  Serial.println("Starting to look for sensors...");
  for (x = 0; x < num_ds18; x++) {
    if (sensor_bus.search(sensors_address[x]))
      c++;
  }
  if (c > 0) {
    Serial.println("Found this sensors : ");
    for (x = 0; x < num_ds18; x++) {
      Serial.print("\tSensor ");
      Serial.print(x + 1);
      Serial.print(" at address : ");
      for (y = 0; y < 8; y++) {
        Serial.print(sensors_address[x][y], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  } else
    Serial.println("Didn't find any sensors");
}

void loop() {
  Serial.print("{ \"computer\": { ");
  read_voltages();
  Serial.print(",");
  read_temperature();
  Serial.println("} }");

  // Blink lights
  blink_led();
}

void blink_led() {
  led_value = ! led_value;
  digitalWrite(led_pin, led_value);
  delay(1000);
}

void toggle_fan() {
  // Turn Fan On
  if (digitalRead(fan_pin) == 'HIGH') {
    digitalWrite(fan_pin, LOW);
  } else {
    digitalWrite(fan_pin, HIGH);
  }
}

/*

DC Probe: ~730 = 11.53

*/
void read_voltages() {
  int ac_reading = analogRead(ac_probe);
  float ac_voltage = ac_reading / 1023 * 5;

  int dc_reading = analogRead(dc_probe);
  float dc_voltage = dc_reading * 0.0158;

  Serial.print("\"voltages\":{");
  Serial.print("\"ac\":"); Serial.print(ac_voltage); Serial.print(',');
  Serial.print("\"dc\":"); Serial.print(dc_voltage);
  Serial.print('}');
}

//// Reading temperature or humidity takes about 250 milliseconds!
//// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
void read_temperature() {
  float h = dht.readHumidity();
  float c = dht.readTemperature(); // Celsius

  // Check if any reads failed and exit early (to try again).
  // if (isnan(h) || isnan(t)) {
  //   Serial.println("Failed to read from DHT sensor!");
  //   return;
  // }

  Serial.print("\"temp\":{");
  Serial.print("\"h\":"); Serial.print(h); Serial.print(',');
  Serial.print("\"c\":"); Serial.print(c); Serial.print(',');

  for (int x = 0; x < num_ds18; x++) {
    Serial.print("\"temp_");
    Serial.print(x + 1);
    Serial.print("\":");
    Serial.print(get_temperature(sensors_address[x]));

    if (x < num_ds18 - 1) {
      Serial.print(",");
    }
  }

  Serial.print('}');
}

/*
float get_ds18b20_temp() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    Serial.println("No sensors on chain");
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}
*/

float get_temperature(uint8_t *address) {
  byte data[12];
  int x;
  sensor_bus.reset();
  sensor_bus.select(address);
  sensor_bus.write(0x44, 1);

  sensor_bus.reset();
  sensor_bus.select(address);
  sensor_bus.write(0xBE, 1);

  for (x = 0; x < 9; x++)
    data[x] = sensor_bus.read();

  int tr = data[0];
  if (data[1] > 0x80) {
    tr = !tr + 1;
    tr = tr * -1;
  }
  int cpc = data[7];
  int cr = data[6];

  tr = tr >> 1;

  float temperature = tr - (float)0.25 + (cpc - cr) / (float)cpc;

  return temperature;
}
