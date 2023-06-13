// External Libs:
// https://github.com/queuetue/Q2-HX711-Arduino-Library
// https://github.com/solvek/CO2Sensor/

#include <DHT.h>
#include <Q2HX711.h>
#include <CO2Sensor.h>

#define DHTPIN 7
#define DHTTYPE DHT22

const byte MPS_OUT_pin = 2;
const byte MPS_SCK_pin = 3;

float humidity, temperature, pressure, co2;

DHT temp_dht(DHTPIN, DHTTYPE);
Q2HX711 pressure_hx711(MPS_OUT_pin, MPS_SCK_pin);
CO2Sensor co2_mg811(A0, 0.99, 100);

void setup() {
  Serial.begin(9600);

  temp_dht.begin();
  co2_mg811.calibrate();
}

void loop() {
  delay(1000);

  humidity = temp_dht.readHumidity();
  temperature = temp_dht.readTemperature();
  pressure = pressure_hx711.read() / 100.0;
  co2 = co2_mg811.read();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("error: DHT22!");
    return;
  }

  if (isnan(pressure)) {
    Serial.println("error: HX711!");
    return;
  }

  if (isnan(co2)) {
    Serial.println("error: MG811!");
    return;
  }

  Serial.println();

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");

  Serial.print("Pressure: ");
  Serial.println(pressure);

  Serial.print("CO2: ");
  Serial.print(co2);
  Serial.println(" ppm");
}
