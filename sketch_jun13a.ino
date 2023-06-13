#include <DHT.h>
#include <Q2HX711.h>

#define DHTPIN 7
#define DHTTYPE DHT22

const byte MPS_OUT_pin = 2;
const byte MPS_SCK_pin = 3;
const int avg_size = 10;

float humidity, temperature, pressure;

DHT dht(DHTPIN, DHTTYPE);
Q2HX711 hx711(MPS_OUT_pin, MPS_SCK_pin);

void setup() {
  Serial.begin(9600);

  dht.begin();
}

void loop() {
  delay(2000);

  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  pressure = hx711.read()/100.0;

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("error: DHT22!");
    return;
  }

  Serial.print("Pressure: ");
  Serial.println(pressure);

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
}
