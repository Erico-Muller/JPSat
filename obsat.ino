// External Libs:
// https://github.com/queuetue/Q2-HX711-Arduino-Library
// https://github.com/solvek/CO2Sensor/

#include <DHT.h>
#include <Q2HX711.h>
#include <CO2Sensor.h>
#include <BH1750.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_AMG88xx.h>
#include <MPU6050_tockn.h>

#define DHT_PIN 7
#define DHT_TYPE DHT22

#define MPS_OUT 10
#define MPS_SCK 5

struct {
  float humidity;
  float temperature;
  float pressure;
  float co2;
  float light;
  float coord[2]; // lng, lat
  float term_cam_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
} payload;

bool is_there_sd = false;

DHT temp_dht(DHT_PIN, DHT_TYPE);
Q2HX711 pressure_hx711(MPS_OUT, MPS_SCK);
CO2Sensor co2_mg811(A0, 0.99, 100);
BH1750 light_bh1750;
File fs;
TinyGPSPlus gps;
SoftwareSerial ss(3, 2);
Adafruit_AMG88xx amg;
MPU6050 acc_mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  ss.begin(9600);

  temp_dht.begin();
  co2_mg811.calibrate();

  Wire.begin();
  light_bh1750.begin();

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
  } else {
    sd = false;
  }

  if (!amg.begin()) {
    Serial.println("Could not find a valid AMG8833 sensor, check wiring!");
    while (1);
  }

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  delay(500);

  //read_temperature_and_humidity();
  //read_pressure();
  //read_co2();
  //read_light();
  //read_acc();
  //read_gps();
  //read_term_camera();

  Serial.println();
  //print_temperature_and_humidity();
  //print_pressure();
  //print_co2();
  //print_light();
  //print_acc();
  //print_gps();
  //print_term_camera();
}

void read_temperature_and_humidity() {
  temperature = temp_dht.readTemperature();
  humidity = temp_dht.readHumidity();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("error: DHT22!");
    return;
  }

  if (sd) {
    fs = SD.open("temperature_and_humidity.txt", FILE_WRITE);

    // [temp] [humid]
    if (fs) {
      fs.print(temperature);
      fs.print("\t");
      fs.println(humidity);
      fs.close();
    }
  }
}

void print_temperature_and_humidity() {
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
}

void read_pressure() {
  pressure = pressure_hx711.read() / 100.0;
  
  if (isnan(pressure)) {
    Serial.println("error: HX711!");
    return;
  }

  if (sd) {
    fs = SD.open("pressure.txt", FILE_WRITE);

    if (fs) {
      fs.println(temperature);
      fs.close();
    }
  }
}

void print_pressure() {
  Serial.print("Pressure: ");
  Serial.println(pressure);
}

void read_co2() {
  co2 = co2_mg811.read();

  if (isnan(co2)) {
    Serial.println("error: MG811!");
    return;
  }

  if (sd) {
    fs = SD.open("co2.txt", FILE_WRITE);

    if (fs) {
      fs.println(co2);
      fs.close();
    }
  }
}

void print_co2() {
  Serial.print("CO2: ");
  Serial.print(co2);
  Serial.println(" ppm");
}

void read_light() {
  light = light_bh1750.readLightLevel();

  if (isnan(light)) {
    Serial.println("error: BH1750!");
    return;
  }

  if (sd) {
    fs = SD.open("light.txt", FILE_WRITE);

    if (fs) {
      fs.println(light);
      fs.close();
    }
  }
}

void print_light() {
  Serial.print("Light: ");
  Serial.print(light);
  Serial.println(" lx");
}

void read_gps() {
  if (ss.available() > 0)
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        lng = gps.location.lng();
        lat = gps.location.lat();

        if (sd) {
          fs = SD.open("gps.txt", FILE_WRITE);

          // [lng] [lat]
          if (fs) {
            fs.print(lng);
            fs.print(",");
            fs.println(lat);
            fs.close();
          }
        }
      } else {
        Serial.println("error: GPS!");
      }
    }
}

void print_gps() {
  Serial.print("Location: ");
  Serial.print(lng, 6);
  Serial.print(",");
  Serial.println(lat, 6);
}

void read_term_camera() {
  amg.readPixels(term_pixels);

  if (sd) {
    fs = SD.open("term_camera.txt", FILE_WRITE);

    if (fs) {
      for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
        fs.print(term_pixels[i]);
        fs.print("\t");
      }
      fs.println();
      fs.close();
    }
  }
}

void print_term_camera() {
  for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    Serial.print(term_pixels[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void read_acc() {
  mpu6050.update();

  if (sd) {
    fs = SD.open("mpu.txt", FILE_WRITE);

    if (fs) {
      fs.println("=======================================================");
      fs.print("temp : ");fs.println(mpu6050.getTemp());
      fs.print("accX : ");fs.print(mpu6050.getAccX());
      fs.print("\taccY : ");fs.print(mpu6050.getAccY());
      fs.print("\taccZ : ");fs.println(mpu6050.getAccZ());
  
      fs.print("gyroX : ");fs.print(mpu6050.getGyroX());
      fs.print("\tgyroY : ");fs.print(mpu6050.getGyroY());
      fs.print("\tgyroZ : ");fs.println(mpu6050.getGyroZ());
  
      fs.print("accAngleX : ");fs.print(mpu6050.getAccAngleX());
      fs.print("\taccAngleY : ");fs.println(mpu6050.getAccAngleY());
  
      fs.print("gyroAngleX : ");fs.print(mpu6050.getGyroAngleX());
      fs.print("\tgyroAngleY : ");fs.print(mpu6050.getGyroAngleY());
      fs.print("\tgyroAngleZ : ");fs.println(mpu6050.getGyroAngleZ());
    
      fs.print("angleX : ");fs.print(mpu6050.getAngleX());
      fs.print("\tangleY : ");fs.print(mpu6050.getAngleY());
      fs.print("\tangleZ : ");fs.println(mpu6050.getAngleZ());
      fs.println("=======================================================\n");
      fs.close();
    }
  }
}

void print_acc() {
  Serial.println("=======================================================");
  Serial.print("temp : ");Serial.println(mpu6050.getTemp ());
  Serial.print("accX : ");Serial.print(mpu6050.getAccX());
  Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
  Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());

  Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
  Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
  Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());

  Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
  Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());

  Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
  Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
  Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
  
  Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
  Serial.println("=======================================================\n");
}