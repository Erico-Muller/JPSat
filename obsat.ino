// External Libs:
// https://github.com/queuetue/Q2-HX711-Arduino-Library
// https://github.com/solvek/CO2Sensor/

#include <Adafruit_AMG88xx.h>
#include <BH1750.h>
#include <CO2Sensor.h>
#include <DHT.h>
#include <MPU6050_tockn.h>
#include <Q2HX711.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

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
   float coord[2];  // lng, lat
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
      is_there_sd = false;
   }

   if (!amg.begin()) {
      Serial.println("Could not find a valid AMG8833 sensor, check wiring!");
      while (1)
         ;
   }

   acc_mpu6050.begin();
   acc_mpu6050.calcGyroOffsets(true);
}

void loop() {
   delay(500);

   // read_temperature_and_humidity();
   // read_pressure();
   // read_co2();
   // read_light();
   // read_acc();
   // read_gps();
   // read_term_camera();

   Serial.println();
   // print_temperature_and_humidity();
   // print_pressure();
   // print_co2();
   // print_light();
   // print_acc();
   // print_gps();
   // print_term_camera();
}

void read_temperature_and_humidity() {
   payload.temperature = temp_dht.readTemperature();
   payload.humidity = temp_dht.readHumidity();

   if (isnan(payload.humidity) || isnan(payload.temperature)) {
      Serial.println("error: DHT22!");
      return;
   }

   if (is_there_sd) {
      fs = SD.open("temperature_and_humidity.txt", FILE_WRITE);

      // [temp] [humid]
      if (fs) {
         fs.print(payload.temperature);
         fs.print("\t");
         fs.println(payload.humidity);
         fs.close();
      }
   }
}

void print_temperature_and_humidity() {
   Serial.print("Humidity: ");
   Serial.print(payload.humidity);
   Serial.print("%\t");
   Serial.print("Temperature: ");
   Serial.print(payload.temperature);
   Serial.println("°C");
}

void read_pressure() {
   payload.pressure = pressure_hx711.read() / 100.0;

   if (isnan(payload.pressure)) {
      Serial.println("error: HX711!");
      return;
   }

   if (is_there_sd) {
      fs = SD.open("pressure.txt", FILE_WRITE);

      if (fs) {
         fs.println(payload.temperature);
         fs.close();
      }
   }
}

void print_pressure() {
   Serial.print("Pressure: ");
   Serial.println(payload.pressure);
}

void read_co2() {
   payload.co2 = co2_mg811.read();

   if (isnan(payload.co2)) {
      Serial.println("error: MG811!");
      return;
   }

   if (is_there_sd) {
      fs = SD.open("co2.txt", FILE_WRITE);

      if (fs) {
         fs.println(payload.co2);
         fs.close();
      }
   }
}

void print_co2() {
   Serial.print("CO2: ");
   Serial.print(payload.co2);
   Serial.println(" ppm");
}

void read_light() {
   payload.light = light_bh1750.readLightLevel();

   if (isnan(payload.light)) {
      Serial.println("error: BH1750!");
      return;
   }

   if (is_there_sd) {
      fs = SD.open("light.txt", FILE_WRITE);

      if (fs) {
         fs.println(payload.light);
         fs.close();
      }
   }
}

void print_light() {
   Serial.print("Light: ");
   Serial.print(payload.light);
   Serial.println(" lx");
}

void read_gps() {
   if (ss.available() > 0)
      if (gps.encode(ss.read())) {
         if (gps.location.isValid()) {
            payload.coord[0] = gps.location.lng();
            payload.coord[1] = gps.location.lat();

            if (is_there_sd) {
               fs = SD.open("gps.txt", FILE_WRITE);

               // [lng], [lat]
               if (fs) {
                  fs.print(payload.coord[0]);
                  fs.print(",");
                  fs.println(payload.coord[1]);
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
   Serial.print(payload.coord[0], 6);
   Serial.print(",");
   Serial.println(payload.coord[1], 6);
}

void read_term_camera() {
   amg.readPixels(payload.term_cam_pixels);

   if (is_there_sd) {
      fs = SD.open("term_camera.txt", FILE_WRITE);

      if (fs) {
         for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
            fs.print(payload.term_cam_pixels[i]);
            fs.print("\t");
         }
         fs.println();
         fs.close();
      }
   }
}

void print_term_camera() {
   for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
      Serial.print(payload.term_cam_pixels[i]);
      Serial.print("\t");
   }
   Serial.println();
}

void read_acc() {
   acc_mpu6050.update();

   if (is_there_sd) {
      fs = SD.open("mpu.txt", FILE_WRITE);

      if (fs) {
         fs.println("=======================================================");
         fs.print("temp : ");
         fs.println(acc_mpu6050.getTemp());
         fs.print("accX : ");
         fs.print(acc_mpu6050.getAccX());
         fs.print("\taccY : ");
         fs.print(acc_mpu6050.getAccY());
         fs.print("\taccZ : ");
         fs.println(acc_mpu6050.getAccZ());

         fs.print("gyroX : ");
         fs.print(acc_mpu6050.getGyroX());
         fs.print("\tgyroY : ");
         fs.print(acc_mpu6050.getGyroY());
         fs.print("\tgyroZ : ");
         fs.println(acc_mpu6050.getGyroZ());

         fs.print("accAngleX : ");
         fs.print(acc_mpu6050.getAccAngleX());
         fs.print("\taccAngleY : ");
         fs.println(acc_mpu6050.getAccAngleY());

         fs.print("gyroAngleX : ");
         fs.print(acc_mpu6050.getGyroAngleX());
         fs.print("\tgyroAngleY : ");
         fs.print(acc_mpu6050.getGyroAngleY());
         fs.print("\tgyroAngleZ : ");
         fs.println(acc_mpu6050.getGyroAngleZ());

         fs.print("angleX : ");
         fs.print(acc_mpu6050.getAngleX());
         fs.print("\tangleY : ");
         fs.print(acc_mpu6050.getAngleY());
         fs.print("\tangleZ : ");
         fs.println(acc_mpu6050.getAngleZ());
         fs.println(
             "=======================================================\n");
         fs.close();
      }
   }
}

void print_acc() {
   Serial.println("=======================================================");
   Serial.print("temp : ");
   Serial.println(acc_mpu6050.getTemp());
   Serial.print("accX : ");
   Serial.print(acc_mpu6050.getAccX());
   Serial.print("\taccY : ");
   Serial.print(acc_mpu6050.getAccY());
   Serial.print("\taccZ : ");
   Serial.println(acc_mpu6050.getAccZ());

   Serial.print("gyroX : ");
   Serial.print(acc_mpu6050.getGyroX());
   Serial.print("\tgyroY : ");
   Serial.print(acc_mpu6050.getGyroY());
   Serial.print("\tgyroZ : ");
   Serial.println(acc_mpu6050.getGyroZ());

   Serial.print("accAngleX : ");
   Serial.print(acc_mpu6050.getAccAngleX());
   Serial.print("\taccAngleY : ");
   Serial.println(acc_mpu6050.getAccAngleY());

   Serial.print("gyroAngleX : ");
   Serial.print(acc_mpu6050.getGyroAngleX());
   Serial.print("\tgyroAngleY : ");
   Serial.print(acc_mpu6050.getGyroAngleY());
   Serial.print("\tgyroAngleZ : ");
   Serial.println(acc_mpu6050.getGyroAngleZ());

   Serial.print("angleX : ");
   Serial.print(acc_mpu6050.getAngleX());
   Serial.print("\tangleY : ");
   Serial.print(acc_mpu6050.getAngleY());
   Serial.print("\tangleZ : ");
   Serial.println(acc_mpu6050.getAngleZ());
   Serial.println("=======================================================\n");
}