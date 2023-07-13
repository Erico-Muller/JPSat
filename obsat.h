// External Libs:
// https://github.com/queuetue/Q2-HX711-Arduino-Library
// https://github.com/solvek/CO2Sensor/

#ifndef obsat_h
#define obsat_h

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

#define SD_PIN 4

#define DHT_PIN 7
#define DHT_TYPE DHT22

#define MPS_OUT 10
#define MPS_SCK 5

class Sat {
  private:
   bool is_there_sd = false;

   struct {
      File fs;
      DHT temp_dht{DHT_PIN, DHT_TYPE};
      Q2HX711 pressure_hx711{MPS_OUT, MPS_SCK};
      CO2Sensor co2_mg811{A0, 0.99, 100};
      BH1750 light_bh1750;
      TinyGPSPlus gps;
      SoftwareSerial ss{3, 2};
      Adafruit_AMG88xx amg;
      MPU6050 acc_mpu6050{Wire};
   } modules;

   struct {
      float humidity;
      float temperature;
      float pressure;
      float co2;
      float light;
      float coord[2];  // lng, lat
      float term_cam_pixels[AMG88xx_PIXEL_ARRAY_SIZE];
   } payload;

   enum Modules_files {
      Humidity,
      Temperature,
      Pressure,
      Co2,
      Light,
      Coord,
      Cam,
      Acc
   };

  public:
   void init() {
      Wire.begin();
      init_ss(9600);
      init_dht();
      init_mg811();
      init_sd(SD_PIN);
      init_cam();
      init_mpu6050();
      init_bh1750();
   }

   void write_data(Modules_files file) {
      if (is_there_sd) {
         switch (file) {
            case Humidity:
               modules.fs = SD.open("humidity.txt", FILE_WRITE);
               modules.fs.println(payload.humidity);  // %
               modules.fs.close();
               break;

            case Temperature:
               modules.fs = SD.open("temperature.txt", FILE_WRITE);
               modules.fs.println(payload.temperature);  // °C
               modules.fs.close();
               break;

            case Pressure:
               modules.fs = SD.open("pressure.txt", FILE_WRITE);
               modules.fs.println(payload.pressure);
               modules.fs.close();
               break;

            case Co2:
               modules.fs = SD.open("co2.txt", FILE_WRITE);
               modules.fs.println(payload.co2);  // ppm
               modules.fs.close();
               break;

            case Light:
               modules.fs = SD.open("light.txt", FILE_WRITE);
               modules.fs.println(payload.light);  // lux
               modules.fs.close();
               break;

            // [lng], [lat]
            case Coord:
               modules.fs = SD.open("coord.txt", FILE_WRITE);
               modules.fs.print(payload.coord[0]);
               modules.fs.print(",");
               modules.fs.println(payload.coord[1]);
               modules.fs.close();
               break;

            case Cam:
               modules.fs = SD.open("term_camera.txt", FILE_WRITE);
               for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
                  modules.fs.print(payload.term_cam_pixels[i]);
                  modules.fs.print("\t");
               }
               modules.fs.println();
               modules.fs.close();
               break;

            case Acc:
               modules.fs = SD.open("accelerometer.txt", FILE_WRITE);
               modules.fs.println(
                   "====================================================");
               modules.fs.print("temp : ");
               modules.fs.println(modules.acc_mpu6050.getTemp());
               modules.fs.print("accX : ");
               modules.fs.print(modules.acc_mpu6050.getAccX());
               modules.fs.print("\taccY : ");
               modules.fs.print(modules.acc_mpu6050.getAccY());
               modules.fs.print("\taccZ : ");
               modules.fs.println(modules.acc_mpu6050.getAccZ());

               modules.fs.print("gyroX : ");
               modules.fs.print(modules.acc_mpu6050.getGyroX());
               modules.fs.print("\tgyroY : ");
               modules.fs.print(modules.acc_mpu6050.getGyroY());
               modules.fs.print("\tgyroZ : ");
               modules.fs.println(modules.acc_mpu6050.getGyroZ());

               modules.fs.print("accAngleX : ");
               modules.fs.print(modules.acc_mpu6050.getAccAngleX());
               modules.fs.print("\taccAngleY : ");
               modules.fs.println(modules.acc_mpu6050.getAccAngleY());

               modules.fs.print("gyroAngleX : ");
               modules.fs.print(modules.acc_mpu6050.getGyroAngleX());
               modules.fs.print("\tgyroAngleY : ");
               modules.fs.print(modules.acc_mpu6050.getGyroAngleY());
               modules.fs.print("\tgyroAngleZ : ");
               modules.fs.println(modules.acc_mpu6050.getGyroAngleZ());

               modules.fs.print("angleX : ");
               modules.fs.print(modules.acc_mpu6050.getAngleX());
               modules.fs.print("\tangleY : ");
               modules.fs.print(modules.acc_mpu6050.getAngleY());
               modules.fs.print("\tangleZ : ");
               modules.fs.println(modules.acc_mpu6050.getAngleZ());
               modules.fs.println(
                   "====================================================\n");
               modules.fs.close();
               break;
         };
      }
   }

   void delete_all_data() {}

   float read_temperature(bool print = true) {
      payload.temperature = modules.temp_dht.readTemperature();

      if (isnan(payload.temperature)) {
         Serial.println("error: DHT22!");
         return;
      }

      if (print) {
         Serial.println("temperature: ");
         Serial.print(payload.temperature);
         Serial.println(" °C");
      }

      write_data(Temperature);
   }

   float read_humidity(bool print = true) {
      payload.humidity = modules.temp_dht.readHumidity();

      if (isnan(payload.humidity)) {
         Serial.println("error: DHT22!");
         return;
      }

      if (print) {
         Serial.print("humidity: ");
         Serial.print(payload.humidity);
         Serial.println("%");
      }

      write_data(Humidity);
   }

   float read_pressure(bool print = true) {
      payload.pressure = modules.pressure_hx711.read() / 100.0;

      if (isnan(payload.pressure)) {
         Serial.println("error: HX711!");
         return;
      }

      if (print) {
         Serial.print("pressure: ");
         Serial.println(payload.pressure);
      }

      write_data(Pressure);
   }

   float read_co2(bool print = true) {
      payload.co2 = modules.co2_mg811.read();

      if (isnan(payload.co2)) {
         Serial.println("error: MG811!");
         return;
      }

      if (print) {
         Serial.print("co2: ");
         Serial.print(payload.co2);
         Serial.println(" ppm");
      }

      write_data(Co2);
   }

   float read_light(bool print = true) {
      payload.light = modules.light_bh1750.readLightLevel();

      if (isnan(payload.light)) {
         Serial.println("error: BH1750!");
         return;
      }

      if (print) {
         Serial.print("light: ");
         Serial.print(payload.light);
         Serial.println(" lux");
      }

      write_data(Light);
   }

   float read_gps(bool print = true) {
      if (modules.ss.available() > 0) {
         if (modules.gps.encode(modules.ss.read())) {
            if (modules.gps.location.isValid()) {
               payload.coord[0] = modules.gps.location.lng();
               payload.coord[1] = modules.gps.location.lat();

               if (print) {
                  Serial.print(payload.coord[0]);
                  Serial.print(",");
                  Serial.println(payload.coord[1]);
               }

               write_data(Coord);
            } else {
               Serial.println("error: GPS!");
            }
         }
      }
   }

   float read_term_cam(bool print = true) {
      modules.amg.readPixels(payload.term_cam_pixels);

      if (print) {
         for (int i = 0; i < AMG88xx_PIXEL_ARRAY_SIZE; i++) {
            Serial.print(payload.term_cam_pixels[i]);
            Serial.print("\t");
         }
         Serial.println();
      }

      write_data(Cam);
   }

   float read_acc(bool print = true) {
      modules.acc_mpu6050.update();

      if (print) {
         Serial.println("====================================================");
         Serial.print("temp : ");
         Serial.println(modules.acc_mpu6050.getTemp());
         Serial.print("accX : ");
         Serial.print(modules.acc_mpu6050.getAccX());
         Serial.print("\taccY : ");
         Serial.print(modules.acc_mpu6050.getAccY());
         Serial.print("\taccZ : ");
         Serial.println(modules.acc_mpu6050.getAccZ());

         Serial.print("gyroX : ");
         Serial.print(modules.acc_mpu6050.getGyroX());
         Serial.print("\tgyroY : ");
         Serial.print(modules.acc_mpu6050.getGyroY());
         Serial.print("\tgyroZ : ");
         Serial.println(modules.acc_mpu6050.getGyroZ());

         Serial.print("accAngleX : ");
         Serial.print(modules.acc_mpu6050.getAccAngleX());
         Serial.print("\taccAngleY : ");
         Serial.println(modules.acc_mpu6050.getAccAngleY());

         Serial.print("gyroAngleX : ");
         Serial.print(modules.acc_mpu6050.getGyroAngleX());
         Serial.print("\tgyroAngleY : ");
         Serial.print(modules.acc_mpu6050.getGyroAngleY());
         Serial.print("\tgyroAngleZ : ");
         Serial.println(modules.acc_mpu6050.getGyroAngleZ());

         Serial.print("angleX : ");
         Serial.print(modules.acc_mpu6050.getAngleX());
         Serial.print("\tangleY : ");
         Serial.print(modules.acc_mpu6050.getAngleY());
         Serial.print("\tangleZ : ");
         Serial.println(modules.acc_mpu6050.getAngleZ());
         Serial.println(
             "====================================================\n");
      }

      write_data(Acc);
   }

   void init_ss(long speed) { modules.ss.begin(speed); }

   void init_dht() { modules.temp_dht.begin(); }

   void init_mg811() { modules.co2_mg811.calibrate(); }

   void init_sd(short pin) {
      if (!SD.begin(pin)) {
         Serial.println("initialization failed!");
      } else {
         is_there_sd = false;
      }
   }

   void init_cam() {
      if (!modules.amg.begin()) {
         Serial.println("Could not find a valid AMG8833 sensor, check wiring!");
         while (1)
            ;
      }
   }

   void init_mpu6050() {
      modules.acc_mpu6050.begin();
      modules.acc_mpu6050.calcGyroOffsets(true);
   }

   void init_bh1750() { modules.light_bh1750.begin(); }
};

#endif
