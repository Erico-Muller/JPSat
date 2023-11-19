#include "Arduino.h"
// External Libs:
// https://github.com/queuetue/Q2-HX711-Arduino-Library
// https://github.com/solvek/CO2Sensor/

#ifndef JPSat_h
#define JPSat_h

// Importação de bibliotecas
#include <Arduino.h>
#include <Adafruit_AMG88xx.h>
#include <Arduino_JSON.h>
#include <Arduino_LED_Matrix.h>
#include <BH1750.h>
#include <CO2Sensor.h>
#include <DHT.h>
#include <ESP_SSLClient.h>
#include <IPAddress.h>
#include <MPU6050_tockn.h>
#include <Q2HX711.h>
#include <ResponsiveAnalogRead.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <WiFiS3.h>
#include <Wire.h>

#include "arduino_secrets.h"

// Definição do pino do SD
#define SD_PIN 4

// Definição dos pinos do DHT22
#define DHT_PIN 7
#define DHT_TYPE DHT22

// Definição de pinos do HX711
#define MPS_OUT 6
#define MPS_SCK 5

// Definição de pinos para cálculo para bateria 
#define pino1S A3
#define pino2S A1

uint8_t frame[8][12] = {
  { 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 }, 
  { 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0 },
  { 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0 },
  { 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0 }
};

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
char method[] = METHOD;
char host[] = HOST;
char path[] = PATH;
char team_number[] = TEAM_NUMBER;

long interval = INTERVAL * 500 * 30;
int multiplier = 1;

// Definição de class JPSat
class JPSat {
  private:
    bool is_there_sd = false;
   
    // Informação base da bateria
    const float voltRef = 4.89;
   
    // Dados dos resistores para cálculo da bateria
    const float res1 = 33300;
    const float res2 = 32500;

    // Células 1 e 2 da bateria
    float volt1S = 0.0;
    float volt2S = 0.0;

    int status = WL_IDLE_STATUS;

    // Strutura para agrupamento de módulos
    struct {
      File fs;
      DHT temp_dht{DHT_PIN, DHT_TYPE};
      Q2HX711 pressure_hx711{MPS_OUT, MPS_SCK};
      CO2Sensor co2_mg811{A0, 0.99, 100};
      BH1750 light_bh1750;
      TinyGPSPlus gps;
      SoftwareSerial ss{3, 2};
      MPU6050 acc_mpu6050{Wire};
      ESP_SSLClient client;
      ArduinoLEDMatrix matrix;
      Adafruit_AMG88xx term_cam;
    } modules;

    //Strutura para agrupamento de dados coletados
    struct {
      float humidity;
      float temperature;
      float pressure;
      float co2;
      float light;
      float battery;
      float coord[2];  // lng, lat
      float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
    } payload;

    // Arquivos que serão escritos no SD
    enum Modules_files {
      All,
      Humidity,
      Temperature,
      Pressure,
      Co2,
      Light,
      Coord,
      Acc,
      Term_Cam,
    };

  public:
    // Iniciar sensores
    void init() {
      Wire.begin();
      init_ss(9600);
      init_http();
      init_dht();
      init_mg811();
      init_mpu6050();
      init_bh1750(false);
      init_sd(SD_PIN);
      init_term_cam();
    }

    // Escrever dados no SD
    void write_data(Modules_files file) {
      if (is_there_sd) {
        switch (file) {
          case All:
            modules.fs = SD.open(F("all.txt"), FILE_WRITE);
            modules.fs.println(payload_to_json());
            modules.fs.close();
            delay(10);
            break;
          case Humidity:
            modules.fs = SD.open(F("humidity.txt"), FILE_WRITE);
            modules.fs.println(payload.humidity);  // %
            modules.fs.close();
            delay(10);
            break;

          case Temperature:
            modules.fs = SD.open(F("temperature.txt"), FILE_WRITE);
            modules.fs.println(payload.temperature);  // °C
            modules.fs.close();
            delay(10);
            break;
                        
          case Pressure:
            modules.fs = SD.open(F("pressure.txt"), FILE_WRITE);
            modules.fs.println(payload.pressure);
            modules.fs.close();
            delay(10);
            break;

          case Co2:
            modules.fs = SD.open(F("co2.txt"), FILE_WRITE);
            modules.fs.println(payload.co2);  // ppm
            modules.fs.close();
            delay(10);
            break;

          case Light:
            modules.fs = SD.open(F("light.txt"), FILE_WRITE);
            modules.fs.println(payload.light);  // lux
            modules.fs.close();
            delay(10);
            break;

          // [lng], [lat]
          case Coord:
            modules.fs = SD.open(F("coord.txt"), FILE_WRITE);
            modules.fs.print(payload.coord[0]);
            modules.fs.print(F(","));
            modules.fs.println(payload.coord[1]);
            modules.fs.close();
            delay(10);
            break;

          case Acc:
            modules.fs = SD.open(F("accelerometer.txt"), FILE_WRITE);
            modules.fs.print(F("temp : "));
            modules.fs.println(modules.acc_mpu6050.getTemp());
            modules.fs.print(F("accX : "));
            modules.fs.print(modules.acc_mpu6050.getAccX());
            modules.fs.print(F("\taccY : "));
            modules.fs.print(modules.acc_mpu6050.getAccY());
            modules.fs.print(F("\taccZ : "));
            modules.fs.println(modules.acc_mpu6050.getAccZ());

            modules.fs.print(F("gyroX : "));
            modules.fs.print(modules.acc_mpu6050.getGyroX());
            modules.fs.print(F("\tgyroY : "));
            modules.fs.print(modules.acc_mpu6050.getGyroY());
            modules.fs.print(F("\tgyroZ : "));
            modules.fs.println(modules.acc_mpu6050.getGyroZ());

            modules.fs.print(F("accAngleX : "));
            modules.fs.print(modules.acc_mpu6050.getAccAngleX());
            modules.fs.print(F("\taccAngleY : "));
            modules.fs.println(modules.acc_mpu6050.getAccAngleY());

            modules.fs.print(F("gyroAngleX : "));
            modules.fs.print(modules.acc_mpu6050.getGyroAngleX());
            modules.fs.print(F("\tgyroAngleY : "));
            modules.fs.print(modules.acc_mpu6050.getGyroAngleY());
            modules.fs.print(F("\tgyroAngleZ : "));
            modules.fs.println(modules.acc_mpu6050.getGyroAngleZ());

            modules.fs.print(F("angleX : "));
            modules.fs.print(modules.acc_mpu6050.getAngleX());
            modules.fs.print(F("\tangleY : "));
            modules.fs.print(modules.acc_mpu6050.getAngleY());
            modules.fs.print(F("\tangleZ : "));
            modules.fs.println(modules.acc_mpu6050.getAngleZ());
            modules.fs.close();
            delay(10);
            break;

          case Term_Cam:
            modules.fs = SD.open(F("term_cam.txt"), FILE_WRITE);
            modules.fs.print("[");
            for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
              modules.fs.print(payload.pixels[i-1]);
              modules.fs.print(", ");
              if( i%8 == 0 ) modules.fs.println();
            }
            modules.fs.println("]");
            modules.fs.close();
            delay(10);
            break;
        };
      }
    }

    // Enviar dados
    void send_data() {
      if (millis() > interval*multiplier) {
        multiplier++;

        String json = payload_to_json();

        Serial.println(json);

        Serial.println("\nStarting connection to server...");

        if (modules.client.connect(host, 443)) {
          Serial.println("connected to server");

          Serial.println(String(method) + " " + String(path) + " HTTP/1.1");
          Serial.println("Host: " + String(host));
          Serial.println("User-Agent: Arduino/1.0");
          Serial.println("Connection: close");
          Serial.println("Content-Type: application/json");
          Serial.println("Content-Length: " + String(json.length()));
          Serial.println("");
          Serial.println(json);

          modules.client.println(String(method) + " " + String(path) + " HTTP/1.1");
          modules.client.println("Host: " + String(host));
          modules.client.println("User-Agent: Arduino/1.0");
          modules.client.println("Connection: close");
          modules.client.println("Content-Type: application/json");
          modules.client.println("Content-Length: " + json.length());
          modules.client.println("");
          modules.client.println(json);
        } else {
          Serial.println("REQUEST FAILED!");
        }

        read_response();

        if (!modules.client.connected()) {
          Serial.println();
          Serial.println("disconnecting from server.");
          modules.client.stop();
        }
      }
    }

    // Ler temperatura
    float read_temperature(bool print = true) {
      payload.temperature = modules.temp_dht.readTemperature();

      // Verificar se os dados foram lidos
      if (isnan(payload.temperature)) {
        Serial.println(F("error: DHT22!"));
        return payload.temperature || 0.0;
      }

      // Print opcional
      if (print) {
        Serial.print(F("temperature: "));
        Serial.print(payload.temperature);
        Serial.println(F(" °C"));
      }

      // Escrever dados
      write_data(Temperature);

      return payload.temperature;
    }

    float read_humidity(bool print = true) {
      payload.humidity = modules.temp_dht.readHumidity();

      // Verificar se os dados foram lidos
      if (isnan(payload.humidity)) {
        Serial.println(F("error: DHT22!"));
        return payload.humidity || 0.0;
      }

      // Print opcional
      if (print) {
        Serial.print(F("humidity: "));
        Serial.print(payload.humidity);
        Serial.println(F("%")); 
      }

      // Escrever dados
      write_data(Humidity);

      return payload.humidity;
    }

    float read_pressure(bool print = true) {
      payload.pressure = modules.pressure_hx711.read() / 100.0;

      // Verificar se os dados foram lidos
      if (isnan(payload.pressure)) {
        Serial.println("error: HX711!");
        return payload.pressure || 0.0;
      }

      // Print opcional
      if (print) {
        Serial.print("pressure: ");
        Serial.println(payload.pressure);
      }

      // Escrever dados
      write_data(Pressure);

      return payload.pressure;
    }

    float read_co2(bool print = true) {
      payload.co2 = modules.co2_mg811.read();

      // Verificar se os dados foram lidos
      if (isnan(payload.co2)) {
        Serial.println(F("error: MG811!"));
        return payload.co2 || 0.0;
      }

      // Print opcional
      if (print) {
        Serial.print(F("co2: "));
        Serial.print(payload.co2);
        Serial.println(F(" ppm"));
      }

      // Escrever dados
      write_data(Co2);

      return payload.co2;
    }

    float read_light(bool print = true) {
      payload.light = modules.light_bh1750.readLightLevel();

      // Verificar se os dados foram lidos
      if (isnan(payload.light)) {
        Serial.println(F("error: BH1750!"));
        return payload.light || 0.0;
      }

      // Print opcional
      if (print) {
        Serial.print(F("light: "));
        Serial.print(payload.light);
        Serial.println(F(" lux"));
      }

      // Escrever dados
      write_data(Light);

      return payload.light;
    }

    void read_gps(bool print = true) {
      // Verificar se a comunicação serial está disponível
      if (modules.ss.available() > 0) {
        // Traduzir dados da comunicação em uma estrutura organizada
        if (modules.gps.encode(modules.ss.read())) {
          // Verificar se a localização é válida
          if (modules.gps.location.isValid()) {
            // Coleta de dados
            payload.coord[0] = modules.gps.location.lng();
            payload.coord[1] = modules.gps.location.lat();

            // Print opcional
            if (print) {
              Serial.print(payload.coord[0], 6);
              Serial.print(F(","));
              Serial.println(payload.coord[1], 6);
            }

            // Escrever dados
            write_data(Coord);
          } else {
            Serial.println(F("error: GPS!"));
          }
        }
      }
    }

    // Leitura dos dados do acelerômetro e giroscópio
    void read_acc(bool print = true) {
      modules.acc_mpu6050.update();

      // Print opcional
      if (print) {
        Serial.print(F("temp : "));
        Serial.println(modules.acc_mpu6050.getTemp());
        Serial.print(F("accX : "));
        Serial.print(modules.acc_mpu6050.getAccX());
        Serial.print(F("\taccY : "));
        Serial.print(modules.acc_mpu6050.getAccY());
        Serial.print(F("\taccZ : "));
        Serial.println(modules.acc_mpu6050.getAccZ());

        Serial.print(F("gyroX : "));
        Serial.print(modules.acc_mpu6050.getGyroX());
        Serial.print(F("\tgyroY : "));
        Serial.print(modules.acc_mpu6050.getGyroY());
        Serial.print(F("\tgyroZ : "));
        Serial.println(modules.acc_mpu6050.getGyroZ());

        Serial.print(F("accAngleX : "));
        Serial.print(modules.acc_mpu6050.getAccAngleX());
        Serial.print(F("\taccAngleY : "));
        Serial.println(modules.acc_mpu6050.getAccAngleY());

        Serial.print(F("gyroAngleX : "));
        Serial.print(modules.acc_mpu6050.getGyroAngleX());
        Serial.print(F("\tgyroAngleY : "));
        Serial.print(modules.acc_mpu6050.getGyroAngleY());
        Serial.print(F("\tgyroAngleZ : "));
        Serial.println(modules.acc_mpu6050.getGyroAngleZ());

        Serial.print(F("angleX : "));
        Serial.print(modules.acc_mpu6050.getAngleX());
        Serial.print(F("\tangleY : "));
        Serial.print(modules.acc_mpu6050.getAngleY());
        Serial.print(F("\tangleZ : "));
        Serial.println(modules.acc_mpu6050.getAngleZ());
      }

      // Escrever dados
      write_data(Acc);
    }

    void read_battery(bool print = true) {
      ResponsiveAnalogRead respVolt1S(pino1S, true);
      ResponsiveAnalogRead respVolt2S(pino2S, true);
      
      respVolt1S.update();
      delay(20);
      respVolt2S.update();

      volt1S = voltRef * respVolt1S.getValue() / 1023;
      volt2S = (voltRef * respVolt2S.getValue() / 1023);
      volt2S = (volt2S * (res1 + res2) / res2 ) - volt1S;
      payload.battery = (volt1S + volt2S) * 100 / voltRef;

      if (print) {
        Serial.print("1S: ");
        Serial.print(volt1S);
        Serial.print("\t");
        Serial.print("2S: ");
        Serial.print(volt2S);
        Serial.print("\t");
        Serial.print("Batt: ");
        Serial.println(payload.battery);
      }
    }

    void read_term_cam(bool print = true) {
      modules.term_cam.readPixels(payload.pixels);

      if (print) {
        Serial.print("[");
        for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
          Serial.print(payload.pixels[i-1]);
          Serial.print(", ");
          if( i%8 == 0 ) Serial.println();
        }
        Serial.println("]");
      }
      
      write_data(Term_Cam);
    }

    // Iniciar comunicação serial do GPS
    void init_ss(long speed) { modules.ss.begin(speed); }

    // Iniciar módulo de rede
    void init_http() { 
      if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        while (true);
      }  
      
      while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);         
        status = WiFi.begin(ssid, pass);
      }
      
      printWifiStatus();

      modules.client.setInsecure();
      modules.client.setBufferSizes(1024 /* rx */, 512 /* tx */);
      modules.client.setDebugLevel(1);

      modules.client.setClient(&modules.client);
    }

    void init_term_cam() {
      bool status = modules.term_cam.begin();
      if (!status) {
          Serial.println("Term cam not found!");
      }
    }

    // Iniciar DHT22
    void init_dht() { modules.temp_dht.begin(); }

    // Iniciar MG811
    void init_mg811() { modules.co2_mg811.calibrate(); }

    // Iniciar e verificar módulo SD
    void init_sd(short pin) {
      if (!SD.begin(pin)) {
        Serial.println(F("initialization failed!"));
      } else {
        is_there_sd = true;
      }
    }

    // Iniciar MPU6050
    void init_mpu6050() {
      modules.acc_mpu6050.begin();
      modules.acc_mpu6050.calcGyroOffsets(true);
    }

    // Iniciar BH1750
    void init_bh1750(bool init_wire = true) {
      Wire.begin();
      modules.light_bh1750.begin();
    }

    void init_matrix() {
      modules.matrix.begin();
    }

    String payload_to_json() {
      JSONVar obj;

      obj["equipe"] = team_number;
      obj["bateria"] = payload.battery;
      obj["temperatura"] = (int)payload.temperature;
      obj["pressao"] = payload.pressure;
    
      obj["giroscopio"][0] = modules.acc_mpu6050.getGyroX();
      obj["giroscopio"][1] = modules.acc_mpu6050.getGyroY();
      obj["giroscopio"][2] = modules.acc_mpu6050.getGyroZ();
    
      obj["acelerometro"][0] = modules.acc_mpu6050.getAccX();
      obj["acelerometro"][1] = modules.acc_mpu6050.getAccY();
      obj["acelerometro"][2] = modules.acc_mpu6050.getAccZ();

      obj["payload"]["co2"] = payload.co2;
      obj["payload"]["coord"][0] = payload.coord[0];
      obj["payload"]["coord"][1] = payload.coord[1];
      obj["payload"]["light"] = payload.light;
      obj["payload"]["humidity"] = payload.humidity;

      String json = JSON.stringify(obj);

      return json;
    }

    void printWifiStatus() {
      Serial.print("SSID: ");
      Serial.println(WiFi.SSID());

      IPAddress ip = WiFi.localIP();
      Serial.print("IP Address: ");
      Serial.println(ip);
    }

    void read_response() {  
      uint32_t received_data_num = 0;

      while (modules.client.available()) {
        char c = modules.client.read();

        Serial.print(c);
        
        received_data_num++;
        if(received_data_num % 80 == 0) { 
          Serial.println();
        }
      }  
    }

    void display_title() {
      modules.matrix.renderBitmap(frame, 8, 12);
    }
};

#endif
