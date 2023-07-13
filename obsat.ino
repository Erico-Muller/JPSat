#include <obsat.h>

Sat sat;

void setup() {
   Serial.begin(9600);
   sat.init();
}

void loop() {
   delay(500);
   
   sat.read_temperature();
   sat.read_humidity();
   sat.read_pressure();
   sat.read_co2();
   sat.read_light();
   sat.read_acc(false);
   sat.read_gps(false);
   sat.read_term_cam(false);

   Serial.println();
}
