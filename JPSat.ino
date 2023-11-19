// Importação de biblioteca
#include "JPSat.h"

// Instanciação
JPSat sat;

void setup() {
  Serial.begin(115200);
  delay(1000);
   
  // Iniciar modulos
  sat.init_matrix();
  sat.init_http();
}

void loop() {
  // Leituras
  sat.read_humidity();
  sat.read_temperature();
  sat.read_pressure();
  sat.read_light();
  sat.read_co2();
  sat.read_battery();
  sat.read_acc();
  sat.read_gps();
  sat.read_term_cam(false);

  //Envio de dados
  sat.send_data();

  Serial.println();
  sat.display_title();
  delay(500);
}
