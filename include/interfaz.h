#ifndef INTERFAZ_H
  #define INTERFAZ_H

  #include "config.h"
  #include "hardware.h"
  #include "eeprom.h"
  #include "comunicacion.h"

  class Interfaz {
    public:
      static void entrarModoProgramacion();
      static void salirModoProgramacion();
  };

  void endpointsMProg(void *pvParameters);

//Se añadio esto
  void modoprogporbotonfisico();
  void servidorModoProgramacion();
  void mostrarMensaje(const String& titulo, const String& mensaje, int delayMs);
  void configurarDisplay(bool estado);
  String getEncryptionType(wifi_auth_mode_t encryptionType);
#endif