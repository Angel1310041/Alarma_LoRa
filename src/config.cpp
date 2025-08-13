#include <Arduino.h>
#include "config.h"
#include "eeprom.h"

extern LoRaConfig configLora;
Preferences preferences;

// Frecuencias en el borde inferior de la banda ISM 433 MHz (433.05 MHz a 433.25 MHz)
float canales[9] = {
  433.05, 433.07, 433.09, 433.11, 433.13, 433.15, 433.17, 433.19, 433.21
};