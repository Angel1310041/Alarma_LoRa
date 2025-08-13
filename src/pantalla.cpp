#include "pantalla.h"
#include "OLEDDisplayFonts.h"
#include "config.h"
#include "eeprom.h"

#define VERSION_FIRMWARE "3.1.1.1"

// displayActivo ahora se inicializa leyendo el valor guardado en configLora.Pantalla
bool displayActivo = true;

void inicializarPantalla() {
  displayActivo = configLora.Pantalla;
  configLora.Pantalla = displayActivo; // Sincroniza ambos

  Heltec.display->init();
  Heltec.display->setFont(ArialMT_Plain_10); 
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->clear();

  // Apaga o enciende el display según el valor guardado
  if (displayActivo) {
    Heltec.display->displayOn();
    // Mostrar el estado LoRa al iniciar
    mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), VERSION_FIRMWARE);
  } else {
    Heltec.display->displayOff();
  }
}

void limpiarPantalla() {
  if (displayActivo) {
    Heltec.display->clear();
    Heltec.display->display();
  }
}

void mostrarMensaje(const String& titulo, const String& mensaje, int delayMs) {
  if (displayActivo) {
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, titulo);
    Heltec.display->drawString(0, 20, mensaje);
    Heltec.display->display();
    if (delayMs > 0) {
      delay(delayMs);
    }
  }
}

void mostrarEstadoLoRa(const String& idNodo, const String& canal, const String& version) {
  if (displayActivo) {
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "ID: " + idNodo + "  Canal: " + canal);
    Heltec.display->drawString(0, 15, "Ver: " + version);
    if (configLora.UART) {
      Heltec.display->drawString(0, 30, "Vecinal: SV12345670 P:01");
    }
    Heltec.display->display();
  }
}

void mostrarMensajeRecibido(const String& origen, const String& mensaje) {
  if (displayActivo) {
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "MSG Recibido!");
    Heltec.display->drawString(0, 15, "De: " + origen);
    String msgDisplay = mensaje;
    if (msgDisplay.length() > 30) {
      msgDisplay = msgDisplay.substring(0, 27) + "...";
    }
    Heltec.display->drawString(0, 30, "Msg: " + msgDisplay);
    Heltec.display->display();
    Serial.println("[DISPLAY] MSG Recibido!");
    Serial.println("[DISPLAY] De: " + origen);
    Serial.println("[DISPLAY] Msg: " + mensaje);
    vTaskDelay(10000 / portTICK_PERIOD_MS); 
    mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), VERSION_FIRMWARE);
  }
}

void mostrarMensajeEnviado(const String& destino, const String& mensaje) {
  if (displayActivo) {
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "MSG Enviado!");
    Heltec.display->drawString(0, 15, "A: " + destino);
    String msgDisplay = mensaje;
    if (msgDisplay.length() > 30) {
      msgDisplay = msgDisplay.substring(0, 27) + "...";
    }
    Heltec.display->drawString(0, 30, "Msg: " + msgDisplay);
    Heltec.display->display();
    Serial.println("[DISPLAY] MSG Enviado!");
    Serial.println("[DISPLAY] A: " + destino);
    Serial.println("[DISPLAY] Msg: " + mensaje);
    vTaskDelay(200 / portTICK_PERIOD_MS); // Mostrar por 5 segundos
    mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), VERSION_FIRMWARE);
  }
}

void mostrarError(const String& mensajeError) {
  if (displayActivo) {
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "ERROR!");
    Heltec.display->drawString(0, 20, mensajeError);
    Heltec.display->display();
    delay(3000);
  }
}

void mostrarInfo(const String& mensajeInfo) {
  if (displayActivo) {
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "INFO:");
    Heltec.display->drawString(0, 20, mensajeInfo);
    Heltec.display->display();
    delay(200);
  }
}

void configurarDisplay(bool habilitar) {
  displayActivo = habilitar;
  configLora.Pantalla = habilitar; // Guarda el estado en la variable Pantalla
  ManejoEEPROM::guardarTarjetaConfigEEPROM(); // Guarda en EEPROM

  if (habilitar) {
    Heltec.display->displayOn();
    // Solo mostrar el estado LoRa, NO el mensaje de info
    mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), VERSION_FIRMWARE);
  } else {
    Heltec.display->displayOff();
    Serial.println("Display DESHABILITADO.");
  }
}