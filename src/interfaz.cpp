#include "interfaz.h"
#include "hardware.h"
#include "comunicacion.h"
#include "config.h"
#include "eeprom.h"
#include "pantalla.h"
#include <WebServer.h>
#include <FS.h>
#include <Update.h>
#include <SPIFFS.h>

WebServer server(80);

void Interfaz::entrarModoProgramacion() {
  modoProgramacion = true;
  esp_task_wdt_reset();
  digitalWrite(LED_STATUS, HIGH);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  xTaskCreatePinnedToCore(
    endpointsMProg,
    "Endpoints",
    8192,
    NULL,
    2,
    NULL,
    0
  );
  imprimirSerial("---# Modo Programacion Activado #---", 'g');
}

void endpointsMProg(void *pvParameters) {
  imprimirSerial("Modo Programacion Falso", 'b');
  servidorModoProgramacion();  // Inicia el servidor web
  vTaskDelete(NULL);
}

// Modifica el modo de programacion por boton fisico
void modoprogporbotonfisico() {
  static unsigned long tiempoInicio = 0;
  static bool botonAnterior = false;

  bool botonPresionado = digitalRead(BOTON_MODO_PROG) == LOW;

  if (!modoProgramacion) {
    if (botonPresionado) {
      if (!botonAnterior) {
        tiempoInicio = millis();
      } else {
        if (millis() - tiempoInicio >= 3000) {
          Interfaz::entrarModoProgramacion();
        }
      }
    } else if (botonAnterior) {
      tiempoInicio = 0;
    }
  }
  botonAnterior = botonPresionado;
}

void Interfaz::salirModoProgramacion() {
  modoProgramacion = false;
  digitalWrite(LED_STATUS, LOW);  // Apagar el LED 35
  server.stop();                  // Detener el servidor web
  imprimirSerial("---# Modo Programacion Desactivado #---", 'g');
  delay(1000);                    // Pequeña pausa antes de reiniciar
  ESP.restart();                  // Reiniciar la tarjeta
}

void servidorModoProgramacion() {
  if (!SPIFFS.begin(true)) {
    imprimirSerial("Error al montar SPIFFS", 'r');
    return;
  }

  // Sin WiFi ni SoftAP, solo servidor HTTP sencillo en la IP local (requiere acceso ethernet/serial para ver)
  imprimirSerial("Servidor web iniciado (solo acceso local, sin WiFi ni AP)", 'y');

  // Página HTML principal
  server.on("/", HTTP_GET, []() {
    if (SPIFFS.exists("/interfaz.html.gz")) {
      File file = SPIFFS.open("/interfaz.html.gz", FILE_READ);
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "Archivo no encontrado");
    }
  });

  server.on("/Interfaz", HTTP_GET, []() {
    if (SPIFFS.exists("/interfaz.html.gz")) {
      File file = SPIFFS.open("/interfaz.html.gz", FILE_READ);
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "Archivo no encontrado: interfaz.html.gz");
    }
  });

  server.on("/TEST", HTTP_GET, []() {
    if (SPIFFS.exists("/serial.html.gz")) {
      File file = SPIFFS.open("/serial.html.gz", FILE_READ);
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "Archivo no encontrado: serial.html.gz");
    }
  });

  // --- ENDPOINT PARA OTA ---
  server.on("/OTA", HTTP_GET, []() {
    if (SPIFFS.exists("/ota.html.gz")) {
      File file = SPIFFS.open("/ota.html.gz", FILE_READ);
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "Archivo no encontrado: ota.html.gz");
    }
  });

  // OTA POST
  server.on("/actualizar", HTTP_POST, []() {
    if (Update.hasError()) {
      server.send(500, "application/json", "{\"error\":\"Fallo en la actualización\"}");
    } else {
      server.send(200, "application/json", "{\"success\":true, \"message\":\"Actualización exitosa. Reiniciando...\"}");
      delay(1000);
      ESP.restart();
    }
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Iniciando actualización: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Actualización completada: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  // Configuración GET (sin WiFi ni respuesta de comandos)
  server.on("/api/config", HTTP_GET, []() {
    DynamicJsonDocument doc(128);
    doc["id"] = configLora.IDLora;
    doc["channel"] = configLora.Canal;
    doc["displayOn"] = configLora.Pantalla;
    doc["UART"] = configLora.UART;
    doc["I2C"] = configLora.I2C;
    doc["DEBUG"] = configLora.DEBUG;
    String respuesta;
    serializeJson(doc, respuesta);
    server.send(200, "application/json", respuesta);
  });

  // Configuración POST (sin WiFi)
  server.on("/api/config", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "Bad Request");
      return;
    }

    String body = server.arg("plain");
    DynamicJsonDocument doc(128);
    deserializeJson(doc, body);

    String newId = doc["id"].as<String>();
    int newChannel = doc["channel"];

    if (newId.length() > 0 && newId.length() <= 3 && newChannel >= 0 && newChannel <= 8) {
      strncpy(configLora.IDLora, newId.c_str(), sizeof(configLora.IDLora));
      configLora.Canal = newChannel;

      if (doc.containsKey("DEBUG")) configLora.DEBUG = doc["DEBUG"];

      ManejoEEPROM::guardarTarjetaConfigEEPROM();
      lora.begin(canales[configLora.Canal]);
      lora.startReceive();

      DynamicJsonDocument responseDoc(64);
      responseDoc["success"] = true;
      String response;
      serializeJson(responseDoc, response);
      server.send(200, "application/json", response);

      mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), "3.1.1.1");
    } else {
      server.send(400, "text/plain", "Parámetros inválidos");
    }
  });

  // Control de pantalla
  server.on("/api/display", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "Bad Request");
      return;
    }

    String body = server.arg("plain");
    DynamicJsonDocument doc(64);
    deserializeJson(doc, body);

    bool nuevoEstado = doc["state"] == "1";
    configurarDisplay(nuevoEstado); // Esta función ahora muestra el estado en serial

    DynamicJsonDocument responseDoc(32);
    responseDoc["success"] = true;
    responseDoc["display"] = nuevoEstado ? 1 : 0;
    String response;
    serializeJson(responseDoc, response);
    server.send(200, "application/json", response);
  });

  // Control UART
  server.on("/api/uart", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "Bad Request");
      return;
    }

    String body = server.arg("plain");
    DynamicJsonDocument doc(32);
    deserializeJson(doc, body);

    bool nuevoEstado = doc["state"] == "1";
    configLora.UART = nuevoEstado;
    ManejoEEPROM::guardarTarjetaConfigEEPROM();

    if (!nuevoEstado) {
      imprimirSerial("UART inhabilitado", 'y');
    }

    ManejoComunicacion::initUART();

    DynamicJsonDocument responseDoc(32);
    responseDoc["success"] = true;
    responseDoc["uart"] = configLora.UART;
    String response;
    serializeJson(responseDoc, response);
    server.send(200, "application/json", response);
  });

  // Control I2C
  server.on("/api/i2c", HTTP_POST, []() {
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "Bad Request");
      return;
    }

    String body = server.arg("plain");
    DynamicJsonDocument doc(32);
    deserializeJson(doc, body);

    bool enable = doc["state"] == "1";
    configLora.I2C = enable;
    ManejoEEPROM::guardarTarjetaConfigEEPROM();

    if (enable) {
      Wire.begin(SDA, SCL); // Cambia SDA/SCL si corresponden otros pines
      Wire.setClock(100000); 
      imprimirSerial("I2C activado", 'g');
    } else {
      Wire.end();
      imprimirSerial("I2C desactivado", 'y');
    }

    DynamicJsonDocument response(32);
    response["success"] = true;
    response["i2c_enabled"] = configLora.I2C;
    String jsonResponse;
    serializeJson(response, jsonResponse);
    server.send(200, "application/json", jsonResponse);
  });

  // Escaneo I2C simple
  server.on("/api/i2c/scan", HTTP_GET, []() {
    if (!configLora.I2C) {
      server.send(200, "application/json", "{\"error\":\"I2C desactivado\"}");
      return;
    }

    DynamicJsonDocument doc(256);
    JsonArray dispositivos = doc.createNestedArray("dispositivos");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
        JsonObject dispositivo = dispositivos.createNestedObject();
        dispositivo["direccion"] = "0x" + String(address, HEX);
        dispositivo["decimal"] = address;
        nDevices++;
      }
    }
    doc["dispositivos_encontrados"] = nDevices;
    doc["status"] = "Escaneo completado";
    String respuesta;
    serializeJson(doc, respuesta);
    server.send(200, "application/json", respuesta);
  });

  // Control de prueba de pantalla - Muestra SOLO ID y Versión
  server.on("/api/display/test", HTTP_POST, []() {
    if (!configLora.Pantalla) {
      configurarDisplay(true);
    }
    mostrarMensaje("ID: " + String(configLora.IDLora), 
                  "Ver: " + Version, 
                  5000);  // Mostrar por 5 segundos
    DynamicJsonDocument response(64);
    response["success"] = true;
    response["id"] = configLora.IDLora;
    response["version"] = Version;
    String jsonResponse;
    serializeJson(response, jsonResponse);
    server.send(200, "application/json", jsonResponse);
  });

  // Reinicio del dispositivo
  server.on("/api/reboot", HTTP_POST, []() {
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Reiniciando dispositivo...\"}");
    delay(500);
    Interfaz::salirModoProgramacion();
    ESP.restart();
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "Ruta no encontrada");
  });

  server.begin();

  while (modoProgramacion) {
    server.handleClient();
    delay(10);
  }
}