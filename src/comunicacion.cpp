#include <Arduino.h>
#include <RCSwitch.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "config.h"
#include "pantalla.h"
#include "heltec.h"
#include "RadioLib.h"
#include "comunicacion.h"
#include "eeprom.h"
#include "interfaz.h"

extern SX1262 lora;
LoRaConfig configLora;
String ultimoComandoRecibido = "";
TwoWire I2CGral = TwoWire(1);

volatile bool rfSignalReceived = false;
unsigned long lastRFSignalTime = 0;
static RCSwitch mySwitch = RCSwitch();

unsigned long io1TimerStart = 0;
unsigned long io2TimerStart = 0;
bool io1TimerActive = false;
bool io2TimerActive = false;
const unsigned long IO_TIMEOUT_MS = 5UL * 60UL * 1000UL; // 5 minutos

void habilitarPantalla() {
    configLora.Pantalla = true;
    configurarDisplay(true);
    imprimirSerial("Pantalla habilitada.", 'g');
    ManejoEEPROM::guardarTarjetaConfigEEPROM();
}

void deshabilitarPantalla() {
    configLora.Pantalla = false;
    configurarDisplay(false); 
    imprimirSerial("Pantalla deshabilitada.", 'y');
    ManejoEEPROM::guardarTarjetaConfigEEPROM();
}

void parpadearLEDStatus(int veces, int periodo_ms) {
    for (int i = 0; i < veces; ++i) {
        digitalWrite(LED_STATUS, HIGH);
        delay(periodo_ms / 2);
        digitalWrite(LED_STATUS, LOW);
        delay(periodo_ms / 2);
    }
}

void ManejoComunicacion::inicializar() {
    Serial.begin(9600);
    delay(1000);
    initUART();

    if (lora.begin() != RADIOLIB_ERR_NONE) {
        Serial.println("LoRa init failed!");
        while (true);
    }
}

void ManejoComunicacion::initUART() {
    if (configLora.UART) {
        Serial2.begin(9600, SERIAL_8N1, UART_RX, UART_TX); 
        imprimirSerial("UART inicializado correctamente.", 'g');
    } else {
        imprimirSerial("UART inhabilitado", 'y');
    }
}

String ManejoComunicacion::leerVecinal() {
    if (configLora.UART) {
        String comandoVecinal = "";
        if (Serial2.available()) {
            comandoVecinal = Serial2.readStringUntil('\n');
            if (comandoVecinal != "") {
                imprimirSerial("Comando recibido en Serial2: " + comandoVecinal, 'y');
                return comandoVecinal;
            } else {
                return "";
            }
        }
        return "";
    } else {
        imprimirSerial("Comunicacion UART deshabilitada", 'y');
        return "";
    }
}

void ManejoComunicacion::escribirVecinal(String envioVecinal) {
    if (configLora.UART) {
        Serial2.println(envioVecinal);
        imprimirSerial("Comando enviado a la Alarma Vecinal:\n   -> " + envioVecinal, 'g');
    }
}

String ManejoComunicacion::leerSerial() {
    String comandoVecinal = "";
    if (Serial.available()) {
        comandoVecinal = Serial.readStringUntil('\n');
        if (comandoVecinal != "") {
            imprimirSerial("Comando recibido en Serial: " + comandoVecinal, 'y');
            return comandoVecinal;
        }
    }
    return "";
}

void ManejoComunicacion::envioMsjLoRa(String comandoLoRa, const String& destino) {
    esp_task_wdt_reset(); // <-- Antes de transmitir
    String msgID = String(configLora.IDLora) + "-" + String(millis());
    char paquete[256];
    snprintf(paquete, sizeof(paquete), "ORIG:%s|DEST:%s|MSG:%s|HOP:%s|CANAL:%d|ID:%s",
        configLora.IDLora, destino.c_str(), comandoLoRa.c_str(), destino.c_str(), configLora.Canal, msgID.c_str());
    imprimirSerial("Enviando comando " + String(paquete) + " a lora con el ID " + destino);
    lora.standby();
    int resultado = lora.transmit(paquete);
    esp_task_wdt_reset(); // <-- Después de transmitir
    lora.startReceive();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void ManejoComunicacion::procesarComando(const String &comandoRecibido, const String &idOrigen) {
    esp_task_wdt_reset();

    if (comandoRecibido.isEmpty()) { 
        return;
    } else if (comandoRecibido == ultimoComandoRecibido) {
        return;
    }
    ultimoComandoRecibido = comandoRecibido;
    
    int primerSep = comandoRecibido.indexOf('@');
    int segundoSep = comandoRecibido.indexOf('@', primerSep + 1);
    int tercerSep = comandoRecibido.indexOf('@', segundoSep + 1);

    String idDestino = comandoRecibido.substring(0, primerSep);

    if (primerSep == -1 || segundoSep == -1 || tercerSep == -1 ||
        primerSep == 0 || segundoSep == primerSep + 1 || tercerSep == segundoSep + 1) {
        return;
    }

    char red = comandoRecibido.charAt(primerSep + 1);
    String comandoProcesar = comandoRecibido.substring(segundoSep + 1, tercerSep); 
    String valorAleatorio = comandoRecibido.substring(tercerSep + 1); 

    // ==== Bloquea procesamiento de INFOV si viene por LoRa ====
    if (red == 'L' && comandoProcesar.indexOf("INFOV") != -1) {
        imprimirSerial("Comando INFOV por LoRa ignorado para evitar watchdog.", 'y');
        return;
    }
    // ===========================================================

    String prefix1 = "";
    String accion = "";

    imprimirSerial("\nComando recibido en procesado de comando: " + comandoRecibido);
    imprimirSerial("ID de Destino del Comando -> " + idDestino);
    imprimirSerial("Red -> " + String(red));
    imprimirSerial("Comando a Procesar -> " + comandoProcesar);
    imprimirSerial("Valor Aleatorio -> " + valorAleatorio);

    if (strcmp(idDestino.c_str(), configLora.IDLora) == 0 || strcmp(idDestino.c_str(), "000") == 0) {
        imprimirSerial("Comando dirigido a este nodo (" + String(configLora.IDLora) + ") o es broadcast (000).", 'g');

        // --- Ejecuta acciones físicas/config/flags, sin generar respuesta ni eco ----

        if (red == 'L') { 
            if (comandoProcesar.startsWith("ID")) {
                accion = comandoProcesar.substring(2, 3);
                if (accion == "C") {
                    if (comandoProcesar.length() >= 7) { 
                        prefix1 = comandoProcesar.substring(4, 7); 
                        imprimirSerial("Cambiando el ID del nodo a -> " + prefix1, 'c');
                        strncpy(configLora.IDLora, prefix1.c_str(), sizeof(configLora.IDLora) - 1);
                        configLora.IDLora[sizeof(configLora.IDLora) - 1] = '\0';
                        ManejoEEPROM::guardarTarjetaConfigEEPROM();
                    }
                }
            } else if (comandoProcesar.startsWith("CH")) {
                accion = comandoProcesar.substring(2, 3);
                if (accion == "C") {
                    int idxMayor = comandoProcesar.indexOf('>');
                    if (idxMayor != -1 && idxMayor + 1 < comandoProcesar.length()) {
                        prefix1 = comandoProcesar.substring(idxMayor + 1);
                        imprimirSerial("Cambiando el canal del nodo a -> " + prefix1, 'c');
                        configLora.Canal = prefix1.toInt();
                        if (configLora.Canal >= 0 && configLora.Canal < (sizeof(canales) / sizeof(canales[0]))) { 
                            ManejoEEPROM::guardarTarjetaConfigEEPROM();
                            lora.begin(canales[configLora.Canal]); 
                            esp_restart(); 
                        }
                    }
                }
            } else if (comandoProcesar.startsWith("SCR")) {
                accion = comandoProcesar.substring(4, 5);
                if (accion == "0") {
                    imprimirSerial("Desactivando la pantalla");
                    configurarDisplay(false); 
                } else if (accion == "1") {
                    imprimirSerial("Activando la pantalla");
                    configurarDisplay(true);
                }
            } else if (comandoProcesar.startsWith("OUT>")) {
                int primerMayor = comandoProcesar.indexOf('>', 4);
                if (primerMayor != -1) {
                    String salidaStr = comandoProcesar.substring(4, primerMayor);
                    String accionStr = comandoProcesar.substring(primerMayor + 1);

                    int pin = -1;
                    bool isIO1 = false, isIO2 = false;

                    if (salidaStr == "1") { pin = PIN_IO1; isIO1 = true; }
                    else if (salidaStr == "2") { pin = PIN_IO2; isIO2 = true; }
                    else { /* salida no reconocida */ }

                    if (accionStr == "1") {
                        pinMode(pin, OUTPUT);
                        digitalWrite(pin, HIGH);
                        if (isIO1) {
                            io1TimerStart = millis();
                            io1TimerActive = true;
                        } else if (isIO2) {
                            io2TimerStart = millis();
                            io2TimerActive = true;
                        }
                    } else if (accionStr == "0") {
                        pinMode(pin, OUTPUT);
                        digitalWrite(pin, LOW);
                        if (isIO1) {
                            io1TimerActive = false;
                            io1TimerStart = 0;
                        } else if (isIO2) {
                            io2TimerActive = false;
                            io2TimerStart = 0;
                        }
                    }
                }
            }  else if (comandoProcesar.startsWith("FORMAT")) {
                imprimirSerial("Reiniciando la tarjeta LoRa...");
                delay(1000);
                ManejoEEPROM::borrarTarjetaConfigEEPROM(); 
                esp_restart();
            } else if(comandoProcesar.startsWith("RESET")){
                imprimirSerial("Reiniciando el sistema...");
                delay(1000);
                esp_restart();
            } else if (comandoProcesar.startsWith("MPROG")) {
                imprimirSerial("Entrando a modo programacion a traves de comando...");
                if (!modoProgramacion) { 
                    Interfaz::entrarModoProgramacion(); 
                }
            } else if (comandoProcesar == "TEST") {
                imprimirSerial("Ejecutando comando TEST: mostrando mensaje personalizado y parpadeando LED_STATUS 10s", 'c');
                Heltec.display->clear();
                Heltec.display->drawString(0, 0, "TEST: Esta es una prueba de LoRa");
                Heltec.display->display();
                unsigned long startTime = millis();
                while (millis() - startTime < 10000) {
                    digitalWrite(LED_STATUS, HIGH);
                    delay(100);
                    digitalWrite(LED_STATUS, LOW);
                    delay(100);
                }
            }
            // Otros comandos se pueden agregar aquí.
        } else if (red == 'V') {
            imprimirSerial("Comando para red Vecinal (UART).", 'g');
            String comandoVecinal = comandoProcesar + ">" + valorAleatorio;
            ManejoComunicacion::escribirVecinal(comandoVecinal); 
        }
    }
    // Otros destinos se ignoran por completo
}