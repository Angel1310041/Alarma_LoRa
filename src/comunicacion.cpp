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
unsigned long io1TimerSeconds = 0; // segs dinámica para OUT
unsigned long io2TimerSeconds = 0;

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
    esp_task_wdt_reset();
    String msgID = String(configLora.IDLora) + "-" + String(millis());
    char paquete[256];
    snprintf(paquete, sizeof(paquete), "ORIG:%s|DEST:%s|MSG:%s|HOP:%s|CANAL:%d|ID:%s",
        configLora.IDLora, destino.c_str(), comandoLoRa.c_str(), destino.c_str(), configLora.Canal, msgID.c_str());
    imprimirSerial("Enviando comando " + String(paquete) + " a lora con el ID " + destino);
    lora.standby();
    int resultado = lora.transmit(paquete);
    esp_task_wdt_reset();
    lora.startReceive();
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void ManejoComunicacion::gestionarTimersSalidas() {
    if (io1TimerActive && io1TimerSeconds > 0) {
        if ((millis() - io1TimerStart) >= io1TimerSeconds * 1000) {
            digitalWrite(PIN_IO1, LOW);
            io1TimerActive = false;
            io1TimerSeconds = 0;
            imprimirSerial("PIN_IO1 apagado por timeout dinámica.", 'y');
        }
    }
    if (io2TimerActive && io2TimerSeconds > 0) {
        if ((millis() - io2TimerStart) >= io2TimerSeconds * 1000) {
            digitalWrite(PIN_IO2, LOW);
            io2TimerActive = false;
            io2TimerSeconds = 0;
            imprimirSerial("PIN_IO2 apagado por timeout dinámica.", 'y');
        }
    }
}

// --- Comando OUT actualizado: OUTxyz (x=salida, y=estado, zzz=timer) ---
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

    if (red == 'V') {
        imprimirSerial(">> Comando recibido desde la Vecinal: " + comandoRecibido, 'c');
    }

    if (strcmp(idDestino.c_str(), configLora.IDLora) == 0 || strcmp(idDestino.c_str(), "000") == 0) {
        imprimirSerial("Comando dirigido a este nodo (" + String(configLora.IDLora) + ") o es broadcast (000).", 'g');

        // --------- Nuevo comando OUT[x][y][zzz] ----------
        if (comandoProcesar.startsWith("OUT")) {
            // Formato: OUTxyz, donde
            // x = id salida (0=ambas, 1=IO1, 2=IO2)
            // y = estado (0=apagado, 1=encender)
            // zzz = timer (en segundos, tres dígitos)

            if (comandoProcesar.length() >= 7) {
                char salidaChar = comandoProcesar.charAt(3);
                char estadoChar = comandoProcesar.charAt(4);
                String timerStr = comandoProcesar.substring(5, 8);

                int salidaId = salidaChar - '0';
                int estado = estadoChar - '0';
                int timerSegundos = timerStr.toInt();

                if ((salidaId >= 0 && salidaId <= 2) && (estado == 0 || estado == 1) && timerSegundos >= 0) {
                    if (salidaId == 0) {
                        // Ambas salidas
                        pinMode(PIN_IO1, OUTPUT);
                        pinMode(PIN_IO2, OUTPUT);
                        digitalWrite(PIN_IO1, estado ? HIGH : LOW);
                        digitalWrite(PIN_IO2, estado ? HIGH : LOW);

                        if (estado == 1 && timerSegundos > 0) {
                            io1TimerStart = millis();
                            io2TimerStart = millis();
                            io1TimerActive = true;
                            io2TimerActive = true;
                            io1TimerSeconds = timerSegundos;
                            io2TimerSeconds = timerSegundos;
                        } else {
                            io1TimerActive = false;
                            io2TimerActive = false;
                            io1TimerSeconds = 0;
                            io2TimerSeconds = 0;
                        }
                    } else if (salidaId == 1) {
                        pinMode(PIN_IO1, OUTPUT);
                        digitalWrite(PIN_IO1, estado ? HIGH : LOW);
                        if (estado == 1 && timerSegundos > 0) {
                            io1TimerStart = millis();
                            io1TimerActive = true;
                            io1TimerSeconds = timerSegundos;
                        } else {
                            io1TimerActive = false;
                            io1TimerSeconds = 0;
                        }
                    } else if (salidaId == 2) {
                        pinMode(PIN_IO2, OUTPUT);
                        digitalWrite(PIN_IO2, estado ? HIGH : LOW);
                        if (estado == 1 && timerSegundos > 0) {
                            io2TimerStart = millis();
                            io2TimerActive = true;
                            io2TimerSeconds = timerSegundos;
                        } else {
                            io2TimerActive = false;
                            io2TimerSeconds = 0;
                        }
                    }
                    imprimirSerial("OUT: Pin " + String(salidaId) + " => " + String(estado) + " por " + String(timerSegundos) + "seg", 'g');
                } else {
                    imprimirSerial("Formato OUT invalido o fuera de rango.", 'r');
                }
            } else {
                imprimirSerial("Formato OUT demasiado corto.", 'r');
            }
        }
        // --- Resto de comandos: mantén igual ---
        if (red == 'L') {
            if (comandoProcesar.startsWith("ID")) {
                String accion = comandoProcesar.substring(2, 3);
                if (accion == "C" && comandoProcesar.length() >= 7) {
                    String nuevoId = comandoProcesar.substring(4, 7);
                    imprimirSerial("Cambiando el ID del nodo a -> " + nuevoId, 'c');
                    strncpy(configLora.IDLora, nuevoId.c_str(), sizeof(configLora.IDLora) - 1);
                    configLora.IDLora[sizeof(configLora.IDLora) - 1] = '\0';
                    ManejoEEPROM::guardarTarjetaConfigEEPROM();
                }
            } else if (comandoProcesar.startsWith("CH")) {
                String accion = comandoProcesar.substring(2, 3);
                if (accion == "C") {
                    int idxMayor = comandoProcesar.indexOf('>');
                    if (idxMayor != -1 && idxMayor + 1 < comandoProcesar.length()) {
                        String canalStr = comandoProcesar.substring(idxMayor + 1);
                        imprimirSerial("Cambiando el canal del nodo a -> " + canalStr, 'c');
                        int canalNuevo = canalStr.toInt();
                        if (canalNuevo >= 0 && canalNuevo < (sizeof(canales) / sizeof(canales[0]))) {
                            configLora.Canal = canalNuevo;
                            ManejoEEPROM::guardarTarjetaConfigEEPROM();
                            lora.begin(canales[configLora.Canal]);
                            esp_restart();
                        }
                    }
                }
            } else if (comandoProcesar.startsWith("SCR")) {
                String accion = comandoProcesar.substring(4, 5);
                if (accion == "0") {
                    imprimirSerial("Desactivando la pantalla");
                    configurarDisplay(false);
                } else if (accion == "1") {
                    imprimirSerial("Activando la pantalla");
                    configurarDisplay(true);
                }
            } else if (comandoProcesar.startsWith("FORMAT")) {
                imprimirSerial("Reiniciando la tarjeta LoRa...");
                delay(1000);
                ManejoEEPROM::borrarTarjetaConfigEEPROM();
                esp_restart();
            } else if (comandoProcesar.startsWith("RESET")) {
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
        } else if (red == 'V') {
            imprimirSerial("Comando para red Vecinal (UART).", 'g');
            String comandoVecinal = comandoProcesar + ">" + valorAleatorio;
            ManejoComunicacion::escribirVecinal(comandoVecinal);
        }
    }
}

// ManejoComunicacion::gestionarTimersSalidas();