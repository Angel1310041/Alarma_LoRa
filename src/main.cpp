#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "heltec.h"
#include "config.h"
#include "pantalla.h"
#include "comunicacion.h"
#include "eeprom.h"
#include "hardware.h"
#include "interfaz.h"
#include "freertos/queue.h"

SX1262 lora = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
String Version = "1.1.1.1";
volatile bool receivedFlag = false;
bool modoProgramacion = false;

TaskHandle_t tareaComandosSerial = NULL;
TaskHandle_t tareaComandosVecinal = NULL;
TaskHandle_t tareaComandosLoRa = NULL;
TaskHandle_t tareaLoraTx = NULL;

extern String ultimoComandoRecibido; 
#define MSG_ID_BUFFER_SIZE 16
String msgIdBuffer[MSG_ID_BUFFER_SIZE];
int msgIdBufferIndex = 0;

#define MAX_NODOS_ACTIVOS 32
String nodosActivos[MAX_NODOS_ACTIVOS];
int numNodosActivos = 0;
String mensaje = "";

#define LORA_TX_QUEUE_SIZE 8
typedef struct {
    String destino;
    String comando;
    char red;
    String valorAleatorio;
} LoraTxMsg;
QueueHandle_t loraTxQueue = NULL;

void imprimirSerial(String mensaje, char color) {
  String colorCode;
  switch (color) {
    case 'r': colorCode = "\033[31m"; break; 
    case 'g': colorCode = "\033[32m"; break; 
    case 'b': colorCode = "\033[34m"; break; 
    case 'y': colorCode = "\033[33m"; break; 
    case 'c': colorCode = "\033[36m"; break; 
    case 'm': colorCode = "\033[35m"; break; 
    case 'w': colorCode = "\033[37m"; break; 
    default: colorCode = "\033[0m"; 
  }
  Serial.print(colorCode);
  Serial.println(mensaje);
  Serial.print("\033[0m");
}

void agregarNodoActivo(const String& id) {
    for (int i = 0; i < numNodosActivos; i++)
        if (nodosActivos[i] == id) return;
    if (numNodosActivos < MAX_NODOS_ACTIVOS)
        nodosActivos[numNodosActivos++] = id;
}

void limpiarNodosActivos() { numNodosActivos = 0; }

void mostrarNodosActivos() {
    Serial.println("Nodos activos detectados:");
    for (int i = 0; i < numNodosActivos; i++)
        Serial.println(" - " + nodosActivos[i]);
}

String generarMsgID() { return String(configLora.IDLora) + "-" + String(millis()); }

bool esMsgDuplicado(const String& msgId) {
    for (int i = 0; i < MSG_ID_BUFFER_SIZE; i++)
        if (msgIdBuffer[i] == msgId) return true;
    return false;
}

void guardarMsgID(const String& msgId) {
    msgIdBuffer[msgIdBufferIndex] = msgId;
    msgIdBufferIndex = (msgIdBufferIndex + 1) % MSG_ID_BUFFER_SIZE;
}

void setFlag() { receivedFlag = true; }

void encolarTransmisionLoRa(const String& destino, char red, const String& comando, const String& valorAleatorio) {
    LoraTxMsg msg;
    msg.destino = destino;
    msg.comando = comando;
    msg.red = red;
    msg.valorAleatorio = valorAleatorio;
    if (xQueueSend(loraTxQueue, &msg, 0) != pdPASS) {
        imprimirSerial("Cola LoRa llena, no se pudo encolar el mensaje.", 'r');
    }
}

void tareaLoraTransmit(void *pvParameters) {
    LoraTxMsg msg;
    while (true) {
        if (xQueueReceive(loraTxQueue, &msg, portMAX_DELAY) == pdPASS) {
            esp_task_wdt_reset();
            int numAzar = msg.valorAleatorio.length() > 0 ? msg.valorAleatorio.toInt() : random(10, 99);
            String paqueteMsg = msg.destino + "@" + msg.red + "@" + msg.comando + "@" + String(numAzar);
            String siguienteHop = msg.destino, msgID = String(configLora.IDLora) + "-" + String(millis());
            char paquete[256];
            snprintf(paquete, sizeof(paquete), "ORIG:%s|DEST:%s|MSG:%s|HOP:%s|CANAL:%d|ID:%s",
                configLora.IDLora, msg.destino.c_str(), paqueteMsg.c_str(), siguienteHop.c_str(), configLora.Canal, msgID.c_str());
            lora.standby();
            int resultado = lora.transmit(paquete);
            esp_task_wdt_reset();
            lora.startReceive();
            if (resultado == RADIOLIB_ERR_NONE) {
                Hardware::manejarComandoPorFuente("lora");
                guardarMsgID(msgID);
            }
            vTaskDelay(1 / portTICK_PERIOD_MS); // Muy pequeño delay para máxima velocidad
        }
    }
}

void enviarComandoEstructurado(const String& destino, char red, const String& comando) {
    String valorAleatorio = String(random(10, 99));
    encolarTransmisionLoRa(destino, red, comando, valorAleatorio);
}

void recibirComandoSerial(void *pvParameters) {
    imprimirSerial("Esperando comandos por Serial...", 'b');
    tareaComandosSerial = xTaskGetCurrentTaskHandle();
    String comandoSerial = "";
    while (true) {
        comandoSerial = ManejoComunicacion::leerSerial();
        comandoSerial.trim();

        if (!comandoSerial.isEmpty()) {
            imprimirSerial("Comando recibido por Serial: " + comandoSerial, 'y');
            Hardware::manejarComandoPorFuente("serial");
            int idx1 = comandoSerial.indexOf('@');
            int idx2 = comandoSerial.indexOf('@', idx1 + 1);
            int idx3 = comandoSerial.indexOf('@', idx2 + 1);
            if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
                String destino = comandoSerial.substring(0, idx1);
                char red = comandoSerial.charAt(idx1 + 1);
                String comando = comandoSerial.substring(idx2 + 1, idx3);

                // --- Transmitir primero y luego procesar local ---
                if (strcmp(destino.c_str(), "000") == 0) {
                    enviarComandoEstructurado(destino, red, comando);
                }
                if (strcmp(destino.c_str(), configLora.IDLora) == 0 || strcmp(destino.c_str(), "000") == 0) {
                    ManejoComunicacion::procesarComando(comandoSerial, String(configLora.IDLora));
                } else {
                    enviarComandoEstructurado(destino, red, comando);
                }
            } else {
                imprimirSerial("Formato inválido. Usa: ID@R@CMD@##", 'r');
                mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), Version);
            }
            ultimoComandoRecibido = comandoSerial;
        }
        esp_task_wdt_reset();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void recibirComandosLoRa(void *pvParameters) {
    imprimirSerial("Esperando comandos por LoRa...", 'b');
    tareaComandosLoRa = xTaskGetCurrentTaskHandle();
    String msg;

    while (true) {
        if (receivedFlag) {
            receivedFlag = false;
            int state = lora.readData(msg); 
            Hardware::manejarComandoPorFuente("lora");
            if (state == RADIOLIB_ERR_NONE) {
                int idxId = msg.indexOf("ID:");
                int idxPipeId = msg.indexOf("|", idxId);
                String msgId = "";
                if (idxId != -1) {
                    if (idxPipeId != -1)
                        msgId = msg.substring(idxId + 3, idxPipeId);
                    else
                        msgId = msg.substring(idxId + 3);
                    msgId.trim();
                }
                if (!msgId.isEmpty() && esMsgDuplicado(msgId)) {
                    lora.startReceive();
                    mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), Version);
                    continue;
                }
                if (!msgId.isEmpty()) {
                    guardarMsgID(msgId);
                }

                String destinoRecibido = "";
                int idxDest = msg.indexOf("DEST:");
                int idxPipeDest = msg.indexOf("|", idxDest);
                if (idxDest != -1 && idxPipeDest != -1) {
                    destinoRecibido = msg.substring(idxDest + 5, idxPipeDest);
                    destinoRecibido.trim();
                }

                if (strcmp(destinoRecibido.c_str(), configLora.IDLora) == 0 || strcmp(destinoRecibido.c_str(), "000") == 0) {
                    int idxMsg = msg.indexOf("MSG:");
                    int idxPipe = msg.indexOf("|", idxMsg);
                    if (idxMsg != -1) {
                        String comandoRecibido;
                        if (idxPipe != -1)
                            comandoRecibido = msg.substring(idxMsg + 4, idxPipe);
                        else
                            comandoRecibido = msg.substring(idxMsg + 4);
                        comandoRecibido.trim();

                        int idx1 = comandoRecibido.indexOf('@');
                        int idx2 = comandoRecibido.indexOf('@', idx1 + 1);
                        int idx3 = comandoRecibido.indexOf('@', idx2 + 1);
                        if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
                            String comando = comandoRecibido.substring(idx2 + 1, idx3);
                            if (comando.indexOf("RESP:") != -1) {
                                lora.startReceive();
                                mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), Version);
                                continue; // NO PROCESAR
                            }
                        } else {
                            lora.startReceive();
                            mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), Version);
                            continue;
                        }

                        String origen = ""; 
                        int idxOrig = msg.indexOf("ORIG:");
                        int idxPipeOrig = msg.indexOf("|", idxOrig);
                        if (idxOrig != -1 && idxPipeOrig != -1) {
                            origen = msg.substring(idxOrig + 5, idxPipeOrig);
                        }
                        //mostrarMensajeRecibido(origen, comandoRecibido); 
                        ManejoComunicacion::procesarComando(comandoRecibido, origen); 
                        ultimoComandoRecibido = comandoRecibido;
                    }
                }
            }
            lora.startReceive();
            mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), Version);
        }
        esp_task_wdt_reset();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void recibirComandosVecinal(void *pvParameters) {
  imprimirSerial("Esperando comandos por UART (Vecinal)...", 'b');
  tareaComandosVecinal = xTaskGetCurrentTaskHandle();
  String comandoVecinal = "";
  while (true) {
    comandoVecinal = ManejoComunicacion::leerVecinal();
    comandoVecinal.trim();

    if (!comandoVecinal.isEmpty()) {
      int idx1 = comandoVecinal.indexOf('@');
      int idx2 = comandoVecinal.indexOf('@', idx1 + 1);
      int idx3 = comandoVecinal.indexOf('@', idx2 + 1);

      if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
        String destino = comandoVecinal.substring(0, idx1);
        char red = comandoVecinal.charAt(idx1 + 1);
        String comando = comandoVecinal.substring(idx2 + 1, idx3);

        // Transmitir primero en caso de broadcast
        if (strcmp(destino.c_str(), "000") == 0) {
            enviarComandoEstructurado(destino, red, comando);
        }
        if (strcmp(destino.c_str(), configLora.IDLora) == 0 || strcmp(destino.c_str(), "000") == 0) {
            ManejoComunicacion::procesarComando(comandoVecinal, String(configLora.IDLora));
        } else {
            enviarComandoEstructurado(destino, red, comando);
        }
      } else {
        imprimirSerial("Formato inválido. Usa: ID@R@CMD@##", 'r');
        mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), Version);
      }
      ultimoComandoRecibido = comandoVecinal;
    }
    esp_task_wdt_reset();
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void setup() {
    configLora.DEBUG = true; 
    Serial.begin(9600);
    delay(1000);

    Heltec.begin(false, false, true);
    inicializarPantalla();

    Hardware::inicializar();
    ManejoComunicacion::inicializar();
    ManejoEEPROM::leerTarjetaEEPROM(); 
    configurarDisplay(configLora.Pantalla);
    pinMode(BOTON_MODO_PROG, INPUT_PULLUP);
    if (strlen(configLora.IDLora) == 0) {
        imprimirSerial("ID no encontrado en memoria, usando valor por defecto.", 'r');
        strcpy(configLora.IDLora, "001");
        ManejoEEPROM::guardarTarjetaConfigEEPROM();
    } else {
        imprimirSerial("ID cargado de memoria: " + String(configLora.IDLora), 'g');
    }

    if (configLora.Canal < 0 || configLora.Canal > 8) {
        imprimirSerial("Canal no válido en memoria, usando valor por defecto.", 'r');
        configLora.Canal = 0;
        ManejoEEPROM::guardarTarjetaConfigEEPROM();
    } else {
        imprimirSerial("Canal cargado de memoria: " + String(configLora.Canal) + " (" + String(canales[configLora.Canal], 1) + " MHz)", 'g');
    }

    if (lora.begin(canales[configLora.Canal]) != RADIOLIB_ERR_NONE) {
        imprimirSerial("LoRa init failed!", 'r'); mostrarError("LoRa init failed!"); while (true);
    }
    lora.setOutputPower(17);      
    lora.setSpreadingFactor(7);   // Mayor velocidad, menor alcance
    lora.setBandwidth(250.0);     
    lora.setCodingRate(7);
    lora.setDio1Action(setFlag);
    lora.startReceive();
    pinMode(LED_STATUS, OUTPUT);

    imprimirSerial("LoRa ready.", 'g');
    imprimirSerial("ID de este nodo: " + String(configLora.IDLora), 'c');
    imprimirSerial("Canal: " + String(configLora.Canal) + " (" + String(canales[configLora.Canal], 1) + " MHz)", 'c');
    imprimirSerial("Escribe en el formato: ID@R@CMD@##", 'y');
    imprimirSerial("Ejemplo: A01@L@GETID@42", 'y');
    imprimirSerial("Version:" + Version, 'm');

    mostrarEstadoLoRa(String(configLora.IDLora), String(configLora.Canal), Version);

    // --- Inicializa la cola y la tarea de transmisión LoRa ---
    loraTxQueue = xQueueCreate(LORA_TX_QUEUE_SIZE, sizeof(LoraTxMsg));
    if (loraTxQueue == NULL) {
        imprimirSerial("Error al crear la cola de transmisión LoRa.", 'r');
        while (true);
    }
    xTaskCreatePinnedToCore(
        tareaLoraTransmit,
        "LoRa TX",
        4096,
        NULL,
        1,
        &tareaLoraTx,
        1
    );

    if (!modoProgramacion && tareaComandosSerial == NULL && configLora.DEBUG) {
      imprimirSerial("Iniciando tarea de recepcion de comandos Seriales...", 'c');
      xTaskCreatePinnedToCore(
        recibirComandoSerial,
        "Comandos Seriales",
        5120,
        NULL,
        1,
        &tareaComandosSerial,
        0
      );
      imprimirSerial("Tarea de recepcion de comandos Seriales iniciada", 'c');
    }

    if (!modoProgramacion && tareaComandosLoRa == NULL) {
      imprimirSerial("Iniciando tarea de recepcion de comandos LoRa...", 'c');
      xTaskCreatePinnedToCore(
        recibirComandosLoRa,
        "Comandos LoRa",
        5120,
        NULL,
        1,
        &tareaComandosLoRa,
        0
      );
      imprimirSerial("Tarea de recepcion de comandos LoRa iniciada", 'c');
    }

    if (!modoProgramacion && tareaComandosVecinal == NULL && configLora.UART) {
      imprimirSerial("Iniciando tarea de recepcion de comandos Vecinal...", 'c');
      xTaskCreatePinnedToCore(
        recibirComandosVecinal,
        "Comandos Vecinales",
        5120,
        NULL,
        1,
        &tareaComandosVecinal,
        0
      );
      imprimirSerial("Tarea de recepcion de comandos Vecinal iniciada", 'c');
    }

}

void loop() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    modoprogporbotonfisico();

    if (io1TimerActive && (millis() - io1TimerStart >= IO_TIMEOUT_MS)) {
        digitalWrite(PIN_IO1, LOW);
        io1TimerActive = false;
        io1TimerStart = 0;
        imprimirSerial("PIN_IO1 apagado automáticamente tras 5 minutos.", 'y');
    }
    if (io2TimerActive && (millis() - io2TimerStart >= IO_TIMEOUT_MS)) {
        digitalWrite(PIN_IO2, LOW);
        io2TimerActive = false;
        io2TimerStart = 0;
        imprimirSerial("PIN_IO2 apagado automáticamente tras 5 minutos.", 'y');
    }
}