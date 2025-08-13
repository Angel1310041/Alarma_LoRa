#include "config.h"
#include "hardware.h"
#include "eeprom.h"

#define JSON_FILE "/condicionalesGPIO.json"

// Arreglo con los números de pin fisicos
const int pinNumbers[6] = {PIN_IO1, PIN_IO2, PIN_IO3, PIN_IO4, PIN_IO5, PIN_IO6};
// Arreglo con los nombres para mensajes
const char* pinNames[6] = {"IO1", "IO2", "IO3", "IO4", "IO5", "IO6"};

// --- Variables para tarea de parpadeo asíncrono ---
TaskHandle_t tareaParpadeoLED = NULL;
String fuenteParpadeoPendiente = "";

/*struct ParametroGPIO {
  char nombre;
  int pin;
  char flanco;
  bool estado;
};

struct AccionGPIO {
  char nombre;
  int pin;
  char flanco;
  bool activo;
};

struct CondicionalGPIO {
  int condicion;
  ParametroGPIO parametro[6];
  AccionGPIO accion;
  bool cumplido;
};*/

CondicionalGPIO condicionales[CANT_CONDICIONALES];

TaskHandle_t tareaEvaluarCondicionales = NULL;

CondicionalGPIO Hardware::leerCondicional(int numCondicional) {

}

bool Hardware::escribirCondicional(CondicionalGPIO nuevoCondicional) {
  if (!SPIFFS.begin(true)) {
    imprimirSerial("Error al montar SPIFFS");
    return false;
  }

  // Leer archivo existente o crear uno nuevo si no existe
  DynamicJsonDocument doc(8192);
  if (SPIFFS.exists(JSON_FILE)) {
    File file = SPIFFS.open(JSON_FILE, FILE_READ);
    if (!file) return false;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error) {
      imprimirSerial("Error al parsear JSON");
      return false;
    }
  } else {
    // Estructura base si no existe
    doc["condicionalesGPIO"] = JsonArray();
  }

  JsonArray arr = doc["condicionalesGPIO"].as<JsonArray>();

  // Buscar si ya existe el condicional (por número)
  bool encontrado = false;
  for (JsonObject obj : arr) {
    for (JsonPair kv : obj) {
      String key = kv.key().c_str();
      if (key != "resultado") {
        if (key.toInt() == nuevoCondicional.condicion) {
          // Sobreescribir este condicional
          JsonArray parametros = kv.value().as<JsonArray>();
          for (int i = 0; i < 6; i++) {
            parametros[i]["PARAMETRO"] = String(nuevoCondicional.parametro[i].nombre);
            parametros[i]["PIN"] = nuevoCondicional.parametro[i].pin;
            parametros[i]["FLANCO"] = String(nuevoCondicional.parametro[i].flanco);
            parametros[i]["ESTADO"] = nuevoCondicional.parametro[i].estado;
          }
        }
      }
    }
  }
}

String Hardware::obtenerTodosCondicionales() {

}

bool Hardware::ejecutarCondicionales() {
  bool condCumplido = false;

  for (int i = 0; i < CANT_CONDICIONALES; i++) {
    for (int j = 0; j < 6; j++) {
      condCumplido = condicionales[i].parametro[j].estado;
    }
    if (condicionales[i].cumplido) {
      condicionales[i].accion.activo = true;
    }
  }
}


/*void evaluarCondicionales(void *pvParameters) {
  imprimirSerial("Esperando condiciones GPIO...");
  tareaEvaluarCondicionales = xTaskGetCurrentTaskHandle();

  while(true) {
    actualizarEstadoPinesGPIO();
    for (int i = 0; i < CANT_CONDICIONALES; i++) {
      bool cumple = true;
      for (int j = 0; j < 6; j++) {
        int pin = condicionales[i].parametro[j].pin;
        char flanco = condicionales[i].parametro[j].flanco;
        if (!verificarFlanco(pin, flanco)) {
          cumple = false;
          break;
        }
      }
      if (cumple) {
        AccionGPIO accion = buscarAccion(condicionales[i].condicion);
        ejecutarAccion(accion);
      }
    }
  }
  vTaskDelete(NULL);
}
*/


// --- Prototipo de la tarea ---
void tareaParpadeoLEDHandler(void *pvParameters);



void Hardware::configurarPinesGPIO(char pinesIO[6], char Flancos[6]) {
  String mensajePines = ""; // Variable para armar mensaje personalizado
  for (int i = 0; i < 6; ++i) { // Bucle for para configurar los pines GPIO
      if (pinesIO[i] == 'E') {// Condicional para entradas
      mensajePines = "Pin " + String(pinNames[i]) + " configurado como entrada con flanco ";
      if (Flancos[i] == 'A') {
        mensajePines += "ascendente"; // Activacion con pulsos altos
      } else if (Flancos[i] == 'D') {
        mensajePines += "descendente"; // Activacion con pulsos bajos
      } else {
        mensajePines += "no especificado"; // Sin especificar
      }
      imprimirSerial(mensajePines, 'c');
      pinMode(pinNumbers[i], INPUT);

    } else if (pinesIO[i] == 'S') { // Condicional para salidas
      mensajePines = "Pin " + String(pinNames[i]) + " configurado como entrada con flanco ";
      if (Flancos[i] == 'A') {
        mensajePines += "ascendente"; // Salida activa con pulso alto
      } else if (Flancos[i] == 'D') {
        mensajePines += "descendente"; // Salida activa con pulso bajo
      } else {
        mensajePines += "no especificado"; // Salida con pulso indefinido
      }
      imprimirSerial(mensajePines, 'c');
      pinMode(pinNumbers[i], OUTPUT);

    } else { // Condicional para pines no especificados
      imprimirSerial("Pin " + String(pinNames[i]) + " no especificado", 'y');
    }
  }
}

void Hardware::blinkPin(int pin, int times, int delayTime) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    vTaskDelay(delayTime / portTICK_PERIOD_MS);
    digitalWrite(pin, LOW);
    vTaskDelay(delayTime / portTICK_PERIOD_MS);
  }
}

void Hardware::manejoEstrobo(int pin, int freq, int delayTime) {
  // Enciende el pin una vez durante delayTime ms
  digitalWrite(pin, HIGH);
  vTaskDelay(delayTime / portTICK_PERIOD_MS);
  digitalWrite(pin, LOW);
}

void tareaParpadeoLEDHandler(void *pvParameters) {
  String fuente = *(String*)pvParameters;
  delete (String*)pvParameters;
  if (fuente == "wifi") {
    Hardware::blinkPin(LED_STATUS, 2, 300);
  } else if (fuente == "serial") {
    Hardware::blinkPin(LED_STATUS, 3, 200);
  } else if (fuente == "lora") {
    Hardware::manejoEstrobo(LED_STATUS, 1, 600);
  }
  tareaParpadeoLED = NULL;
  vTaskDelete(NULL);
}

void Hardware::manejarComandoPorFuente(const String& fuente) {
  // Solo permite una tarea de parpadeo a la vez
  if (tareaParpadeoLED == NULL) {
    String* fuenteCopia = new String(fuente);
    xTaskCreatePinnedToCore(
      tareaParpadeoLEDHandler,
      "ParpadeoLED",
      2048,
      fuenteCopia,
      1,
      &tareaParpadeoLED,
      1
    );
  }
}

void Hardware::inicializar() {
    ManejoEEPROM::tarjetaNueva();
    pinMode(LED_STATUS, OUTPUT);
    pinMode(TEST_IN, INPUT);
    Hardware::configurarPinesGPIO(configLora.PinesGPIO, configLora.FlancosGPIO);

    // --- Asegura que los pines de RF estén siempre como salidas ---
    pinMode(PIN_IO1, OUTPUT);
    pinMode(PIN_IO2, OUTPUT);
    digitalWrite(PIN_IO1, LOW); // Inicialmente apagados
    digitalWrite(PIN_IO2, LOW);

    /*if (!modoProgramacion && tareaEvaluarCondicionales == NULL) {
        imprimirSerial("Iniciando tarea de Evaluacion de Condicionales GPIO...", 'c');
        xTaskCreatePinnedToCore(
            evaluarCondicionales,
            "Condicionales GPIO",
            5120,
            NULL,
            1,
            &tareaEvaluarCondicionales,
            0
        );
        imprimirSerial("Tarea de Evaluacion de Condicionales iniciada", 'c');
    }*/
}
