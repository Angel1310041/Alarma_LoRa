#include <Arduino.h>
#include "eeprom.h"


Preferences eeprom;
const int EEPROM_SIZE = 512;
const int DIRECCION_INICIO_CONFIG = 0;
Network redes[3];

void ManejoEEPROM::leerTarjetaEEPROM() {
  eeprom.begin("EEPROM_PPAL", false);
  configLora.magic = eeprom.getInt("magic");
  String idLeido = eeprom.getString("IDLora");
  strcpy(configLora.IDLora, idLeido.c_str());
  configLora.Canal = eeprom.getInt("Canal");
  configLora.Pantalla = eeprom.getBool("Pantalla");
  configLora.UART = eeprom.getBool("UART");
  configLora.I2C = eeprom.getBool("I2C");
  configLora.WiFi = eeprom.getBool("WiFi");
  configLora.DEBUG = eeprom.getBool("DEBUG");
  String GPIOS = eeprom.getString("PinesGPIO");
  strcpy(configLora.PinesGPIO, GPIOS.c_str());
  String FLANCOS = eeprom.getString("FlancosGPIO");
  strcpy(configLora.FlancosGPIO, FLANCOS.c_str());
  eeprom.end();
}

void ManejoEEPROM::guardarTarjetaConfigEEPROM() {
  eeprom.begin("EEPROM_PPAL", false);
  eeprom.putInt("magic", configLora.magic);
  eeprom.putString("IDLora", configLora.IDLora);
  eeprom.putInt("Canal", configLora.Canal);
  eeprom.putBool("Pantalla", configLora.Pantalla);
  eeprom.putBool("UART", configLora.UART);
  eeprom.putBool("I2C", configLora.I2C);
  eeprom.putBool("WiFi", configLora.WiFi);
  eeprom.putBool("DEBUG", configLora.DEBUG);
  eeprom.putString("PinesGPIO", configLora.PinesGPIO);
  eeprom.putString("FlancosGPIO", configLora.FlancosGPIO);
  eeprom.end();

  imprimirSerial("Configuracion guardada: ");
  imprimirSerial("ID: " + String(configLora.IDLora));
  imprimirSerial("Canal: " + String(configLora.Canal));
  imprimirSerial("Pantalla: " + String(configLora.Pantalla));
  imprimirSerial("UART: " + String(configLora.UART));
  imprimirSerial("DEBUG: " + String(configLora.DEBUG));
  imprimirSerial("I2C: " + String(configLora.I2C));

  for (int i = 0; i < 6; ++i) {
    if (configLora.PinesGPIO[i] == 1) {
      imprimirSerial("o- Pin " + String(pinNames[i]) + " configurado como entrada", 'c');
    } else if (configLora.PinesGPIO[i] == 2) {
      imprimirSerial("o- Pin " + String(pinNames[i]) + " configurado como salida", 'c');
    } else {
      imprimirSerial("o- Pin " + String(pinNames[i]) + " no especificado", 'y');
    }
  }
}

void ManejoEEPROM::borrarTarjetaConfigEEPROM() {
    eeprom.begin("EEPROM_PPAL", false);
    eeprom.clear();
    eeprom.end();
    Serial.println("Configuración borrada. Reinicia el dispositivo o espera...");
    delay(1000);
    ESP.restart();
}

void ManejoEEPROM::tarjetaNueva() {
  ManejoEEPROM::leerTarjetaEEPROM();
  if (configLora.magic != 0xDEADBEEF) {
    imprimirSerial("Esta tarjeta es nueva, comenzando formateo...", 'c');
    configLora.magic = 0xDEADBEEF;
    strcpy(configLora.IDLora, "001");
    configLora.Canal = 0;
    configLora.Pantalla = false;
    configLora.UART = true;
    configLora.I2C = false;
    configLora.DEBUG = true;
    //strcpy(configLora.PinesGPIO, "IIIIII");
    //strcpy(configLora.FlancosGPIO, "NNNNNN");

    ManejoEEPROM::guardarTarjetaConfigEEPROM();

    imprimirSerial("\n\t\t\t<<< Tarjeta formateada correctamente >>>", 'g');
    imprimirSerial("Version de la tarjeta: " + Version);
  } else {
    imprimirSerial("\n\t\t\t<<< Tarjeta lista para utilizarse >>>", 'y');
  }
}
//Guarda una red WiFi (ssid y password) en un slot disponible (hay máximo 3). Si todos los slots están ocupados, reemplaza el primero.
bool ManejoEEPROM::guardarWiFiCredenciales(const char* ssid, const char* password) {
    // Buscar un slot vacío o reemplazar el más antiguo
    int emptySlot = -1;
    for (int i = 0; i < 3; i++) {
        if (strlen(redes[i].ssid) == 0) {
            emptySlot = i;
            break;
        }
    }
    
    // Si no hay slots vacíos, reemplazar el primero
    if (emptySlot == -1) emptySlot = 0;
    
    // Guardar las credenciales
    strncpy(redes[emptySlot].ssid, ssid, sizeof(redes[emptySlot].ssid));
    strncpy(redes[emptySlot].password, password, sizeof(redes[emptySlot].password));
    
    // Guardar en EEPROM
    guardarRedesEEPROM();
    
    imprimirSerial(String("Red guardada en slot ") + emptySlot + ": " + ssid, 'g');
    return true;
}

//Llena un arreglo JSON (JsonArray) con las redes WiFi guardadas (solo sus SSID).
void ManejoEEPROM::obtenerRedesGuardadas(JsonArray& redesJson) {
    for (int i = 0; i < 3; ++i) {
        if (strlen(redes[i].ssid) > 0) {
            JsonObject red = redesJson.createNestedObject();
            red["ssid"] = redes[i].ssid;
        }
    }
}

//Elimina una red WiFi guardada con el SSID especificado
bool ManejoEEPROM::eliminarRedWiFi(const char* ssid) {
    for (int i = 0; i < 3; ++i) {
        if (strcmp(redes[i].ssid, ssid) == 0) {
            memset(&redes[i], 0, sizeof(Network));
            guardarRedesEEPROM();
            return true;
        }
    }
    return false;
}

//Guarda todas las redes actualmente almacenadas en el arreglo redes[] a la EEPROM 
void ManejoEEPROM::guardarRedesEEPROM() {
    Preferences prefs;
    prefs.begin("WIFI_NETS", false);
    
    for (int i = 0; i < 3; i++) {
        String ssidKey = "ssid_" + String(i);
        String passKey = "pass_" + String(i);
        String favKey = "fav_" + String(i);
        
        if (strlen(redes[i].ssid) > 0) {
            prefs.putString(ssidKey.c_str(), redes[i].ssid);
            prefs.putString(passKey.c_str(), redes[i].password);
            prefs.putBool(favKey.c_str(), redes[i].isFavorite);
        } else {
            prefs.remove(ssidKey.c_str());
            prefs.remove(passKey.c_str());
            prefs.remove(favKey.c_str());
        }
    }
    
    prefs.end();
}

//cargar las redes desde EEPROM y guardarlas en el arreglo redes[] al iniciar el programa
void ManejoEEPROM::cargarRedesEEPROM() {
  Preferences prefs;
    prefs.begin("WIFI_NETS", true);
    
    for (int i = 0; i < 3; i++) {
        String ssidKey = "ssid_" + String(i);
        String passKey = "pass_" + String(i);
        String favKey = "fav_" + String(i);
        
        if (prefs.isKey(ssidKey.c_str())) {
            String ssid = prefs.getString(ssidKey.c_str(), "");
            String password = prefs.getString(passKey.c_str(), "");
            bool isFavorite = prefs.getBool(favKey.c_str(), false);
            
            if (ssid.length() > 0) {
                strncpy(redes[i].ssid, ssid.c_str(), sizeof(redes[i].ssid));
                strncpy(redes[i].password, password.c_str(), sizeof(redes[i].password));
                redes[i].isFavorite = isFavorite;
            }
        }
    }
    
    prefs.end();
}

// Nueva función para marcar red favorita
bool ManejoEEPROM::guardarRedFavorita(const char* ssid) {
    Preferences prefs;
    prefs.begin("WIFI_NETS", false);
    
    // Primero desmarcar cualquier favorita existente
    for(int i = 0; i < 3; i++) {
        String favKey = "fav_" + String(i);
        prefs.putBool(favKey.c_str(), false);
        redes[i].isFavorite = false;
    }
    
    // Buscar y marcar la nueva favorita
    for(int i = 0; i < 3; i++) {
        String keySSID = "ssid_" + String(i);
        if(prefs.getString(keySSID.c_str(), "") == String(ssid)) {
            String favKey = "fav_" + String(i);
            prefs.putBool(favKey.c_str(), true);
            redes[i].isFavorite = true;
            prefs.end();
            return true;
        }
    }
    
    prefs.end();
    return false;
}//función para conectar a la red favorita al iniciar
bool ManejoEEPROM::conectarRedFavorita() {
    cargarRedesEEPROM(); // Asegurarse de tener las redes actualizadas
    
    for(int i = 0; i < 3; i++) {
        if(redes[i].isFavorite && strlen(redes[i].ssid) > 0) {
            imprimirSerial(String("Conectando a red favorita: ") + redes[i].ssid, 'g');
            
            WiFi.begin(redes[i].ssid, redes[i].password);
            
            unsigned long startTime = millis();
            while(WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
                delay(500);
                imprimirSerial(".", 'y');
            }
            
            if(WiFi.status() == WL_CONNECTED) {
                imprimirSerial("\nConectado a red favorita!", 'g');
                imprimirSerial("IP: " + WiFi.localIP().toString(), 'g');
                return true;
            } else {
                imprimirSerial("\nError al conectar a red favorita", 'r');
                return false;
            }
        }
    }
    
    imprimirSerial("No hay red favorita configurada", 'y');
    return false;
}
