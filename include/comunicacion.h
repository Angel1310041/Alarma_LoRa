#ifndef COMUNICACION_H
  #define COMUNICACION_H
  #include "config.h"

  struct Activacion {
    int pin;                   // Pin a controlar (IO2 o IO3)
    int tiempoDesactivacion;   // Tiempo en milisegundos
    bool activo;               // Estado actual
    TimerHandle_t timer;       // Timer asociado
};

static Activacion activaciones[2] = {
    {PIN_IO1, 0, false, NULL},
    {PIN_IO2, 0, false, NULL}
};

  class ManejoComunicacion {
    public:
      static void inicializar();
      static String leerSerial();
      static String leerVecinal();
      static void initUART();
      static void escribirVecinal(String envioVecinal);
      static void procesarComando(const String &comandoRecibido, const String &idOrigen);
      static void envioMsjLoRa(String comandoLoRa, const String& destino);
  };
void enviarComandoEstructurado(const String& destino, char red, const String& comando);
extern unsigned long io1TimerStart;
extern unsigned long io2TimerStart;
extern bool io1TimerActive;
extern bool io2TimerActive;
extern const unsigned long IO_TIMEOUT_MS;
#endif