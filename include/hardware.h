#ifndef HARDWARE_H
  #define HARDWARE_H
  #include "config.h"


  #define CANT_CONDICIONALES 12

  struct ParametroGPIO {
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
  };

  extern CondicionalGPIO condicionales[CANT_CONDICIONALES];

  class Hardware {
    public:
      static void inicializar();
      static void tarjetaNueva();
      static void blinkPin(int pin, int times, int delayTime);
      static void manejoEstrobo(int pin, int freq, int delayTime);
      static void configurarPinesGPIO(char PinesGPIO[6], char Flancos[6]);
      static void manejarComandoPorFuente(const String& fuente);
      static CondicionalGPIO leerCondicional(int numCondicional);
      static bool escribirCondicional(CondicionalGPIO nuevoCondicional);
      static String obtenerTodosCondicionales();
      static bool ejecutarCondicionales();
  };

#endif