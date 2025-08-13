#ifndef PANTALLA_H
#define PANTALLA_H

#include <Arduino.h>
#include "heltec.h"

extern bool displayActivo; // Declare as extern so it can be accessed from main.cpp

void inicializarPantalla();
void mostrarMensaje(const String& titulo, const String& mensaje, int delayMs = 2000);
void limpiarPantalla();
void mostrarEstadoLoRa(const String& idNodo, const String& canal, const String& version);
void mostrarMensajeRecibido(const String& origen, const String& mensaje);
void mostrarMensajeEnviado(const String& destino, const String& mensaje);
void mostrarError(const String& mensajeError);
void mostrarInfo(const String& mensajeInfo);
void configurarDisplay(bool habilitar); // New function to enable/disable display

#endif // PANTALLA_H