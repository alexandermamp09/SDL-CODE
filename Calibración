#include <EEPROM.h>

const int numSensores = 5;
const int pinesSensores[numSensores] = {A4, A3, A0, A2, A1};

// Direcciones de memoria iniciales
// Cada 'int' ocupa 2 bytes en la EEPROM
const int direccionBaseMin = 0;   // Direcciones 0, 2, 4, 6, 8
const int direccionBaseMax = 20;  // Direcciones 20, 22, 24, 26, 28

int valorMin[numSensores];
int valorMax[numSensores];

void setup() {

  pinMode(10, INPUT_PULLUP);
  Serial.begin(9600);
  
  // Inicialización de arreglos
  for (int i = 0; i < numSensores; i++) {
    valorMin[i] = 1023;
    valorMax[i] = 0;
  }

  if(digitalRead(10) == LOW) {
  Serial.println("--- MODO CALIBRACIÓN ---");
  Serial.println("Mueve el robot sobre la linea negra por 10 segundos...");
  delay(2000);

  // Calibración activa
  unsigned long tiempoInicio = millis();
  while (millis() - tiempoInicio < 10000) { // 10 segundos para mayor precisión
    for (int i = 0; i < numSensores; i++) {
      int lectura = analogRead(pinesSensores[i]);
      if (lectura < valorMin[i]) valorMin[i] = lectura;
      if (lectura > valorMax[i]) valorMax[i] = lectura;
    }
    
    // Feedback visual rápido
    if ((millis() % 500) == 0) Serial.print("."); 
  }

  Serial.println("\nCalibración completa. Guardando en EEPROM...");
  guardarEnEEPROM();
  
  Serial.println("Valores guardados con éxito:");
  mostrarValores();

  }
}

void loop() {
  // No hace nada, la calibración ya terminó.
}

void guardarEnEEPROM() {
  for (int i = 0; i < numSensores; i++) {
    // EEPROM.put maneja automáticamente el tamaño de los datos (int = 2 bytes)
    EEPROM.put(direccionBaseMin + (i * 2), valorMin[i]);
    EEPROM.put(direccionBaseMax + (i * 2), valorMax[i]);
  }
}

void mostrarValores() {
  for (int i = 0; i < numSensores; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" [Min: "); Serial.print(valorMin[i]);
    Serial.print(" Max: "); Serial.print(valorMax[i]);
    Serial.println("]");
  }
}
