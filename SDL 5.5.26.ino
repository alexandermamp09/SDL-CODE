#include <EEPROM.h>


// --- PARÁMETROS PID (Sugiero empezar con Ki y Kd en 0) ---
float Kp = 0.24;   // Aumentado: el error máximo es 200, 200*0.8 = 160 de corrección
float Ki = 0.1;  
float Kd = 1;   

// Variables Globales PID
float integral = 0;
float LIMITE_INTEGRAL = 20; 

// Pines Puente H (Mantengo tus pines)
int ENA = 9; int IN1 = 4; int IN2 = 5;
int ENB = 6; int IN3 = 2; int IN4 = 3;

const int numSensores = 5;
int pinesSensores[numSensores] = {A4, A3, A0, A2, A1};
const int pesos[5] = {100, 200, 300, 400, 500};
int valorMin[numSensores], valorMax[numSensores];

float setpoint = 300;
int VELOCIDAD_BASE = 100; // Sube esto si el mínimo es 60 (2r 3az, 4r 5n)
float ultimo_error = 0;
unsigned long tiempo_anterior = 0;

//Modo Inercia

int inercia_min = 60;
int inercia_max= 120;

int velA;
int velB;
          


// --- VARIABLES MÁQUINA DE ESTADOS ---
enum EstadoRobot { SIGUIENDO_LINEA, MODO_INERCIA, MODO_RECUPERACION };
EstadoRobot estadoActual = SIGUIENDO_LINEA;

unsigned long tiempoPerdido = 0;
const int UMBRAL_INERCIA = 700; // ms que el robot irá recto (ajusta según el hueco)
float ultima_correccion = 0;   // Para recordar qué hacía el PID antes de perderse


float calcular_pid(float error_actual, float dt) {
  // Proporcional
  float P = Kp * error_actual;

  // Integral (Ahora global y acumulativa)
  integral += error_actual * dt;
  integral = constrain(integral, -LIMITE_INTEGRAL, LIMITE_INTEGRAL);
  float I = Ki * integral;

  // Derivativo (Evita fluctuaciones por DT pequeño)
  float D = Kd * ((error_actual - ultimo_error) / dt);

  return P + I + D;
}

float calcularError() {
  float sumaPonderada = 0;
  float sumaLecturas = 0;

  for (int i = 0; i < 5; i++) {
    float valor = obtenerLecturaNormalizada(i);
    sumaPonderada += (valor * pesos[i]);
    sumaLecturas += valor;
  }

  if (sumaLecturas > 50) { // Umbral mínimo para detectar línea
    return (sumaPonderada / sumaLecturas) - setpoint;
  } else {
    return NAN; // Not a Number para indicar pérdida de línea
  }
}

// --- FUNCIONES AUXILIARES ---



// MOVER MOTORES //
void moverMotores(int velA, int velB) {
  // --- CONTROL MOTOR A (Izquierdo) ---
  if (velA >= 0) {
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(velA, 0, 255));
  } else {
    digitalWrite(IN1, LOW); // Invertimos para reversa
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(abs(velA), 0, 255));
  }

  // --- CONTROL MOTOR B (Derecho) ---
  if (velB >= 0) {
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(velB, 0, 255));
  } else {
    digitalWrite(IN3, HIGH); // Invertimos para reversa
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(abs(velB), 0, 255));
  }



}

void configurarMotores() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Sentido motor A
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Sentido motor B
}

int obtenerLecturaNormalizada(int i) {
  int lectura = analogRead(pinesSensores[i]);
  int normalizado = map(lectura, valorMin[i], valorMax[i], 0, 1000);
  return constrain(normalizado, 0, 1000);
}

void cargarCalibracion() {
  for (int i = 0; i < numSensores; i++) {
    EEPROM.get(0 + (i * 2), valorMin[i]);
    EEPROM.get(20 + (i * 2), valorMax[i]);
  }
}

void mostrarValoresCargados() {
  for (int i = 0; i < numSensores; i++) {
    Serial.print("Sensor "); Serial.print(i);
    Serial.print(": Min = "); Serial.print(valorMin[i]);
    Serial.print(" | Max = "); Serial.println(valorMax[i]);
  }}


// --- NUEVAS VARIABLES PARA BIFURCACIONES Y BUCLES ---
int contadorVueltas = 0;
int limiteVueltasParaCambio = 2; // A la 3ra vez que pase, cambia prioridad
bool enInterseccion = false;
String prioridadActual = "DERECHA"; // Prioridad inicial: "IZQUIERDA", "DERECHA" o "RECTO"

// Función para detectar bifurcaciones/intersecciones
void gestionarBifurcaciones() {
  bool s_izq = obtenerLecturaNormalizada(0) > 600; // Sensor extremo izquierdo
  bool s_der = obtenerLecturaNormalizada(4) > 600; // Sensor extremo derecho
  bool s_centro = obtenerLecturaNormalizada(2) > 600; // Sensor central

  // Detectamos si estamos en una bifurcación (más de una opción clara)
  if ((s_izq && s_centro) || (s_der && s_centro) || (s_izq && s_der)) {
    
    if (!enInterseccion) { // Solo ejecutamos la decisión una vez al entrar
      enInterseccion = true;
      
      if (contadorVueltas >= limiteVueltasParaCambio) {
        // Cambiar prioridad si detecta que está repitiendo
        prioridadActual = (prioridadActual == "DERECHA") ? "IZQUIERDA" : "DERECHA";
        contadorVueltas = 0; // Reiniciamos contador tras el cambio
      }

      ejecutarGiroPrioritario();
    }
  } else {
    enInterseccion = false; 
  }
}

void ejecutarGiroPrioritario() {
  if (prioridadActual == "DERECHA") {
    moverMotores(100, 0); // Giro rápido a la derecha
    // Tiempo para "saltar" la bifurcación
  } 
  else if (prioridadActual == "IZQUIERDA") {
    moverMotores(0, 100); // Giro rápido a la izquierda
    
  }
  // Si es RECTO, simplemente dejamos que el PID siga su curso
}

// Lógica para contar "vueltas" o marcas en la pista
// Puedes llamar a esta función si tienes una marca lateral o por tiempo
void registrarPasoPorPuntoControl() {
  static unsigned long ultimoPaso = 0;
  if (millis() - ultimoPaso > 5000) { // Evita contar dos veces el mismo punto
    contadorVueltas++;
    ultimoPaso = millis();
    Serial.print("Vuelta registrada: "); Serial.println(contadorVueltas);
  }
}




void setup() {

  Serial.begin(115200);
  configurarMotores();
  cargarCalibracion();

  mostrarValoresCargados();
  
  pinMode(10, OUTPUT);
  analogWrite(10, 255); // Iluminación sensores
  
  tiempo_anterior = millis();
}

void loop() {

  
  unsigned long tiempo_actual = millis();
  float DT = (tiempo_actual - tiempo_anterior) / 1000.0;

  if (DT >= 0.01) {
    tiempo_anterior = tiempo_actual;
    gestionarBifurcaciones(); 

    float error = calcularError();

    // --- MÁQUINA DE ESTADOS ---
    if (!isnan(error)) {
      // ESTADO 1: SIGUIENDO LÍNEA
      estadoActual = SIGUIENDO_LINEA;
      tiempoPerdido = 0; // Resetear cronómetro
      
      float correccion = calcular_pid(error, DT);
      ultima_correccion = correccion; // Guardamos la última acción
      
      int vel_din = VELOCIDAD_BASE - (abs(error) * 0.2);
      if (vel_din < 40) vel_din = 40;
      int velmin = -60; 
      int velmax = 100;
      velA = constrain(vel_din - correccion, velmin, velmax);
      velB = constrain(vel_din + correccion, velmin, velmax);
      moverMotores(velA, velB);
      ultimo_error = error;
    } 
    else {
      // SI NO HAY LÍNEA, DECIDIMOS SEGÚN EL TIEMPO
      if (tiempoPerdido == 0) tiempoPerdido = millis();
      unsigned long duracionPerdida = millis() - tiempoPerdido;

      if (duracionPerdida < UMBRAL_INERCIA) {
        // ESTADO 2: MODO INERCIA (Para líneas segmentadas)
        estadoActual = MODO_INERCIA;
        
        // Mantenemos la dirección anterior pero la suavizamos (multiplicamos por 0.7)
        velA = constrain(VELOCIDAD_BASE - (ultima_correccion * 0.7), inercia_min, inercia_max);
        velB = constrain(VELOCIDAD_BASE + (ultima_correccion * 0.7), inercia_min, inercia_max);
   
        moverMotores(velA, velB);
      } 
      else {
        // ESTADO 3: MODO RECUPERACIÓN (Se perdió de verdad)
        estadoActual = MODO_RECUPERACION;
        
        // Girar sobre su propio eje hacia donde vio la línea por última vez
        if (ultimo_error > 0) {
          moverMotores(80, -60); // Giro rápido derecha
        } else {
          moverMotores(-60, 80); // Giro rápido izquierda
    
        }

      }}
  }

 Serial.print("VelA: ");
 Serial.println(velA);
 Serial.print("VelB: ");
 Serial.println(velB);

 
 }

  
