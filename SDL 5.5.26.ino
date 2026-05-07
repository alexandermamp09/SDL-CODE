#include <EEPROM.h>

float promedioErrorReciente = 0; // Para saber si veníamos rectos

// --- VARIABLES PARA FILTRO DE MEDIANA ---
float historialErrores[3] = {0, 0, 0};

// --- CONFIGURACIÓN DE SENSORES Y FILTRO ---
const int numSensores = 5;
// El orden debe ser físico: de izquierda a derecha
int pinesSensores[numSensores] = {A4, A3, A0, A2, A1}; 
const int pesos[5] = {1000, 2000, 3000, 4000, 5000}; 
float setpoint = 3000; // El centro ahora es el peso del sensor A0

// Calibración (Ahora ajustada a 3 sensores para evitar errores de índice)
int valorMin[5], valorMax[5];
// --- SISTEMA DE MUESTREO (FILTRO DE MEDIA MÓVIL) ---
const int TAMANO_FILTRO = 5; // Número de muestras para promediar
float muestras[TAMANO_FILTRO];
int indiceMuestra = 0;

// --- PARÁMETROS PID ---
float Kp = 0.08;   // Bajamos un poco más la agresividad inicial
float Ki = 0.004;   
float Kd = 0.04;    // Reducimos Kd para evitar los saltos bruscos de velocidad (de 123 a 74)

float integral = 0;
float LIMITE_INTEGRAL = 50; 

unsigned long UMBRAL_INERCIA = 3400;

// --- PINES PUENTE H ---
int ENA = 9; int IN1 = 4; int IN2 = 5;
int ENB = 6; int IN3 = 2; int IN4 = 3;

// --- VARIABLES DE MOVIMIENTO ---
// Variables para capturar el último estado antes del modo inercia
int ultimaVelA = 0;
int ultimaVelB = 0;
int VELOCIDAD_BASE = 70; 
float ultimo_error = 0;
unsigned long tiempo_anterior = 0;

int inercia_min = 60;
int inercia_max = 100;
int velA, velB;

// --- MÁQUINA DE ESTADOS ---
enum EstadoRobot { SIGUIENDO_LINEA, MODO_INERCIA, MODO_RECUPERACION };
EstadoRobot estadoActual = SIGUIENDO_LINEA;

unsigned long tiempoPerdido = 0;
float ultima_correccion = 0;

// --- VARIABLES BIFURCACIONES ---
int contadorVueltas = 0;
int limiteVueltasParaCambio = 1; 
bool enInterseccion = false;
String prioridadActual = "DERECHA";

// --- VARIABLE GLOBAL DE TIEMPO ---
unsigned long momentoPerdida = 0; 
bool perdiendoLinea = false;

// --- FUNCIONES CORE ---

int obtenerLecturaNormalizada(int i) {
  int lectura = analogRead(pinesSensores[i]);
  int normalizado = map(lectura, valorMin[i], valorMax[i], 0, 1000);
  return constrain(normalizado, 0, 1000);
}

float calcularError() {
  float sumaPonderada = 0;
  float sumaLecturas = 0;

  for (int i = 0; i < numSensores; i++) {
    float valor = obtenerLecturaNormalizada(i);
    sumaPonderada += (valor * pesos[i]);
    sumaLecturas += valor;
  }

  if (sumaLecturas > 150) { 
    float posicionInstantanea = sumaPonderada / sumaLecturas;
    float errorCrudo = posicionInstantanea - setpoint;

    return errorCrudo; 
  } else {
    return NAN; 
  }
}

float calcular_pid(float error_actual, float dt) {
  float P = Kp * error_actual;
  integral += error_actual * dt;
  integral = constrain(integral, -LIMITE_INTEGRAL, LIMITE_INTEGRAL);
  float I = Ki * integral;
  float D = Kd * ((error_actual - ultimo_error) / dt);
  return P + I + D;
}

void moverMotores(int vA, int vB) {
  // Motor A (Izquierdo)
  if (vA >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, constrain(abs(vA), 0, 255));

  // Motor B (Derecho)
  if (vB >= 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  }
  analogWrite(ENB, constrain(abs(vB), 0, 255));
}

void gestionarBifurcaciones() {
  bool s_extremo_izq = obtenerLecturaNormalizada(0) > 700; 
  bool s_extremo_der = obtenerLecturaNormalizada(4) > 700; 
  bool s_centro       = obtenerLecturaNormalizada(2) > 700; 

  if ((s_extremo_izq && s_centro) || (s_extremo_der && s_centro)) {
    if (!enInterseccion) {
      enInterseccion = true;
      // ... resto de tu lógica de prioridad ...
      ejecutarGiroPrioritario();
    }
  } else {
    enInterseccion = false; 
  }
}

void ejecutarGiroPrioritario() {
  if (prioridadActual == "DERECHA") moverMotores(120, -50); 
  else if (prioridadActual == "IZQUIERDA") moverMotores(-50, 120);
}

void cargarCalibracion() {
  for (int i = 0; i < numSensores; i++) {
    EEPROM.get(0 + (i * 2), valorMin[i]);
    EEPROM.get(20 + (i * 2), valorMax[i]);
  }
}



// --- SETUP Y LOOP ---

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(10, OUTPUT); analogWrite(10, 255); 

  cargarCalibracion();
  
  // Llenar el filtro inicialmente con el setpoint
  for(int i=0; i<TAMANO_FILTRO; i++) muestras[i] = setpoint;

  tiempo_anterior = millis();
}

void loop() {
  unsigned long tiempo_actual = millis();
  float DT = (tiempo_actual - tiempo_anterior) / 1000.0;

  if (DT >= 0.01) {
    tiempo_anterior = tiempo_actual;
    gestionarBifurcaciones(); 

    float error = calcularError();

  if (!isnan(error)) {
      // --- ESTADO: SIGUIENDO LÍNEA ---
      estadoActual = SIGUIENDO_LINEA;
      tiempoPerdido = 0;
      perdiendoLinea = false;
      
      // Calculamos un promedio rápido del error (suavizado)
      //promedioErrorReciente = (promedioErrorReciente * 0.3) + (abs(error) * 0.7);

      float correccion = calcular_pid(error, DT);
      // ... resto de tu código de velocidades ...

      int vel_din = VELOCIDAD_BASE - (abs(error) * 0.10); 
      if (vel_din < 30) vel_din = 30;

      velA = constrain(vel_din - correccion, -100, 120);
      velB = constrain(vel_din + correccion, -100, 120);
      
      // GUARDAMOS ESTA VELOCIDAD como la "Inercia Proporcional"
      ultimaVelA = velA;
      ultimaVelB = velB;
      
      moverMotores(velA, velB);
      ultimo_error = error;
    } 

  else {

      if (!perdiendoLinea) {
          momentoPerdida = tiempo_actual;
          perdiendoLinea = true;
        }
      // --- ESTADO: MODO INERCIA DINÁMICO ---
      
      unsigned long tiempoPerdido = tiempo_actual - momentoPerdida;

      if (tiempoPerdido < UMBRAL_INERCIA) {
        estadoActual = MODO_INERCIA;
        
        // Si veníamos estables, forzamos ir recto con decisión
        if (abs(ultimo_error) < 250) { // Subimos el umbral de 250 a 400
            // Usamos la VELOCIDAD_BASE para asegurar que cruce el hueco
            moverMotores(VELOCIDAD_BASE, VELOCIDAD_BASE * 1.2 ); 
        } 
        else {
            // Estamos en curva: mantenemos el giro que traíamos
            moverMotores(ultimaVelA, ultimaVelB);
        }
      } 
      else {
        // --- ESTADO: MODO RECUPERACIÓN (Evitar el giro de 180 grados) ---
        estadoActual = MODO_RECUPERACION;
        
        // Si veníamos de una recta (error bajo), NO giramos brusco, 
        // avanzamos un poco más lento para intentar encontrar la línea adelante
        if (abs(ultimo_error) < 300) {
            moverMotores(60, 60 * 1.2); 
        } 
        else {
            // Solo si veníamos de una curva cerrada giramos fuerte
            if (ultimo_error > 0) moverMotores(120, -80); 
            else moverMotores(-80, 120);
        }
      }
    }
   
    // Telemetría básica
    Serial.print("Error: "); Serial.println(error);
    Serial.print(" | VelA: "); Serial.print(velA);
    Serial.print(" | VelB: "); Serial.println(velB);
    Serial.print(" | ultimaVelA: "); Serial.print(ultimaVelA);
    Serial.print(" | ultimaVelB: "); Serial.println(ultimaVelB);
    
  }
}
