#include "trayectoria.h"

// --- PINES ---
const int ENA = 13; const int IN1 = 12; const int IN2 = 14; 
const int ENB = 25; const int IN3 = 27; const int IN4 = 26; 
const int BOTON_BOOT = 0; 

// --- PARÁMETROS DE "DOMESTICACIÓN" ---
const float L_virtual = 0.45; // EXTREMO: Forzará giros sobre el eje para cerrar la mariposa
const float max_v = 3.5;      // EXTREMO: Hará que el robot avance MUY lento y cubra poca distancia

// --- CALIBRACIÓN DE ARRANQUE ---
const int MIN_PWM_DER = 100;   // Bajado para recuperar control
const int MIN_PWM_IZQ = 120;   // El izquierdo siempre un poco más fuerte por tu hardware

const float BOOST_IZQ = 1.30;  

// --- FRECUENCIA ---
const int freq = 400;  // Frecuencia baja para mantener torque a baja velocidad
const int resolucion = 8;

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  ledcAttach(ENA, freq, resolucion);
  ledcAttach(ENB, freq, resolucion);
  frenar();

  pinMode(BOTON_BOOT, INPUT_PULLUP);
  Serial.println("--- INTENTO 17: MODO TORTUGA ---");

  while (digitalRead(BOTON_BOOT) == HIGH) { delay(50); }
  delay(2000); 
}

void loop() {
  for (int k = 0; k < total_pasos; k++) {
    float v = pgm_read_float(&v_ref[k]);
    float w = pgm_read_float(&w_ref[k]);

    // Cálculo con L_virtual extremo para forzar el giro cerrado
    float v_der = v + (w * L_virtual / 2.0);
    float v_izq = v - (w * L_virtual / 2.0);

    // Mapeo con max_v muy alto (PWM resultante será pequeño)
    int pwm_der_raw = abs(v_der) * (255.0 / max_v);
    int pwm_izq_raw = abs(v_izq) * (255.0 / max_v) * BOOST_IZQ;

    // Compensación de zona muerta baja
    int pwm_der = (pwm_der_raw > 2) ? map(pwm_der_raw, 1, 255, MIN_PWM_DER, 220) : 0;
    int pwm_izq = (pwm_izq_raw > 2) ? map(pwm_izq_raw, 1, 255, MIN_PWM_IZQ, 220) : 0;

    // Limitamos a 220 para no estresar las pilas y evitar el reinicio (Brownout)
    controlMotores(constrain(pwm_der, 0, 220), v_der >= 0, 
                   constrain(pwm_izq, 0, 220), v_izq >= 0);
    
    // Si quieres que vaya aún más lento, puedes subir este delay a 110 o 120
    delay(100); 
  }

  frenar();
  Serial.println("Fin.");
  while (true); 
}

void controlMotores(int p_der, bool fwd_der, int p_izq, bool fwd_izq) {
  digitalWrite(IN1, fwd_der ? HIGH : LOW);
  digitalWrite(IN2, fwd_der ? LOW : HIGH);
  ledcWrite(ENA, p_der); 
  digitalWrite(IN3, fwd_izq ? HIGH : LOW);
  digitalWrite(IN4, fwd_izq ? LOW : HIGH);
  ledcWrite(ENB, p_izq); 
}

void frenar() {
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
