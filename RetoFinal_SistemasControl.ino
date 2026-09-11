// variables del motor y encoder

volatile long count = 0;
volatile float current_speed = 0.0;
volatile float setpoint = 0.0;

// Pines del puente H
const int ENA = 6;
const int IN1 = 5;
const int IN2 = 4;

// Encoder
const int pinChannelA = 2;
const int pinChannelB = 3;

// frecuencia de control

const float fs = 50.0f;
const float Ts = 1.0f / fs;   // 0.020 s

// conversión de ticks a RPS
const float TICKS_TO_RPS = (1.0f / 35.0f) * (fs / 26.0f);

// filtro de velocidad

volatile float filteredSpeed = 0.0;
const float alpha = 0.2f;

// controlador PID

float Kp = 0.8215f;
float Ki = 7.8238f;
float Kd = 0.0110f;

// límites normalizados
const float U_MAX = 1.0f;
const float U_MIN = -1.0f;

// variables internas del PID
float speed_prev = 0.0f;
float integral = 0.0f;

// interrupciones del encoder 

void callback_A() {
  if (digitalRead(pinChannelB) == HIGH) count++;
  else count--;
}

void callback_B() {
  if (digitalRead(pinChannelA) == LOW) count++;
  else count--;
}

//  controlador PID con anti wind-up por clamping

float computePID(float target, float actual_speed) {
  // 1. error
  float error = target - actual_speed;

  // 2. componente proporcional
  float P = Kp * error;

  // 3. componente derivativo
  float derivative = -(actual_speed - speed_prev) / Ts;
  float D = Kd * derivative;
  speed_prev = actual_speed;

  // 4. límites de la integral
  // margen disponible después de aplicar P, y si ya saturó el margen es cero
  float i_max = U_MAX - P;
  float i_min = U_MIN - P;
  if (i_max < 0.0f) i_max = 0.0f;
  if (i_min > 0.0f) i_min = 0.0f;

  // 5. candidato de integración
  float integral_new = integral + (Ki * error * Ts);

  // 6. prueba de saturación
  // Se congela si la salida satura y el error empuja en la misma dirección de la saturación 
  float u_test = P + integral_new + D;
  bool empuja_arriba = (u_test > U_MAX) && (error > 0.0f);
  bool empuja_abajo  = (u_test < U_MIN) && (error < 0.0f);

  if (!empuja_arriba && !empuja_abajo) {
    integral = integral_new;
  }

  // 7. clamping 
  if (integral > i_max) integral = i_max;
  else if (integral < i_min) integral = i_min;

  // 8. salida y saturación física del actuador
  float u = P + integral + D;
  if (u > U_MAX) u = U_MAX;
  else if (u < U_MIN) u = U_MIN;

  return u;
}

  // control del motor
  void moveMotor(float data) {
  
  //limitar señal entre -1 y 1
  float pwm = max(U_MIN, min(data, U_MAX));

  // convertir a PWM 0-255
  int pwmValue = int(255.0f * abs(pwm));

  // dirección 
  if (pwm > 0.0f) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (pwm < 0.0f) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // escribir PWM
  analogWrite(ENA, pwmValue);
}

// TIMER1 - 50 Hz

ISR(TIMER1_COMPA_vect) {
  // 1. Obtener Ticks
  //    Al entrar a una ISR el hardware ya puso el flag I en 0
  long ticks = count;
  count = 0;

  // si el ciclo anterior todavía se está ejecutando, devolvemos los ticks al acumulador y se sale sin
  // corromper el estado del PID
  static bool busy = false;
  if (busy) { count += ticks; return; }
  busy = true;

  //el encoder sigue contando
  sei();

  // 2. se calcula la velocidad bruta 
  float raw_speed = float(ticks) * TICKS_TO_RPS;

  // 3. filtro pasa-bajas
  filteredSpeed = (alpha * raw_speed) + ((1.0f - alpha) * filteredSpeed);
  current_speed = filteredSpeed;

  // 4. calcular señal de control PID
  float control_signal = computePID(setpoint, current_speed);

  // 5. Aplicar al motor
  moveMotor(control_signal);

  cli();
  busy = false;
}

void setup() {
  // pines de potencia
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // pines del encoder
  pinMode(pinChannelA, INPUT_PULLUP);
  pinMode(pinChannelB, INPUT_PULLUP);

  // interrupciones de hardware para encoder nativas
  attachInterrupt(digitalPinToInterrupt(pinChannelA), callback_A, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinChannelB), callback_B, FALLING);

  // configuración de Timer1 a 50 Hz
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 1249;
  TCCR1B |= (1 << WGM12);  // modo CTC
  TCCR1B |= (1 << CS12);   // prescaler 256
  TIMSK1 |= (1 << OCIE1A); // interrupción por comparación
  sei();

  // inicializar detenido
  moveMotor(0.0f);

  // Serial para telemetría
  Serial.begin(115200);
  Serial.setTimeout(5);
}

//loop principal
void loop() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    float sp = inputString.toFloat();
    uint8_t sreg = SREG;
    cli();
    setpoint = sp;
    float speed_now = current_speed; 
    SREG = sreg;

    Serial.println(speed_now);
  }
}
