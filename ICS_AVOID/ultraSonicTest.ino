/****************************************************
 *  FORWARD + OBSTACLE AVOIDANCE (HC-SR04)
 *  WITH MOTOR STATUS SERIAL OUTPUT
 ****************************************************/

// ================== PINOUT (TB6612FNG) ==================
const uint8_t PWMA = 9;
const uint8_t AIN1 = 8;
const uint8_t AIN2 = 7;

const uint8_t PWMB = 3;
const uint8_t BIN1 = 5;
const uint8_t BIN2 = 4;

const uint8_t STBY = 6;

// ================== HC-SR04 PINS ==================
const uint8_t TRIG_PIN = 10;
const uint8_t ECHO_PIN = 11;

// ================== USER CONFIGURATION ==================
const uint8_t FWD_PWM  = 100;
const uint8_t TURN_PWM = 110;
const uint8_t BACK_PWM = 100;

const int OBSTACLE_CM = 20;   // distance threshold

// ================== MOTOR HELPERS ==================
inline uint8_t clampPWM(int v){ return (uint8_t)constrain(v,0,255); }

void leftMotorForward(uint8_t pwm){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, clampPWM(pwm));
}
void leftMotorBackward(uint8_t pwm){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, clampPWM(pwm));
}
void leftMotorStop(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
}

void rightMotorForward(uint8_t pwm){
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, clampPWM(pwm));
}
void rightMotorBackward(uint8_t pwm){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, clampPWM(pwm));
}
void rightMotorStop(){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

void motorsBrake(){
  Serial.println("MOTORS: STOP");
  leftMotorStop();
  rightMotorStop();
}

// Keeping original direction logic
void motorsForward(uint8_t l, uint8_t r){
  Serial.println("MOTORS: FORWARD");
  leftMotorBackward(l);
  rightMotorBackward(r);
}

void motorsReverse(uint8_t l, uint8_t r){
  Serial.println("MOTORS: REVERSE");
  leftMotorForward(l);
  rightMotorForward(r);
}

void pivotLeft(uint8_t l, uint8_t r){
  Serial.println("MOTORS: TURN LEFT");
  leftMotorBackward(l);
  rightMotorForward(r);
}

void pivotRight(uint8_t l, uint8_t r){
  Serial.println("MOTORS: TURN RIGHT");
  leftMotorForward(l);
  rightMotorBackward(r);
}

// ================== HC-SR04 READ ==================
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1;

  return duration / 58;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(STBY, HIGH);
  motorsBrake();

  Serial.println("System Ready: HC-SR04 Obstacle Avoidance");
}

// ================== MAIN LOOP ==================
void loop() {

  long distance = readDistanceCM();

  Serial.print("Ultrasonic Distance: ");
  if (distance < 0) Serial.println("No reading");
  else {
    Serial.print(distance);
    Serial.println(" cm");
  }

  // ---------- OBSTACLE DETECTED ----------
  if (distance > 0 && distance <= OBSTACLE_CM) {
    Serial.println("OBSTACLE DETECTED!");

    motorsBrake();
    delay(150);

    motorsReverse(BACK_PWM, BACK_PWM);
    delay(250);

    motorsBrake();
    delay(100);

    pivotRight(TURN_PWM, TURN_PWM);
    delay(350);

    motorsBrake();
    delay(100);
    return;
  }

  // ---------- CLEAR PATH ----------
  Serial.println("PATH CLEAR");
  motorsForward(FWD_PWM, FWD_PWM);
  delay(50);
}
