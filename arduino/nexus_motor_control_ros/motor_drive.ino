#include <PinChangeInt.h>

// pin number definition
// motor #1
// PWM: Pin 3, dir: Pin 2, encoder(A&B): Pin 4, 5
const int PIN_MOTOR_PWM_1 = 3;
const int PIN_MOTOR_DIR_1 = 2;
const int PIN_ENCODER_1A = 4;
const int PIN_ENCODER_1B = 5;
// motor #2
// PWM: Pin 11, dir: Pin 12, encoder(A&B): Pin 6, 7
const int PIN_MOTOR_PWM_2 = 11;
const int PIN_MOTOR_DIR_2 = 12;
const int PIN_ENCODER_2A = 6;
const int PIN_ENCODER_2B = 7;

// initialize for motor drive (PWM freq. and pin mode)
void initMotorDrive() {
  // change PWM freq
  TCCR1B=TCCR1B&0xf8|0x01;  // Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;  // Pin3,Pin11 PWM 31250Hz

  // set pin mode
  pinMode(PIN_MOTOR_PWM_1, OUTPUT);
  pinMode(PIN_MOTOR_PWM_2, OUTPUT);
  pinMode(PIN_ENCODER_1A, INPUT);
  pinMode(PIN_ENCODER_1B, INPUT);
  pinMode(PIN_ENCODER_2A, INPUT);
  pinMode(PIN_ENCODER_2B, INPUT);

  // set interrupt handler for encoder pulse count
  PCintPort::attachPinChangeInterrupt(PIN_ENCODER_1A, riseEnc1A, RISING);
  PCintPort::attachPinChangeInterrupt(PIN_ENCODER_2A, riseEnc2A, RISING);
}

// interrupt handlers for encoder pulse count
// called when the pulse of phase A rises
// check the pulse of phase B to count encCountPerInstant
void riseEnc1A() {
     if (digitalRead(PIN_ENCODER_1B) == HIGH) encCount_Instant1++;
     else encCount_Instant1--;
}

void riseEnc2A() {
   if (digitalRead(PIN_ENCODER_2B) == HIGH) encCount_Instant2++;
   else encCount_Instant2--;
}

// limit motor command (-255 ~ 255)
inline int limitMotorCommand(int com) {
  if (com > 255) return 255;
  if (com < -255) return -255;
  return com;
}

// set a motor command to drive a motor
// setMotorCommand?() for motor ? (?: number of motor)
//
// Parameters
// com: the motor command to apply a voltage to the motor (-255 ~ 255)
//      positive for CW (255 for 12V)
//      negative for CCW (-255 for -12V)
//
// Returns
// nothing
inline void setMotorCommand1(int com) {
  if (com > 0) digitalWrite(PIN_MOTOR_DIR_1, HIGH);
  else digitalWrite(PIN_MOTOR_DIR_1, LOW);
  
  analogWrite(PIN_MOTOR_PWM_1, abs(com));
}

inline void setMotorCommand2(int com) {
  if (com > 0) digitalWrite(PIN_MOTOR_DIR_2, HIGH);
  else digitalWrite(PIN_MOTOR_DIR_2, LOW);
  
  analogWrite(PIN_MOTOR_PWM_2, abs(com));
}

