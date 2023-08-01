#include <Wire.h>
#include <ZumoMotors.h>

#define I2C_SLAVE_ADDRESS  0x11

ZumoMotors motors;

struct {
  int16_t rightSpeed;
  int16_t leftSpeed;
} motorSpeed;

union {
  uint8_t raw[2];
  uint16_t value;
} vel_l, vel_r;

uint8_t safeCounter = 0;
bool dataArrive = false;

void moveF(int16_t vel_l, int16_t vel_r){
  motors.setSpeeds(vel_l, vel_r);
}

void receiveData(int byteCount);

void setup() {
  Serial.begin(9600);

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveData);

  motorSpeed.rightSpeed = 0;
  motorSpeed.leftSpeed  = 0;

//  motors.flipRightMotor(true);
}

void loop() {
    moveF(motorSpeed.leftSpeed, motorSpeed.rightSpeed);
}

// callback received data
void receiveData(int byteCount) {
    
  Wire.read();

  vel_l.raw[1] = (int8_t) Wire.read();
  vel_l.raw[0] = (int8_t) Wire.read();

  vel_r.raw[1] = (int8_t) Wire.read();
  vel_r.raw[0] = (int8_t) Wire.read();

  motorSpeed.leftSpeed = vel_l.value;
  motorSpeed.rightSpeed = vel_r.value;

  Serial.print("leftSpeed: ");
  Serial.print(motorSpeed.leftSpeed);
  Serial.print("\t rightSpeed: ");
  Serial.println(motorSpeed.rightSpeed);
  
}
