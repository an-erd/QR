#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// Servo: Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface, https://www.adafruit.com/product/815
// MUX: SparkFun Analog/Digital MUX Breakout - CD74HC4067, https://www.sparkfun.com/products/9056
// IR: https://learn.adafruit.com/ir-sensor/overview, http://www.crcibernetica.com/ir-receiver-diode/
// https://github.com/bborncr/Hexapod_AdafruitShield/blob/master/Hexapod_AdafruitShield.ino

//Mux control pins and SIG (D74HC4067)
#define MUX_S0                A0
#define MUX_S1                A1
#define MUX_S2                A2
#define MUX_S3                A3
#define MUX_SIG               A7

#define LED_PIN_BLUE          3
#define LED_PIN_RED           9
#define LED_PIN_YELLOW        10
#define LED_PIN_GREEN         11

#define BUTTON_PIN_RED        4
#define BUTTON_PIN_YELLOW     5
#define BUTTON_PIN_GREEN      6

// SDA -> Analog 4
// SCL -> Analog 5
#define SERVOMIN              150 // min pulse length count (out of 4096)
#define SERVOMAX              600 // max pulse length count (out of 4096)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t servonum =            0;  // our servo # counter

void setup() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);

  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(60);
}


void loop() {

  for (int i = 0; i < 16; i ++) {
    Serial.print(readMux(i));
    Serial.print("\t");
  }
  Serial.println("");
  delay(1000);

/*
  pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);

  // Drive each PWM in a 'wave'
  for (uint16_t i=0; i<4096; i += 8) {
    for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) {
      pwm.setPWM(pwmnum, 0, (i + (4096/16)*pwmnum) % 4096 );
    }
  }

 // Drive each servo one at a time
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
  }

  delay(500);

  servonum ++;
  if (servonum > 7) servonum = 0;
*/
}


int readMux(int channel) {
  int controlPin[] = {MUX_S0, MUX_S1, MUX_S2, MUX_S3};

  int muxChannel[16][4] = {
    {0, 0, 0, 0}, //channel 0
    {1, 0, 0, 0}, //channel 1
    {0, 1, 0, 0}, //channel 2
    {1, 1, 0, 0}, //channel 3
    {0, 0, 1, 0}, //channel 4
    {1, 0, 1, 0}, //channel 5
    {0, 1, 1, 0}, //channel 6
    {1, 1, 1, 0}, //channel 7
    {0, 0, 0, 1}, //channel 8
    {1, 0, 0, 1}, //channel 9
    {0, 1, 0, 1}, //channel 10
    {1, 1, 0, 1}, //channel 11
    {0, 0, 1, 1}, //channel 12
    {1, 0, 1, 1}, //channel 13
    {0, 1, 1, 1}, //channel 14
    {1, 1, 1, 1}  //channel 15
  };

  //loop through the 4 sig
  for (int i = 0; i < 4; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  int val = analogRead(MUX_SIG);

  //return the value
  return val;
}

