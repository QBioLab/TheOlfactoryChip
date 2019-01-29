/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/

unsigned int Relay1 = 29;
unsigned int Relay2 = 31;
unsigned int Relay3 = 33;
unsigned int Relay4 = 35;

// Pressure Sensor
unsigned int pressure_port = 0; // A0
unsigned int sensorValue  = 0;
float voltageValue  = 0;
unsigned int pressureValue  = 0;
unsigned int a  = 0;

// Button and LED
unsigned int hold1 = 22;
unsigned int hold1_led = 24;
unsigned int hold2 = 30;
unsigned int hold2_led = 32;
unsigned int hold3 = 38;
unsigned int hold3_led = 40;
unsigned int holds = 46;
unsigned int holds_led = 48;

unsigned int pushs = 52;
unsigned int pushs_led = 50;
unsigned int holdv = 28;
unsigned int holdv_led = 26;
unsigned int pushu = 36;
unsigned int pushu_led = 34;
unsigned int pushd = 44;
unsigned int pushd_led = 42;

// Brushless motor
unsigned int motor = 2;
int pwm = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // Relay
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  digitalWrite(Relay1, LOW);
  digitalWrite(Relay2, LOW);
  digitalWrite(Relay3, LOW);
  digitalWrite(Relay4, LOW);

  // Button and LED
  pinMode(hold1, INPUT);
  pinMode(hold1_led, OUTPUT);
  pinMode(hold2, INPUT);
  pinMode(hold2_led, OUTPUT);
  pinMode(hold3, INPUT);
  pinMode(hold3_led, OUTPUT);
  pinMode(holds, INPUT);
  pinMode(holds_led, OUTPUT);
  
  pinMode(pushs, INPUT);
  pinMode(pushs_led, OUTPUT);
  pinMode(holdv, INPUT);
  pinMode(holdv_led, OUTPUT);
  pinMode(pushu, INPUT);
  pinMode(pushu_led, OUTPUT);
  pinMode(pushd, INPUT);
  pinMode(pushd_led, OUTPUT);
    
  digitalWrite(hold1_led, LOW);
  digitalWrite(hold2_led, LOW);
  digitalWrite(hold3_led, LOW);
  digitalWrite(holds_led, LOW);
  
  digitalWrite(pushs_led, LOW);
  digitalWrite(pushu_led, LOW);
  digitalWrite(pushd_led, LOW);
  digitalWrite(holdv_led, LOW);

  pinMode(motor, OUTPUT);
  analogWrite(motor, pwm);

  //debug  
  pinMode(LED_BUILTIN, OUTPUT);

  // PWM freq
  TCCR3B = TCCR2B & 0b11111000 | 0x01;  // change freq to 33k
}

// the loop routine runs over and over again forever:
void loop() {
  // Pressure sensor test
  Serial.println(pressure(pressure_port));

  // Relay Test
  /*
  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);
  */

  // Button and LED
  buttontest(hold1, hold1_led);
  buttontest(hold2, hold2_led);
  buttontest(hold3, hold3_led);
  buttontest(holds, holds_led);
  buttontest(holdv, holdv_led);
  buttontest(pushs, pushs_led);
  buttontest(pushd, pushd_led);
  buttontest(pushu, pushu_led);



  // burushless motor
  //analogWrite(motor, 255);
  if(digitalRead(pushu)){
    pwm++;
    if(pwm>255){
      pwm = 255;
      }
    analogWrite(motor,pwm);
    analogWrite(LED_BUILTIN, pwm);
  }
  if(digitalRead(pushd)){
    pwm--;
    if(pwm <0 ){
      pwm = 0;
      }
    analogWrite(motor, pwm);
    analogWrite(LED_BUILTIN, pwm);
  }
  Serial.print("PWM: ");
  Serial.println(pwm);
  //delay(1);

}

void buttontest(unsigned int button, unsigned int led) {
  if (digitalRead(button)) {
    digitalWrite(led, HIGH);
  } else {
    digitalWrite(led, LOW);
  }

}

float pressure(unsigned int port) {
  sensorValue = analogRead(port);
  voltageValue = sensorValue * (5000.0 / 1023.0);
  if (voltageValue <= 4500) {
    a = 4500 - voltageValue;
  }
  pressureValue = a * 0.0225;
  if (pressureValue > 0) {
    sensorValue = 0;
    a = 0;
    voltageValue = 0;
  }
  //Serial.println("kpa");
  return pressureValue;
}
