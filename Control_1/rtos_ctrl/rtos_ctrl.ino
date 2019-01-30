#include <Arduino_FreeRTOS.h>

// define two tasks for Blink & AnalogRead
void TaskBlink(      void *pvParameters );
void TaskMessage( void *pvParameters );
void TaskVacuumCtrl( void *pvParameters );
void TaskStimulate(  void *pvParameters );

void startStimulate();
void stopStimulate();
void channelCheck();
unsigned int rest_time = 30000; // 30s
unsigned int action_time = 5000; // 5000ms
unsigned int cycle = 3;

// relay
#define relay1  29
#define relay2  31
#define relay3  33
#define relay4  35

// Pressure Sensor
#define pressure_port 0 // A0
unsigned int sensorValue  = 0;
float voltageValue  = 0;
unsigned int pressureValue  = 0;
unsigned int a  = 0;

// Button and LED
#define hold1     22
#define hold1_led 24
#define hold2     38
#define hold2_led 40
#define hold3     46
#define hold3_led 48
#define holds     30
#define holds_led 32

#define pushs     52
#define pushs_led 50
#define holdv     28
#define holdv_led 26
#define pushu     36
#define pushu_led 34
#define pushd     44
#define pushd_led 42

// Brushless motor
#define motor   2
int motor_pwm = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  HardwareInit();

  xTaskCreate(
    TaskStimulate
    ,  (const portCHAR *) "Stimulate"
    ,  128 // This stack size can be checked & adjusted by reading Highwater
    ,  NULL
    ,  4  // priority
    ,  NULL );

  xTaskCreate(
    TaskVacuumCtrl
    , (const portCHAR *) "Contrl Vacuum pump"
    , 128
    , NULL
    , 3
    , NULL );

  xTaskCreate(
    TaskMessage
    ,  (const portCHAR *) "AnalogRead"
    ,  128 // This stack size can be checked & adjusted by reading Highwater ??
    ,  NULL
    ,  3  // priority
    ,  NULL );

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  (const portCHAR *)"Blink"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  3  // priority
    ,  NULL );


  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop() {
  // put your main code here, to run repeatedly:
}


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/



void TaskStimulate(void *pvParameters)
{
  (void) pvParameters;

  bool now_holds = digitalRead(holds);
  bool last_holds = now_holds;
  bool now_pushs = digitalRead(pushs);
  bool last_pushs = now_pushs;
  unsigned long last;

  for (;;)
  {
    // When button holds is relased, control stimulion by hand
    channelCheck();
    digitalWrite(holds_led, LOW);
    while ( digitalRead(holds) == LOW) {
      channelCheck();
      now_pushs = digitalRead(pushs);
      if (last_pushs != now_pushs) {
        if (now_pushs == HIGH) { // begin stimulation
          digitalWrite(pushs_led, HIGH);
          startStimulate();
        } else { // stop stimulation
          stopStimulate();
          digitalWrite( pushs_led, LOW);
        }
        last_pushs = now_pushs;
      }
    }

    // When button holds is pressed, run automatical program
    digitalWrite(holds_led, HIGH);
    while ( digitalRead(holds) == HIGH ) {
      channelCheck();
      now_holds = digitalRead(holds);
      //if(now_butB - last_butB == 1){
      if ( now_holds == HIGH && last_holds == LOW) {
        // main control flow
        for (int i = 0; i < cycle; i++  ) {
          // rest 30s
          last = millis();
          stopStimulate();
          while (millis() - last < rest_time) {
            now_holds = digitalRead(holds);
            if (now_holds == LOW) {
              break;
            }
          }
          // action 5s
          digitalWrite(holds, LOW);
          last = millis();
          startStimulate();
          while (millis() - last < action_time) {
            now_holds = digitalRead(holds);
            if (now_holds == LOW) {
              break;
            }
          }
          stopStimulate();
          digitalWrite(holds_led, HIGH);
          //last_butB = ; // reset but1 state to LOW
        }
      }

      last_holds = now_holds;
    }
    last_holds = LOW;
    digitalWrite(holds, LOW);
  }
}

//void TaskStimulate(void *pvParameters)
void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    //digitalWrite(hold2_led, HIGH);
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    //digitalWrite(hold2_led, LOW);
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskMessage(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    sensorValue = analogRead(pressure_port);
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
    Serial.print("Pressure: -");
    Serial.print(pressureValue);
    Serial.println("kPa");
    Serial.print("PWM Duty:  ");
    Serial.println(motor_pwm);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskVacuumCtrl(void *pvParameters)
{
  (void) pvParameters;
  pinMode(holdv, INPUT);
  pinMode(holdv_led, OUTPUT);
  pinMode(motor, OUTPUT);
  // PWM freq
  TCCR3B = TCCR2B & 0b11111000 | 0x01;  // change freq to 33k
  unsigned long last = millis();
  unsigned long last2;
  bool now_pushu = digitalRead(pushu);
  bool last_pushu = now_pushu;
  bool now_pushd = digitalRead(pushd);
  bool last_pushd = now_pushd;

  for (;;)
  {
    last = millis();
    now_pushu = digitalRead(pushu);
    while (now_pushu)
    {
      now_pushu = digitalRead(pushu);
      digitalWrite(pushu_led, HIGH);
      if (last_pushu == LOW && (millis() - last < 1000)) {
        if ((motor_pwm) < 255 && (motor_pwm > 120)) {
          motor_pwm ++;
        } else if (motor_pwm < 120) {
          motor_pwm = 200;
        } else {
          motor_pwm = 255;
        }
      }

      last2 = millis();
      while (last_pushu == HIGH && (millis() - last > 1100)) {
        if (millis() - last2 > 50) {
          if ((180 < motor_pwm) && (motor_pwm < 255)) {
            motor_pwm += 1;
          } else if ( motor_pwm < 180) {
            motor_pwm = 200;
          } else {
            motor_pwm = 255;
          }
          last2 = millis();
          analogWrite(motor, motor_pwm);
        }
        if (digitalRead(pushu) == LOW) {
          break;
        }
      }
      last_pushu = now_pushu;
    }
    digitalWrite(pushu_led, LOW);
    last_pushu = LOW;

    now_pushd = digitalRead(pushd);
    while (now_pushd)
    {
      now_pushd = digitalRead(pushd);
      digitalWrite(pushd_led, HIGH);
      if (last_pushd == LOW && (millis() - last < 1000)) {
        if (motor_pwm > 120) {
          motor_pwm --;
        }
      }

      last2 = millis();
      while (last_pushd == HIGH && (millis() - last > 1100)) {
        if (millis() - last2 > 50) {
          if (motor_pwm > 180) {
            --motor_pwm;
          } else {
            motor_pwm = 0;
          }
          last2 = millis();
          analogWrite(motor, motor_pwm);
        }
        if (digitalRead(pushd) == LOW) {
          break;
        }
      }
      last_pushd = now_pushd;
    }
    digitalWrite(pushd_led, LOW);
    last_pushd = LOW;
  }
}


void stopStimulate()
{
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
  digitalWrite(relay4, LOW);
}

void startStimulate()
{
  if (digitalRead(hold1))
  {
    digitalWrite(relay4, HIGH);
    digitalWrite(hold1_led, HIGH);
  } else {
    digitalWrite(relay4, LOW);
    digitalWrite(hold1_led, LOW);
  }

  if (digitalRead(hold2))
  {
    digitalWrite(relay3, HIGH);
    digitalWrite(hold2_led, HIGH);
  } else {
    digitalWrite(relay3, LOW);
    digitalWrite(hold2_led, LOW);
  }
  if (digitalRead(hold3))
  {
    digitalWrite(relay1, HIGH);
    digitalWrite(hold3_led, HIGH);
  } else {
    digitalWrite(relay1, LOW);
    digitalWrite(hold3_led, LOW);
  }
}

void channelCheck()
{
  if (digitalRead(hold1))
  {
    digitalWrite(hold1_led, HIGH);
  } else {
    digitalWrite(hold1_led, LOW);
  }

  if (digitalRead(hold2))
  {
    digitalWrite(hold2_led, HIGH);
  } else {
    digitalWrite(hold2_led, LOW);
  }
  if (digitalRead(hold3))
  {
    digitalWrite(hold3_led, HIGH);
  } else {
    digitalWrite(hold3_led, LOW);
  }
}

void HardwareInit()
{

  Serial.begin(9600);
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

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
  digitalWrite(relay4, LOW);

  pinMode(motor, OUTPUT);
  analogWrite(motor, 100);
}
