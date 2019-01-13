/* Control program for the olfacotry chip
 *  Version  |  Commit 
 *    0.1    | First version
 *    
 */
// Relay 1, always close, dose
const int relay1 = 2;
// Realy 2, always open, buffer
const int relay2 = 5;
// Button 1
const int buttonA = 4;
// Button 2
const int buttonB = 7;
// LED 1
const int ledA = 3;  // auto recover
// LED B;
const int ledB = 11;  // lock button
int now_butA = LOW;
int last_butA = LOW;
int now_butB = LOW;
int last_butB = LOW;
unsigned long last;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, LOW);
 
  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, LOW);
  
  pinMode(buttonB, INPUT);

  pinMode(buttonA, INPUT);
  
  pinMode(ledA, OUTPUT);
  digitalWrite(ledA, LOW);
 
  pinMode(ledB, OUTPUT);
  digitalWrite(ledB, LOW);  

  now_butA = digitalRead(buttonB);
  last_butA = now_butA;
  now_butB = digitalRead(buttonA); 
  last_butB = now_butB;
}

void loop() {
  // When button A is relased, control stimulion by hand
  digitalWrite(ledA, LOW);
  while( digitalRead(buttonB) == LOW){
    now_butA = digitalRead(buttonA);
    if(last_butA != now_butA){
      if(now_butA == HIGH){ // begin stimulation
          start_stimulate();
       }else{ // stop stimulation
          stop_stimulate();
        }
      last_butA = now_butA;
     }
   }
  
  // When button B is pressed, run automatical program
  digitalWrite(ledB, HIGH);
  while( digitalRead(buttonB) == HIGH ){  
    now_butB = digitalRead(buttonB);
    //if(now_butB - last_butB == 1){
    if( now_butB == HIGH && last_butB == LOW){
        // rest 4s
        last = millis();
        stop_stimulate();
        while(millis() - last < 4000){
            now_butB = digitalRead(buttonB);
            if(now_butB == LOW){
              break;
            }
        }
        // action 1s
        digitalWrite(ledB, LOW);
        last = millis();
        start_stimulate();
        while(millis() - last < 1000){
            now_butB = digitalRead(buttonB);
            if(now_butB == LOW){
              break;
            }
        }
        stop_stimulate();
        digitalWrite(ledB, HIGH);
        //last_butB = ; // reset but1 state to LOW
      }
    last_butB = now_butB;
   }
   last_butB = LOW;
   digitalWrite(ledB, LOW);
}

void start_stimulate(){
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    digitalWrite(ledA, HIGH);

}

void stop_stimulate(){
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    digitalWrite(ledA, LOW);
}
