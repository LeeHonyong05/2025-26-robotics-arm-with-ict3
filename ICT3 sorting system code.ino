#include <Servo.h>
unsigned long curr_time = 0;
unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long time3 = 0;
unsigned long time4 = 0;
unsigned long time5 = 0;
unsigned long time6 = 0;
const int delay1 = 2000;
const int delay2 = 3500;
const int delay3 = 900;
const int delay4 = 500;
const int delay5 = 1000; //assem no
const int delay6 = 2500;
int on1 = 0;
int on3 = 0;
int on4 = 0;
int on5 = 0;
int on6 = 0;
int ring = 0;
int sort1 = 0;
int sort2 = 0;
int onstate = 0;

int s0 =28; // USE PIN 28, IF USE OTHERS PIN WILL DELAY
int s2 = 34; 
int s4 = 37; 
int s6 = 38; 
int s8 = 41; 
int s10 = 42;

int s1 = 46;
int s3 = 24;
int s5 = 50;
int s7 = 31;
int s9 = 45;

int o0 = 7;
int o1 = 9;
int o2 = 5;
int o3 = 11;
int o4 = 2;
char signal =0;

Servo myservo;

void setup() {
  Serial.begin(9600);
  pinMode(s0, INPUT);
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  pinMode(s6, INPUT);
  pinMode(s7, INPUT);
  pinMode(s8, INPUT);
  pinMode(s9, INPUT);
  pinMode(s10, INPUT);
  pinMode(o0, OUTPUT);
  pinMode(o1, OUTPUT);
  pinMode(o2, OUTPUT);
  pinMode(o3, OUTPUT);
  pinMode(o4, OUTPUT);
  myservo.attach(12);
}

void loop() {
if(Serial.available()>0){
signal = Serial.read();
}
  if(signal == '1'){
    myservo.write(5);

  }else if(signal == '0'){
    myservo.write(60);
  }else if(signal == '2'){
    myservo.write(70);
  }
//on state 
if(digitalRead(s6) == LOW && digitalRead(s7) == LOW){
onstate = 1;
//digitalWrite(o0,HIGH);
}
if(digitalRead(s6) == HIGH && digitalRead(s7) == HIGH){
onstate = 0;
//digitalWrite(o0,LOW);
}

if(onstate == 1){
//coveyer chain
digitalWrite(o0 , HIGH);
digitalWrite(o4 , HIGH);

//timer
  if(millis() - curr_time >= 100){
   curr_time = millis();
  }

//upper part
if(digitalRead(s1) == LOW && digitalRead(s0) == HIGH && ring < 5 && curr_time - time1 > delay1){
analogWrite(o1 , 255);
on1 = 1;
}
if(digitalRead(s9) == LOW && on1 == 1){
time1 = curr_time;
ring += 1;
on1 = 0;
}
if(digitalRead(s0) == LOW){
time1 = curr_time;
}
if( curr_time - time1 >= delay1/2){
analogWrite(o1 , 0);
}

//ring part
if( curr_time - time4 >= delay4 && on3 == 1){
analogWrite(o2 , 0);
on3 = 0;
}
if(digitalRead(s2) == HIGH && curr_time - time2 >= delay2 && ring > 0){
analogWrite(o2 , 255);
ring -= 1;
Serial.println(ring);
time4 = curr_time;
on3 = 1;
}
if(digitalRead(s9) == LOW && curr_time - time2 >= delay2){
time2 = curr_time;
}

//assem part(assem=yes)
if(digitalRead(s3) == HIGH && digitalRead(s8 ) == LOW && sort2 == 0){
  Serial.println("assem in");
sort2 = 1;
time6 = curr_time;
on6 = 1;
}
if(curr_time   - time6 >= delay6 && on6 == 1){
sort1 = sort2;
sort2 = 0;
on6 = 0;
 
}
if(digitalRead(s5) == LOW && sort1 == 1 && on4 == 0){
time3 = curr_time;
on4 = 1;
Serial.println("assem out");
}
if(curr_time   - time3 >= delay3 && on4 == 1){
on4 = 0;
sort1 = 0;
}

//assem part(assem=no)
if(digitalRead(s5) == LOW && sort1 == 0 && on5 == 0){
analogWrite(o3 , 255);
Serial.println("executed");
time5 = curr_time;
on5 = 1;
}
if(curr_time - time5 >= delay5 && on5 == 1){
analogWrite(o3 , 0);
on5 = 0;
}

}else{//off state
digitalWrite(o0 , LOW); //coveyer chain
digitalWrite(o4 , LOW); //coveyer chain
}
}
