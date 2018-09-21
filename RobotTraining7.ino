#include <Boards.h>
#include <Firmata.h>
#include <FirmataConstants.h>
#include <FirmataDefines.h>
#include <FirmataMarshaller.h>
#include <FirmataParser.h>

#include <Servo.h>

//initialize servos 
Servo myservo1;
Servo myservo2;

//initialize ping sesnors
const int pingPin1 = 4; //left
const int pingPin2 = 2; //right
const int pingPin3 = 3; //front

//initialize IR
int sensorPin=A0;
int ledPin=13;
int val=0;

//initialize switch
int sw=0;

void setup() { 
  //initialize servos
  myservo1.attach(10); //left  
  myservo2.attach(11); //right

  //setup IR
  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);

  //initialize serial communication for ping
  Serial.begin(9600);
} 


//-----------------PING FUNCTIONS-------------------

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

int leftPing(){ // This function is for first sensor.
  int duration1, distance1;
  pinMode(pingPin1, OUTPUT);
  digitalWrite(pingPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin1, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin1, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin1, INPUT);
  duration1 = pulseIn(pingPin1, HIGH);

  // convert the time into a distance
  //inches = microsecondsToInches(duration);
  distance1 = microsecondsToCentimeters(duration1);
//  Serial.print("Left Sensor: ");
//  Serial.print(distance1); 
//  Serial.print("cm    ");
  return distance1;
}

int rightPing(){ 
  int duration2, distance2;
  pinMode(pingPin2, OUTPUT);
  digitalWrite(pingPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin2, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin2, LOW);
  pinMode(pingPin2, INPUT);
  duration2 = pulseIn(pingPin2, HIGH);
  distance2 = microsecondsToCentimeters(duration2);
//  Serial.print("Right Sensor: ");
//  Serial.print(distance2); 
//  Serial.print("cm    ");
  return distance2;
}

int frontPing(){ 
  int duration3, distance3;
  pinMode(pingPin3, OUTPUT);
  digitalWrite(pingPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin3, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin3, LOW);
  pinMode(pingPin3, INPUT);
  duration3 = pulseIn(pingPin3, HIGH);
  distance3 = microsecondsToCentimeters(duration3);
//  Serial.print("Right Sensor: ");
//  Serial.print(distance3); 
//  Serial.print("cm    ");
  return distance3;
}

//---------------------------------------------------




//-----------------SERVO FUNCTIONS-------------------

void forward() {
  myservo1.writeMicroseconds(1700);     
  myservo2.writeMicroseconds(1700);
}

void reverse() {
  myservo1.writeMicroseconds(1200);     
  myservo2.writeMicroseconds(1200);
}

void turnRight() {
  myservo1.writeMicroseconds(1720);     
  myservo2.writeMicroseconds(1200);
}

void turnLeft() {
  myservo1.writeMicroseconds(1200);     
  myservo2.writeMicroseconds(1720);
}

void stopRobot() {
  myservo1.writeMicroseconds(1470);     
  myservo2.writeMicroseconds(1470);
}

//---------------------------------------------------


//-----------------IR FUNCTIONS-------------------

void IRsensor(){
  tone(13, 40000);
  val=analogRead(sensorPin);
  Serial.println(val);
  if (val<800){
    Serial.print("Something is ahead.");
  }
  if (val>=800){
    Serial.print("No Object Detected.");
  }
  delay(1000);
}

//---------------------------------------------------



//-----------------PHOTO-RESISTOR FUNCTIONS-------------------

float photoResistor1() {
  // read the input on analog pin 0:
  int sensorValue1 = analogRead(A4); //left side
  //int sensorValue2 = analogRead(A5); //right side
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage1 = sensorValue1* (5.0/1023.0);
  //float voltage2 = sensorValue2 * (5.0 / 1023.0);
  // print out the value you read:
  float voltage3=5-voltage1; //left
  //float voltage4=5-voltage2; //right
  Serial.println("left photoResistor: ");
  Serial.println(voltage3);
  //Serial.println(voltage4);
  //Serial.println("\n");
  delay(10);
  return voltage3;
}

float photoResistor2() {
  int sensorValue2 = analogRead(A5); //right side
  float voltage2 = sensorValue2 * (5.0 / 1023.0);
  float voltage4=5-voltage2; //right
  Serial.println("right photoResistor: ");
  Serial.println(voltage4);
  //Serial.println("\n");
  delay(10);
  return voltage4;
}

/*void findLight() {
  if (photoResistor1()>=4.8 && photoResistor2()>=4.8) { //find dark room
    //forward();
    delay(2000);
    while (photoResistor1()>=4.8 || photoResistor2()>=4.8){
      turnRight();
    }
    forward();
    delay(2000);
  } 
}*/

void darkRoom() {
  forward();
  delay(500);
  if ((photoResistor1()-0.1) < photoResistor2()) {
    stopRobot();
    delay(500);
    turnLeft();
    delay(200);
  }
  if (photoResistor2() < (photoResistor1()-0.1)) {
    stopRobot();
    delay(500);
    turnRight();
    delay(200);
  }
}

void hallway() {
  //forward();
  //delay(1000);
  /*if (frontPing()<20) {
    turnRight();
    delay(200);
  }*/
  if (photoResistor1()>=2.5 && photoResistor2()>2.3) {
    sw = 1;
    stopRobot();
    //forward();
    delay(1000);
  }
  if (leftPing()<20) {
    turnRight();
    delay(200);
  }
  if (rightPing()<20) {
    turnLeft();
    delay(200);
  }
  //stopRobot();
  //delay(100);
  
}

//---------------------------------------------------



void loop() { 

  //forward();
  //delay(2000);
  //========== PING CODE ============ 
  Serial.println("\n");
  /*while (sw == 0) {
    hallway();
  }*/
  darkRoom();
  /*else if (photoResistor1()>=4.6) {  //left
    turnLeft();
    delay(200);
  }
  else if (photoResistor2()>=4.6) {  //right
    turnRight();
    delay(200);
  }*/

  //=================================


//  forward();             
//  delay(2000);           
//  stopRobot();
//  delay(1000);
//  turnRight();
//  delay(570);
//  stopRobot();             
//  delay(2000);
//  stopRobot();
//  delay(2000);
//  turnLeft();
//  delay(570);
//  reverse();             
//  delay(1000); 
} 
