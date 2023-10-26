#include <NewPing.h>

#define MAX_DISTANCE 5000
#define left digitalRead(A4)
#define middle digitalRead(A5)
#define right digitalRead(2)
// #define IRR digitalRead(8)

int STBY=3; //standby power system and it must be high for the motor control board to be enabled
int BIN1=8; //high for forward movement and low for backward movement
// int AIN1=7; // high for forward movement and low for backward movement
int IRR = 8;
int IRL = 12;

int PWMAR=10; // right motors A pin
int PWMBR=9; // right motors B pin
int PWMAL=5; // left motors A pin
int PWMBL=6; // left motors B pin
int high = 72;
int turnHigh = 90;
int low = 0;
// int trigPinL = 13; // Connect the Trig pin of the left Ultrasonic Sensor to pin 9 on Arduino
// int echoPinL = 12; // Connect the Echo pin of the left Ultrasonic Sensor to pin 10 on Arduino
// int trigPinR = 8; // Connect the Trig pin of the right Ultrasonic Sensor to pin 8 on Arduino
// int echoPinR = 7; // Connect the Echo pin of the right Ultrasonic Sensor to pin 7 on Arduino
int trigPinF = 3; // Connect the Trig pin of the front-facing Ultrasonic Sensor to pin 3 on Arduino
int echoPinF = 4; // Connect the Echo pin of the front-facing Ultrasonic Sensor to pin 4 on Arduino
int echoPinFR = A0; // Connect the Trig pin of the right front-facing Ultrasonic Sensor to pin A0 on Arduino
int trigPinFR = A1; // Connect the Echo pin of the right front-facing Ultrasonic Sensor to pin A1 on Arduino
int echoPinFL = A2; // Connect the Trig pin of the left front-facing Ultrasonic Sensor to pin A2 on Arduino
int trigPinFL = A3; // Connect the Echo pin of the left front-facing Ultrasonic Sensor to pin A3 on Arduino
// int echoPinB = A5; // Connect the Echo pin of the left front-facing Ultrasonic Sensor to pin A5 on Arduino
// int trigPinB = A4; // Connect the Trig pin of the left front-facing Ultrasonic Sensor to pin A4 on Arduino


// int left = A4;
// int right = A5;
// int middle = 2;
  long durationR, durationL, durationF, distanceF, durationFR, distanceFR, distanceFL, durationFL, durationB, distanceB;
  unsigned int distanceR, distanceL;
// NewPing sonarL(trigPinL, echoPinL, MAX_DISTANCE);
// NewPing sonarR(trigPinR, echoPinR, MAX_DISTANCE);
NewPing sonarFL(trigPinFL, echoPinFL, MAX_DISTANCE);
NewPing sonarFR(trigPinFR, echoPinFR, MAX_DISTANCE);
NewPing sonarF(trigPinF, echoPinF, MAX_DISTANCE);
// NewPing sonarB(trigPinB, echoPinB, MAX_DISTANCE);

// NewPing sonar(trigPinF, echoPinF, MAX_DISTANCE);
// NewPing sonar[2] = {     // Sensor object array.
//   NewPing(trigPinL, echoPinL, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
//   NewPing(trigPinR, echoPinR, MAX_DISTANCE),
// }


void setup () {
  Serial.begin(9600);   // initialize the serial communications:

  // pinMode (AIN1, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (STBY, OUTPUT);
  pinMode(PWMAR, OUTPUT); 
  pinMode(PWMBR, OUTPUT); 
  pinMode (PWMAL, OUTPUT);
  pinMode (PWMBL, OUTPUT);

  //Adjust Speed
  analogWrite(PWMAR, high); 
  analogWrite(PWMBR, high);   
  analogWrite(PWMAL, high);
  analogWrite(PWMBL, high);

  // Initialize the Ultrasonic Sensor pins
  // pinMode(trigPinL, OUTPUT);
  // pinMode(echoPinL, INPUT);
  // pinMode(trigPinR, OUTPUT);
  // pinMode(echoPinR, INPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(trigPinFR, OUTPUT);
  pinMode(echoPinFR, INPUT);
  pinMode(trigPinFL, OUTPUT);
  pinMode(echoPinFL, INPUT);
  // pinMode(trigPinB, OUTPUT);
  // pinMode(echoPinB, INPUT);

  pinMode(IRR, INPUT);
  pinMode(IRL, INPUT);

  int delayTime = 200 ;
}

void loop () {
  delay(50);
  // Add this code where you want to read from the sensor
  // distanceR = sonarR.ping_cm();
  // distanceL = sonarL.ping_cm();
  distanceFR = sonarFR.ping_cm();
  distanceFL = sonarFL.ping_cm();
  distanceF = sonarF.ping_cm();

  int IRRStatus = digitalRead(IRR);
  int IRLStatus = digitalRead(IRL);
  // distanceB = sonarB.ping_cm();

  // distanceR = findDistance(trigPinR, echoPinR);
  // distanceL = findDistance(trigPinL, echoPinL);
  // distanceF = findDistance(trigPinF, echoPinF);
  // distanceFR = findDistance(trigPinFR, echoPinFR);
  // distanceFL = findDistance(trigPinFL, echoPinFL);
  // distanceB = findDistance(trigPinB, echoPinB);
  
  // Serial.print("Distance Right--: ");
  // Serial.println(distanceR);  // Tools>Serial Monitor
  // Serial.print("Distance Left----: ");
  // Serial.println(distanceL);  // Tools>Serial Monitor
  // Serial.print("Distance Front----: ");
  // Serial.println(distanceF);  // Tools>Serial Monitor
  // Serial.print("Distance Front Right----: ");
  // Serial.println(distanceFR);  // Tools>Serial Monitor
  // Serial.print("Distance Front Left----: ");
  // Serial.println(distanceFL);  // Tools>Serial Monitor
  // Serial.print("Distance Back----: ");
  // Serial.println(distanceB);  // Tools>Serial Monitor
  // Serial.print("Left On--------:");
  // Serial.println(left);
  // Serial.print("Middle On--------:");
  // Serial.println(middle);
  // Serial.print("Right On--------:");
  // Serial.println(right);
  Serial.print("IR Right On--------:");
  Serial.println(IRRStatus);
  Serial.print("IR Left On--------:");
  Serial.println(IRLStatus);


  if (IRRStatus == HIGH && IRLStatus == HIGH) {
    Backward();
    delay(300);
    Right();
    delay(750);
  } else if (IRRStatus == HIGH) {
    Backward();
    delay(300);
    Left();
    delay(300);
  } else if (IRLStatus == HIGH) {
    Backward();
    delay(300);
    Right();
    delay(300);
  // // } else if (distanceB > 80 && distanceL < 40 && distanceR < 40) {
  // //   Forward();
  // //   delay(300);
  // } else if (distanceF <= 20 && distanceF != 0) { // 30 might be too much
  //   // Backward();
  //   // delay(300);
  //   Right();
  //   delay(750);
  // } else if (distanceFR <= 18 && distanceFR != 0) {
  //   // Backward();
  //   // delay(300);
  //   Left();
  //   delay(300);
  // } else if (distanceFL <= 18 && distanceFL != 0) {
  //   // Backward();
  //   // delay(300);
  //   Right();
  //   delay(300);
  // } else if((distanceB < 15 || distanceB > 80) && distanceFR <= 15 ){
  //   Forward();
  //   delay(300);
  //   Left();
  //   delay(750);
  // } else if((distanceB < 15 || distanceB > 80) && distanceFL <= 15 ){
  //   Forward();
  //   delay(300);
  //   Right();
  //   delay(750);
  // }
  // else if((distanceB < 3 || distanceB > 80) && distanceF <= 15 ){
  //   // Forward();
  //   // delay(300);
  //   Right();
  //   delay(750);
  }
  else if(left) {
    SlightLeft();
    delay(25);
  }
  else if (right) {
    SlightRight();
    delay(25);
  }
  else if (middle && left && !right) {
    SlightLeft();
    delay(10);
  }
  else if (middle && !left && right) {
    SlightRight();
    delay(10);
  }
  else if (middle && left && right) {
    Stop();
    delay(750);
  }
  else if (middle && !left && !right) {
    Forward();
  }
  else {
    Forward();
  }

}

void Right () {
  analogWrite(PWMAR, turnHigh); // right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, turnHigh);  // left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void Left () {
  analogWrite(PWMAR, low); // right wheels forward
  analogWrite(PWMBR, turnHigh); 
  analogWrite(PWMAL, low);  // left wheels backwards
  analogWrite(PWMBL, turnHigh); 
  digitalWrite (STBY, HIGH);

}

void Forward () {
  analogWrite(PWMAR, low); // right wheels forward
  analogWrite(PWMBR, high); 
  analogWrite(PWMAL, high);  // left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void Backward () {
  analogWrite(PWMAR, high); // right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, low);  // left wheels backwards
  analogWrite(PWMBL, high); 
  digitalWrite (STBY, HIGH);
}

void Stop () {
  analogWrite(PWMAR, low); // right wheels stop
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, low);  // left wheels stop
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void SlightRight () {
  analogWrite(PWMAR, low); // right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, turnHigh);  // left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void SlightLeft () {
  analogWrite(PWMAR, low); // right wheels forward
  analogWrite(PWMBR, turnHigh); 
  analogWrite(PWMAL, low);  // left wheels backwards
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);

}

long findDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return ((duration/2) / 29.1);
}

// long findDistance2() {
//   sonar1.ping_cm()
// }
