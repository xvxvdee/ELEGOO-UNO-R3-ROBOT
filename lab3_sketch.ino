#include <NewPing.h>

#define MAX_DISTANCE 5000

 // Line Following IR Sensors
#define left digitalRead(A4)
#define middle digitalRead(A5)
#define right digitalRead(2)

int STBY=3; // Standby Power System (must be high to enable Motor Control Board)

// Motors
int PWMAR=10; // Right motors A pin
int PWMBR=9; // Right motors B pin
int PWMAL=5; // Left motors A pin
int PWMBL=6; // Left motors B pin

// Turning Speed
int high = 70;
int turnHigh = 90;
int low = 0;

// Edge Detection IR Sensors
int IRR = 8;
int IRL = 12;

// Ultrasonic Sensors Trig and Echo Pins 
int trigPinF = 3; // Trig pin of Front Ultrasonic Sensor to pin 3 
int echoPinF = 4; // Echo pin of Front Ultrasonic Sensor to pin 4 
int echoPinFR = A0; // Trig pin of Front Right Ultrasonic Sensor to pin A0 
int trigPinFR = A1; // Echo pin of Front Right Ultrasonic Sensor to pin A1 
int echoPinFL = A2; // Trig pin of Front Left Ultrasonic Sensor to pin A2
int trigPinFL = A3; // Echo pin of Front Left Ultrasonic Sensor to pin A3 
long durationR, durationL, durationF, distanceF, durationFR, distanceFR, distanceFL, durationFL;

// Flags
bool edgeDetectionFlag = false;
bool objectDetectionFlag = false;
bool leftFlag = false;
bool rightFlag = false;
bool middleFlag = false;

// Object Detection Ultrasonic Sensors
NewPing sonarFL(trigPinFL, echoPinFL, MAX_DISTANCE);
NewPing sonarFR(trigPinFR, echoPinFR, MAX_DISTANCE);
NewPing sonarF(trigPinF, echoPinF, MAX_DISTANCE);

void setup () {
  Serial.begin(9600); // Initialize the serial communications

  pinMode(STBY, OUTPUT);
  pinMode(PWMAR, OUTPUT); 
  pinMode(PWMBR, OUTPUT); 
  pinMode (PWMAL, OUTPUT);
  pinMode (PWMBL, OUTPUT);

  // Adjust Speed
  analogWrite(PWMAR, high); 
  analogWrite(PWMBR, high);   
  analogWrite(PWMAL, high);
  analogWrite(PWMBL, high);

  // Initialize the Ultrasonic Sensor pins
  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(trigPinFR, OUTPUT);
  pinMode(echoPinFR, INPUT);
  pinMode(trigPinFL, OUTPUT);
  pinMode(echoPinFL, INPUT);

  // IR Sensors
  pinMode(IRR, INPUT);
  pinMode(IRL, INPUT);
}

void loop () {
  int offset = 1;
  unsigned long operationStartTime;
  unsigned long tempTime;

  // Edge Detection --------------------
  int IRRStatus = digitalRead(IRR);
  int IRLStatus = digitalRead(IRL);

  Serial.print("IR Right On--------:");
  Serial.println(IRRStatus);
  Serial.print("IR Left On--------:");
  Serial.println(IRLStatus);

  // Object Avoidance --------------------
  distanceFR = sonarFR.ping_cm();
  distanceFL = sonarFL.ping_cm();
  distanceF = sonarF.ping_cm();

  Serial.print("Distance Front ----: ");
  Serial.println(distanceF);  
  Serial.print("Distance Front Right----: ");
  Serial.println(distanceFR);  
  Serial.print("Distance Front Left----: ");
  Serial.println(distanceFL);  

  // Line Following --------------------
  Serial.print("Left On--------:");
  Serial.println(left);
  Serial.print("Middle On--------:");
  Serial.println(middle);
  Serial.print("Right On--------:");
  Serial.println(right);

  // Edge Detection --------------------
  if (IRRStatus == HIGH && IRLStatus == HIGH) {
    EdgeDetectionMode();
    Backward();
    delay(300);
    Right();
    delay(750);
  } else if (IRRStatus == HIGH) {
    EdgeDetectionMode();
    Backward();
    delay(300);
    Left();
    delay(300);
  } else if (IRLStatus == HIGH) {
    EdgeDetectionMode();
    Backward();
    delay(300);
    Right();
    delay(300);
  } 

  // Object Avoidance --------------------
  else if (distanceF <= 15 && distanceF != 0) { 
    ObjectAvoidanceMode();
    Right();
    delay(750);
  } 
  else if (distanceFR <= 13 && distanceFR != 0) {
    ObjectAvoidanceMode();
    Left();
    delay(300);
  } 
  else if (distanceFL <= 13 && distanceFL != 0) {
    ObjectAvoidanceMode();
    Right();
    delay(300);
  }

  // Line Following --------------------
  else if (middle && !right && !left) {
    Forward();
  }
  else if (middle && right && left || middle && left || left) {
    operationStartTime=millis();
    leftFlag = true;
  }
  else if (middle && right || right) {
    operationStartTime=millis();
    rightFlag = true;
  }
  // Forward --------------------
  else {
    Forward();
  }

  // Line Following Flags -------
  if (!objectDetectionFlag && !edgeDetectionFlag) {
    if (leftFlag){
      tempTime = millis();
      if (tempTime >= operationStartTime + offset) {
        if (digitalRead(A5)==LOW) { SlightLeft(); }
        else { leftFlag = false; }
      }
    }

    if (rightFlag) {
      tempTime = millis();
      if (tempTime >= operationStartTime + offset) {
        if (digitalRead(A5)==LOW) { SlightRight(); }
        else { rightFlag = false; }
      }
    }
  }
  
  // Resetting Flags ---------
  if (objectDetectionFlag) { objectDetectionFlag = false;}
  if (edgeDetectionFlag) { edgeDetectionFlag = false; }
}

void EdgeDetectionMode () {
  edgeDetectionFlag = true;
  objectDetectionFlag = false;
  leftFlag = rightFlag = false;
}

void ObjectAvoidanceMode () {
  objectDetectionFlag = true;
  leftFlag = rightFlag = false;
}

void Right () {
  analogWrite(PWMAR, turnHigh); // Right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, turnHigh);  // Left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void SlightRight () {
  analogWrite(PWMAR, low); // Right wheels stop
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, turnHigh);  // Left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void Left () {
  analogWrite(PWMAR, low); // Right wheels forward
  analogWrite(PWMBR, turnHigh); 
  analogWrite(PWMAL, low);  // Left wheels backwards
  analogWrite(PWMBL, turnHigh); 
  digitalWrite (STBY, HIGH);

}

void SlightLeft () {
  analogWrite(PWMAR, low); // Right wheels forward
  analogWrite(PWMBR, turnHigh); 
  analogWrite(PWMAL, low);  // Left wheels stop
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void Forward () {
  analogWrite(PWMAR, low); // Right wheels forward
  analogWrite(PWMBR, high); 
  analogWrite(PWMAL, high);  // Left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void Backward () {
  analogWrite(PWMAR, high); // Right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, low);  // Left wheels backwards
  analogWrite(PWMBL, high); 
  digitalWrite (STBY, HIGH);
}

void Stop () {
  analogWrite(PWMAR, low); // Right wheels stop
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, low);  // Left wheels stop
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}
