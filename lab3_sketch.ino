#include <NewPing.h>
#include <PCF8574.h>

#define MAX_DISTANCE 5000

// Line Following Pins
#define leftIRPin 7
#define middleIRPin 13
#define rightIRPin 2

 // Line Following IR Sensors
#define leftLine digitalRead(leftIRPin) //red P2 7
#define middleLine digitalRead(middleIRPin) //orange P1 13 NOW GREEN
#define rightLine digitalRead(rightIRPin)

int STBY=3; // Standby Power System (must be high to enable Motor Control Board)

// Motors
int PWMAR=10; // Right motors A pin
int PWMBR=9; // Right motors B pin
int PWMAL=5; // Left motors A pin
int PWMBL=6; // Left motors B pin

// Turning Speed
int high = 115;
int turnHigh = 130;
int turnLow = 110;
int low = 0;

// Edge Detection IR Sensors
int IRR = 8;
int IRL = 12;

// Flame IR Sensors
uint8_t FIRR = P1;
uint8_t FIRL = P2;
uint8_t FIRM = P3;

PCF8574 pcf8574(0x20);

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
bool lineDetectionFlag = false;
bool fireDetectionFlag = false;
bool leftLineFlag = false;
bool rightLineFlag = false;
bool middleLineFlag = false;
bool leftFireFlag = false;
bool rightFireFlag = false;
bool middleFireFlag = false;

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
  pcf8574.pinMode(FIRL, INPUT);
  pcf8574.pinMode(FIRR, INPUT);
  pcf8574.pinMode(FIRM, INPUT);

  // initialize IO Extender
  if (pcf8574.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }

}

void loop () {
  int offset = 1;
  int maxLineTurnDuration = 3000;
  unsigned long operationStartTime;
  unsigned long tempTime;

  // Edge Detection --------------------
  int IRRStatus = digitalRead(IRR);
  int IRLStatus = digitalRead(IRL);

  // Serial.print("IR Right On--------:");
  // Serial.println(IRRStatus);
  // Serial.print("IR Left On--------:");
  // Serial.println(IRLStatus);

  // Flame Detection ---------------------
  int FIRLStatus = pcf8574.digitalRead(FIRL);
  int FIRRStatus = pcf8574.digitalRead(FIRR);
  int FIRMStatus = pcf8574.digitalRead(FIRM);

  Serial.print("IR FIRE LEFT On--------:");
  Serial.println(FIRLStatus);
  Serial.print("IR FIRE RIGHT On--------:");
  Serial.println(FIRRStatus);
  Serial.print("IR FIRE MIDDLE On--------:");
  Serial.println(FIRMStatus);

  // PCF8574::DigitalInput di = pcf8574.digitalReadAll();
	// Serial.print(di.p0);
	// Serial.print(" - ");
	// Serial.print(di.p1);
	// Serial.print(" - ");
	// Serial.println(di.p2);
	// Serial.print(" - ");
	// Serial.println(di.p3);

  // Object Avoidance --------------------
  distanceFR = sonarFR.ping_cm();
  distanceFL = sonarFL.ping_cm();
  distanceF = sonarF.ping_cm();

  // Serial.print("Distance Front ----: ");
  // Serial.println(distanceF);  
  // Serial.print("Distance Front Right----: ");
  // Serial.println(distanceFR);  
  // Serial.print("Distance Front Left----: ");
  // Serial.println(distanceFL);  

  // Line Following --------------------
  // Serial.print("Left On--------:");
  // Serial.println(leftLine);
  // Serial.print("Middle On--------:");
  // Serial.println(middleLine);
  // Serial.print("Right On--------:");
  // Serial.println(rightLine);

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

  // Flame Detection LOW = detected -----------------
 // else if (FIRMStatus == LOW) {
  else if (!FIRMStatus) {
//    Serial.print("IR FIRE MIDDLE RIGHT On--------:");
    operationStartTime=millis();
    FireDetectionMode();
    middleFireFlag = true;
    // stop until the flame goes out
  }
  else if (FIRRStatus == LOW) {
    operationStartTime=millis();
    FireDetectionMode();
    rightFireFlag = true;
  }
  else if (FIRLStatus == LOW) {
    operationStartTime=millis();
    FireDetectionMode();
    leftFireFlag = true;
  }

  // Object Avoidance --------------------
  else if (distanceF <= 10 && distanceF != 0) { 
    ObjectAvoidanceMode();
    Right();
    delay(750);
  } 
  else if (distanceFR <= 10 && distanceFR != 0) {
    ObjectAvoidanceMode();
    Left();
    delay(300);
  } 
  else if (distanceFL <= 10 && distanceFL != 0) {
    ObjectAvoidanceMode();
    Right();
    delay(300);
  }

  // Line Following --------------------
  else if (middleLine && !rightLine && !leftLine) {
    Forward();
  }
  else if (middleLine && rightLine && leftLine || middleLine && leftLine || leftLine) {
    operationStartTime=millis();
    leftLineFlag = true;
  }
  else if (middleLine && rightLine || rightLine) {
    operationStartTime=millis();
    rightLineFlag = true;
  }

  // Forward --------------------
  else {
    Forward();
  }

  // Serial.print("object detection--------:");
  // Serial.println(objectDetectionFlag);
  Serial.print("fire detection--------:");
  Serial.println(fireDetectionFlag);
  // Serial.print("line detection right--------:");
  // Serial.println(rightLineFlag);
  // Serial.print("line detection left--------:");
  // Serial.println(leftLineFlag);
  Serial.print("IR FIRE MIDDLE On--------:");
  Serial.println(FIRMStatus);
  // Serial.print("IR FIRE MIDDLE Flag On--------:");
  // Serial.println(middleFireFlag);

  Serial.print("IR FIRE LEFT On--------:");
  Serial.println(FIRLStatus);
  // Serial.print("IR FIRE LEFT Flag On--------:");
  // Serial.println(leftFireFlag);

  Serial.print("IR FIRE RIGHT On--------:");
  Serial.println(FIRRStatus);
  // Serial.print("IR FIRE RIGHT Flag On--------:");
  // Serial.println(rightFireFlag);

  // Flame Detection Flags
  if (!objectDetectionFlag && !edgeDetectionFlag && fireDetectionFlag) {

    if (middleFireFlag){
      tempTime = millis();
      if (tempTime >= operationStartTime + offset) {
        while (pcf8574.digitalRead(FIRM)==LOW) { 
          
          Stop(); }
        //else { 
          middleFireFlag = false; 
          fireDetectionFlag = false;
            Serial.print("IR FIRE FLAG ENTER HERE--------:");

        //}
      }
    }

    if (leftFireFlag){
      tempTime = millis();
      if (tempTime >= operationStartTime + offset) {
        if (pcf8574.digitalRead(FIRM)==HIGH) { FlameLeft(); }
        else { 
          leftFireFlag = false; 
          fireDetectionFlag = false;
        }
      }
    }

    if (rightFireFlag){
      tempTime = millis();
      if (tempTime >= operationStartTime + offset) {
        if (pcf8574.digitalRead(FIRM)==HIGH) { FlameRight(); }
        else { 
          rightFireFlag = false; 
          fireDetectionFlag = false;
        }
      }
    }
  }

  // Line Following Flags (Turn until middle) -------
  if (!objectDetectionFlag && !edgeDetectionFlag && !fireDetectionFlag) {
    if (leftLineFlag){
      tempTime = millis();
      if (tempTime >= operationStartTime + offset) {
        if (digitalRead(middleIRPin)==LOW && tempTime <= operationStartTime + maxLineTurnDuration) { SlightLeft(); }
        else { leftLineFlag = false; }
      }
    }

    if (rightLineFlag) {
      tempTime = millis();
      if (tempTime >= operationStartTime + offset) {
        if (digitalRead(middleIRPin)==LOW && tempTime <= operationStartTime + maxLineTurnDuration) { SlightRight(); }
        else { rightLineFlag = false; }
      }
    }
  }
  
  // Resetting Flags ---------
  if (objectDetectionFlag) { objectDetectionFlag = false;}
  if (edgeDetectionFlag) { edgeDetectionFlag = false; }
  // if (fireDetectionFlag) { fireDetectionFlag = false; }
}

void EdgeDetectionMode () {
  edgeDetectionFlag = true;
  objectDetectionFlag = false;
  leftLineFlag = false;
  rightLineFlag = false;
}

void ObjectAvoidanceMode () {
  objectDetectionFlag = true;
  leftLineFlag = false;
  rightLineFlag = false;
  // why no middle line flag
}

void FireDetectionMode () {
  objectDetectionFlag = false;
  fireDetectionFlag = true;
  // lineDetectionFlag = false;   CAN WE DO THIS???
  leftLineFlag = false;
  rightLineFlag = false;
}

void LineDetectionMode () {
  lineDetectionFlag = true;
}


void Right () {
  analogWrite(PWMAR, turnHigh); // Right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, turnHigh);  // Left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void FlameRight () {
  analogWrite(PWMAR, turnLow); // Right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, turnLow);  // Left wheels forward
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

void FlameLeft () {
  analogWrite(PWMAR, low); // Right wheels forward
  analogWrite(PWMBR, turnLow); 
  analogWrite(PWMAL, low);  // Left wheels backwards
  analogWrite(PWMBL, turnLow); 
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
