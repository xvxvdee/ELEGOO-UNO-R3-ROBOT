#include <NewPing.h>
#include <PCF8574.h>

#define MAX_DISTANCE 5000
int MAX_LINE_TURN_DURATION = 3000;
int OFFSET = 1;

// AMR ---------------------------------------------------
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

// PINS -------------------------------------------------------------------------
// Line Following - - - - - - - - - - - - - -
#define PIN_lineL 7
#define PIN_lineF 13
#define PIN_lineR 2
// //IR Sensors  
// #define LINE_outputL 
// #define LINE_outputF
// #define LINE_outputR

// Flame IR Sensors - - - - - - - - - - - - - -
uint8_t PIN_EXT_flameR = P1;
uint8_t PIN_EXT_flameL = P2;
uint8_t PIN_EXT_flameF = P3;

// Flame Extinguisher Fan - - - - - - - - - - - -
uint8_t PIN_EXT_fan = P4;

// IO Extender - - - - - - - - - - - - - - - - - 
PCF8574 pcf8574(0x20);

// Ultrasonic Sensors Trig and Echo Pins - - - - 
int PIN_TRIG_objectF = 3; // Trig pin of Front Ultrasonic Sensor to pin 3 
int PIN_ECHO_objectF = 4; // Echo pin of Front Ultrasonic Sensor to pin 4 
int PIN_TRIG_objectFR = A1; // Echo pin of Front Right Ultrasonic Sensor to pin A1 
int PIN_ECHO_objectFR = A0; // Trig pin of Front Right Ultrasonic Sensor to pin A0 
int PIN_TRIG_objectFL = A3; // Echo pin of Front Left Ultrasonic Sensor to pin A3 
int PIN_ECHO_objectFL = A2; // Trig pin of Front Left Ultrasonic Sensor to pin A2
long OBJECT_outputF, OBJECT_outputR, OBJECT_outputL; //Ultrasonic sensors

// Flags ------------------------------------------------------------------------
bool FLAG_objectDetection = false;
bool FLAG_objectL = false;
bool FLAG_objectR = false;
bool FLAG_objectF = false;

bool FLAG_fireDetection = false;
bool FLAG_fireL = false;
bool FLAG_fireR = false;
bool FLAG_fireF = false;

bool FLAG_lineDetection = false; // NOT used
bool FLAG_lineL = false;
bool FLAG_lineR = false;
bool FLAG_lineF = false;

// Object Detection Ultrasonic Sensors
NewPing OBJECT_sonarL(PIN_TRIG_objectFL, PIN_ECHO_objectFL, MAX_DISTANCE);
NewPing OBJECT_sonarR(PIN_TRIG_objectFR, PIN_ECHO_objectFR, MAX_DISTANCE);
NewPing OBJECT_sonarF(PIN_TRIG_objectF, PIN_ECHO_objectF, MAX_DISTANCE);

void setup () {
  Serial.begin(9600); // Initialize the serial communications

  pinMode(STBY, OUTPUT);
  pinMode(PWMAR, OUTPUT); 
  pinMode(PWMBR, OUTPUT); 
  pinMode(PWMAL, OUTPUT);
  pinMode(PWMBL, OUTPUT);

  // Adjust Speed
  analogWrite(PWMAR, high); 
  analogWrite(PWMBR, high);   
  analogWrite(PWMAL, high);
  analogWrite(PWMBL, high);

  // Initialize the Ultrasonic Sensor pins --------------
  pinMode(PIN_TRIG_objectF, OUTPUT);
  pinMode(PIN_ECHO_objectF, INPUT);
  pinMode(PIN_TRIG_objectFR, OUTPUT);
  pinMode(PIN_ECHO_objectFR, INPUT);
  pinMode(PIN_TRIG_objectFL, OUTPUT);
  pinMode(PIN_ECHO_objectFL, INPUT);

  // Flame IR Sensors - - - - - - - -
  pcf8574.pinMode(PIN_EXT_flameL, INPUT);
  pcf8574.pinMode(PIN_EXT_flameR, INPUT);
  pcf8574.pinMode(PIN_EXT_flameF, INPUT);

  // Flame Fan - - - - - - - - - - - - 
  pcf8574.pinMode(PIN_EXT_fan, OUTPUT);
  
  // Initialize IO Extender
  if (pcf8574.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }

  pcf8574.digitalWrite(PIN_EXT_fan, LOW);
}

void loop () {
  unsigned long operationStartTime;
  unsigned long tempTime;

  // Flame Detection Sensors ---------------------
  int FIRE_outputL = pcf8574.digitalRead(PIN_EXT_flameL);
  int FIRE_outputR = pcf8574.digitalRead(PIN_EXT_flameR);
  int FIRE_outputF = pcf8574.digitalRead(PIN_EXT_flameF);
  // FireSensorOutputs(FIRE_outputR,FIRE_outputL,FIRE_outputF);

  // Object Avoidance Sensors --------------------
  OBJECT_outputR = OBJECT_sonarR.ping_cm();
  OBJECT_outputL = OBJECT_sonarL.ping_cm();
  OBJECT_outputF = OBJECT_sonarF.ping_cm();
  // Serial.print("Distance Front ----: ");
  // Serial.println(OBJECT_outputF);  
  // Serial.print("Distance Front Right----: ");
  // Serial.println(OBJECT_outputR);  
  // Serial.print("Distance Front Left----: ");
  // Serial.println(OBJECT_outputL);  
  // ObjectSensorOutputs(OBJECT_outputR,OBJECT_outputL,OBJECT_outputF);
  
  // Line Following --------------------
  int LINE_outputL = digitalRead(PIN_lineL); //red P2 7
  int LINE_outputF = digitalRead(PIN_lineF); //orange P1 13 NOW GREEN
  int LINE_outputR = digitalRead(PIN_lineR);
  // LineSensorOutputs(LINE_outputR,LINE_outputL,LINE_outputF);

  // Object Avoidance ----------------------------------------
  if (OBJECT_outputF <= 10 && OBJECT_outputF != 0) { 
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectF=true;
  } 
  else if (OBJECT_outputR <= 10 && OBJECT_outputR != 0) {
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectR=true;
  } 
  else if (OBJECT_outputL <= 10 && OBJECT_outputL != 0) {
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectL=true;
  }

  // Flame Detection (LOW = detected) -------------------------
  else if (pcf8574.digitalRead(PIN_EXT_flameF) == LOW) { 
    operationStartTime=millis();
    FireDetectionMode();
    FLAG_fireF = true;
  }  else if (pcf8574.digitalRead(PIN_EXT_flameR) == LOW) {
    operationStartTime=millis();
    FireDetectionMode();
    FLAG_fireR = true;
  }  else if (pcf8574.digitalRead(PIN_EXT_flameL) == LOW) {
    operationStartTime=millis();
    FireDetectionMode();
    FLAG_fireL = true;
  }

  // Line Following --------------------
  else if (LINE_outputF && !LINE_outputR && !LINE_outputL) {
    Forward();
  }
  else if (LINE_outputF && LINE_outputR && LINE_outputL || LINE_outputF && LINE_outputL || LINE_outputL) {
    operationStartTime=millis();
    FLAG_lineL = true;
  }
  else if (LINE_outputF && LINE_outputR || LINE_outputR) {
    operationStartTime=millis();
    FLAG_lineR = true;
  }

  FlagOutputs(FLAG_objectDetection,FLAG_fireDetection,FLAG_lineDetection,FLAG_lineL,FLAG_lineR);

  // Object Detection Flags -------
  if (FLAG_objectDetection){
    if (FLAG_objectF){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (OBJECT_outputF <= 15 && OBJECT_outputF != 0) {
          Right();
        } else{
          FLAG_objectF=false;
          FLAG_objectDetection=false;
        }
      }
    }
    if (FLAG_objectR){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (OBJECT_outputR <= 13 && OBJECT_outputR != 0) {
          Left();
        } else{
          FLAG_objectR=false;
          FLAG_objectDetection=false;
        }
      }
    }
    if (FLAG_objectL){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (OBJECT_outputL <= 13 && OBJECT_outputL != 0) {
          Right();
        } else{
          FLAG_objectL=false;
          FLAG_objectDetection=false;
        }
      }
    }
  }

  // Flame Detection Flags
  else if (!FLAG_objectDetection && FLAG_fireDetection) {
    if (FLAG_fireF){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (pcf8574.digitalRead(PIN_EXT_flameF)==LOW) {
          Stop();
          // this might be the issue to the turning problem
          delay(500);
          FanOn();
        } else {
          FLAG_fireF = false; 
          FLAG_fireDetection = false;
          FanOff();
        }
      }
    }
    if (FLAG_fireL){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (pcf8574.digitalRead(PIN_EXT_flameF)==HIGH) { FlameLeft(); }
        else { 
          FLAG_fireL = false; 
          FLAG_fireDetection = false;
        }
      }
    }
    if (FLAG_fireR){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (pcf8574.digitalRead(PIN_EXT_flameF)==HIGH) { FlameRight(); }
        else { 
          FLAG_fireR = false; 
          FLAG_fireDetection = false;
        }
      }
    }
  }

  // Line Following Flags (Turn until middle) -------
  else if (!FLAG_objectDetection && !FLAG_fireDetection && (FLAG_lineL || FLAG_lineR)) {
    if (FLAG_lineL){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (
          digitalRead(PIN_lineF) == LOW && 
          tempTime <= operationStartTime + MAX_LINE_TURN_DURATION ) { 
            SlightLeft(); 
        } 
        else {
          FLAG_lineL = false; 
        }
      }
    }
    if (FLAG_lineR) {
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (
          digitalRead(PIN_lineF) == LOW && 
          tempTime <= operationStartTime + MAX_LINE_TURN_DURATION ) { 
            SlightRight(); 
        } 
        else {
          FLAG_lineR = false; 
        }
      }
    }
  } 
  else {
    Forward();
  }

  if (FLAG_objectDetection) { FLAG_objectDetection = false; }
}

void ObjectAvoidanceMode () {
  FLAG_objectDetection = true;
  FLAG_lineL = false;
  FLAG_lineR = false;
}

void ObjectSensorOutputs(int R, int L,int F){
  Serial.print("Distance Front ----: ");
  Serial.println(F);  
  Serial.print("Distance Front Right----: ");
  Serial.println(R);  
  Serial.print("Distance Front Left----: ");
  Serial.println(L);  
}

void FireDetectionMode () {
  FLAG_objectDetection = false;
  FLAG_fireDetection = true;
  // FLAG_lineDetection = false;   CAN WE DO THIS???
  FLAG_lineL = false;
  FLAG_lineR = false;
}

void FireSensorOutputs(int R, int L, int F){
  Serial.print("IR FIRE LEFT On--------:");
  Serial.println(L);
  Serial.print("IR FIRE RIGHT On--------:");
  Serial.println(R);
  Serial.print("IR FIRE MIDDLE On--------:");
  Serial.println(F);
}

void LineDetectionMode () {
  FLAG_lineDetection = true;
}

void LineSensorOutputs(int R, int L,int F){
  Serial.print("Left On--------:");
  Serial.println(L);
  Serial.print("Right On--------:");
  Serial.println(R);
  Serial.print("Middle On--------:");
  Serial.println(F);
}

void FlagOutputs(bool object,bool fire,bool line,bool lineL,bool lineR){
  Serial.print("object detection--------:");
  Serial.println(object);
  Serial.print("fire detection--------:");
  Serial.println(fire);
  Serial.print("line detection --------:");
  Serial.println(line);
  Serial.print("line detection right--------:");
  Serial.println(lineR);
  Serial.print("line detection left--------:");
  Serial.println(lineL);
}

void FanOn () {
  pcf8574.digitalWrite(PIN_EXT_fan, HIGH);
}

void FanOff () {
  pcf8574.digitalWrite(PIN_EXT_fan, LOW);
}

void Right () {
  analogWrite(PWMAR, turnHigh); // Right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, turnHigh);  // Left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite(STBY, HIGH);
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
