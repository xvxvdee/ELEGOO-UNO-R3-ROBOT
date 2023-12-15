#include <NewPing.h>
#include <PCF8574.h>

#define MAX_DISTANCE 5000
int MAX_LINE_TURN_DURATION = 3000;
int MAX_FLAME_TURN_DURATION = 3000;

int OFFSET = 1;

// AMR ---------------------------------------------------
int STBY=3; // Standby Power System (must be high to enable Motor Control Board)
// Motors
int PWMAR=10; // Right motors A pin
int PWMBR=9; // Right motors B pin
int PWMAL=5; // Left motors A pin
int PWMBL=6; // Left motors B pin
// Turning Speed
int high = 125;
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
uint8_t PIN_EXT_flameML = P1;
uint8_t PIN_EXT_flameL = P2;
uint8_t PIN_EXT_flameMR = P5; 
uint8_t PIN_EXT_flameR = P6; 
uint8_t PIN_EXT_flameF = P3;

// Flame Extinguisher Fan - - - - - - - - - - - -
uint8_t PIN_EXT_fan = P4;

// IO Extender - - - - - - - - - - - - - - - - - 
PCF8574 EXT1(0x20);
PCF8574 EXT2(0x21);

// Ultrasonic Sensors Trig and Echo Pins - - - - 
int PIN_TRIG_objectF = 3; // Trig pin of Front Ultrasonic Sensor to pin 3 
int PIN_ECHO_objectF = 4; // Echo pin of Front Ultrasonic Sensor to pin 4 
int PIN_TRIG_objectFR = A1; // Echo pin of Front Right Ultrasonic Sensor to pin A1 
int PIN_ECHO_objectFR = A0; // Trig pin of Front Right Ultrasonic Sensor to pin A0 
int PIN_TRIG_objectFL = A3; // Echo pin of Front Left Ultrasonic Sensor to pin A3 
int PIN_ECHO_objectFL = A2; // Trig pin of Front Left Ultrasonic Sensor to pin A2

int PIN_EXT_TRIG_objectB = 11; //// Back Sensor
int PIN_EXT_ECHO_objectB = 12; 

// IR Sensors for Edge Detection - - - - - -
int PIN_objectTF = 0;
int PIN_objectTL = 0;
int PIN_objectTR = 0;

long  OBJECT_outputF,OBJECT_outputTF,OBJECT_outputR,OBJECT_outputTR, OBJECT_outputL, OBJECT_outputTL,OBJECT_outputB; //Ultrasonic sensors + IR Sensors

// Flags ------------------------------------------------------------------------
bool FLAG_objectDetection = false;
bool FLAG_objectL = false;
bool FLAG_objectR = false;
bool FLAG_objectF = false;
bool FLAG_objectB = false;

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
// NewPing OBJECT_sonarTL(PIN_EXT_TRIG_objectTFL, PIN_EXT_ECHO_objectTFL, MAX_DISTANCE);
// NewPing OBJECT_sonarTR(PIN_EXT_TRIG_objectTFR, PIN_EXT_ECHO_objectTFR, MAX_DISTANCE);
// NewPing OBJECT_sonarTF(PIN_TRIG_objectTF, PIN_ECHO_objectTF, MAX_DISTANCE);
NewPing OBJECT_sonarB(PIN_EXT_TRIG_objectB, PIN_EXT_ECHO_objectB, MAX_DISTANCE); 

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
  pinMode(PIN_EXT_TRIG_objectB, OUTPUT);
  pinMode(PIN_EXT_ECHO_objectB, INPUT);

  // IR Sensors
  pinMode(PIN_objectTF, INPUT);
  pinMode(PIN_objectTL, INPUT);
  pinMode(PIN_objectTR, INPUT);

  // Flame IR Sensors - - - - - - - -
  EXT1.pinMode(PIN_EXT_flameF, INPUT);
  EXT1.pinMode(PIN_EXT_flameML, INPUT);
  EXT1.pinMode(PIN_EXT_flameMR, INPUT);
  EXT1.pinMode(PIN_EXT_flameL, INPUT);
  EXT1.pinMode(PIN_EXT_flameR, INPUT);

  // Flame Fan - - - - - - - - - - - - 
  EXT1.pinMode(PIN_EXT_fan, OUTPUT);
  
  // Initialize IO Extender
  if (EXT1.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }

  // if (EXT2.begin()) {
  //   Serial.println("OK");
  // } else {
  //   Serial.println("KO");
  // }

  EXT1.digitalWrite(PIN_EXT_fan, LOW);
}

void loop () {
  unsigned long operationStartTime;
  unsigned long tempTime;

  // Flame Detection Sensors --------------------- 
  FireSensorOutputs(EXT1,PIN_EXT_flameF,PIN_EXT_flameMR,PIN_EXT_flameML,PIN_EXT_flameR,PIN_EXT_flameL);
  // Serial.print("IR FIRE MIDDLE ----: ");
  // Serial.println(EXT1.digitalRead(PIN_EXT_flameF));  
  // Serial.print("IR FIRE MIDDLE RIGHT --------: ");
  // Serial.println(EXT1.digitalRead(PIN_EXT_flameMR));  
  // Serial.print("IR FIRE MIDDLE LEFT --------: ");
  // Serial.println(EXT1.digitalRead(PIN_EXT_flameML)); 
  // Serial.print("IR FIRE RIGHT --------: ");
  // Serial.println(EXT1.digitalRead(PIN_EXT_flameR));  
  // Serial.print("IR FIRE LEFT --------: ");
  // Serial.println(EXT1.digitalRead(PIN_EXT_flameL)); 

  // Object Avoidance Sensors --------------------
  OBJECT_outputR = OBJECT_sonarR.ping_cm();
  OBJECT_outputL = OBJECT_sonarL.ping_cm();
  OBJECT_outputF = OBJECT_sonarF.ping_cm();
  OBJECT_outputB = OBJECT_sonarB.ping_cm();

  OBJECT_outputTF = digitalRead(PIN_objectTF);
  OBJECT_outputTL = digitalRead(PIN_objectTL);
  OBJECT_outputTR = digitalRead(PIN_objectTR);

  // Serial.print("Distance Front ----: ");
  // Serial.println(OBJECT_outputF);  
  // Serial.print("Distance Front Right----: ");
  // Serial.println(OBJECT_outputR);  
  // Serial.print("Distance Front Left----: ");
  // Serial.println(OBJECT_outputL); 
  // Serial.print("Distance BACK----: ");
  // Serial.println(OBJECT_outputB); 
  // Serial.print("IR Object Detected Top Front --------:");
  // Serial.println(OBJECT_outputTF);
  // Serial.print("IR Object Detected Top Left --------:");
  // Serial.println(OBJECT_outputTL);
  // Serial.print("IR Object Detected Top Right --------:");
  // Serial.println(OBJECT_outputTR);
  ObjectSensorOutputs(OBJECT_outputR,OBJECT_outputL,OBJECT_outputF,OBJECT_outputB,OBJECT_outputTF,OBJECT_outputTL,OBJECT_outputTR); 
  
  // Line Following --------------------
  int LINE_outputL = digitalRead(PIN_lineL); 
  int LINE_outputF = digitalRead(PIN_lineF); 
  int LINE_outputR = digitalRead(PIN_lineR);
  // LineSensorOutputs(LINE_outputR,LINE_outputL,LINE_outputF);

  // Object Avoidance ----------------------------------------
  if (OBJECT_outputF <= 10 && OBJECT_outputF != 0) { 
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectF=true;
  } 
  else if (OBJECT_outputTF == HIGH){
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectF=true;
  }
  else if ((OBJECT_outputR <= 10 && OBJECT_outputR != 0) || (OBJECT_outputTR == HIGH)) {
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectR=true;
  } 
  else if ((OBJECT_outputL <= 10 && OBJECT_outputL != 0) || (OBJECT_outputTL == HIGH)) { 
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectL=true;
  }
  else if (OBJECT_outputB <= 10 && OBJECT_outputB !=0){
    operationStartTime=millis();
    ObjectAvoidanceMode();
    FLAG_objectB=true;
  }
  // Flame Detection (HIGH = detected) -------------------------
  else if (EXT1.digitalRead(PIN_EXT_flameF) == HIGH) { 
    operationStartTime=millis();
    FireDetectionMode();
    FLAG_fireF = true;
  }  else if ((EXT1.digitalRead(PIN_EXT_flameMR) == HIGH) || (EXT1.digitalRead(PIN_EXT_flameR) == HIGH)) { // (EXT1.digitalRead(PIN_EXT_flameMR) == HIGH) || (EXT1.digitalRead(PIN_EXT_flameR) == HIGH)
    operationStartTime=millis();
    FireDetectionMode();
    FLAG_fireR = true;
  }  else if ((EXT1.digitalRead(PIN_EXT_flameML) == HIGH) || (EXT1.digitalRead(PIN_EXT_flameL) == HIGH)) {//(EXT1.digitalRead(PIN_EXT_flameML) == HIGH) || (EXT1.digitalRead(PIN_EXT_flameL) == HIGH)
    operationStartTime=millis();
    FireDetectionMode();
    FLAG_fireL = true;
  }

  // Line Following --------------------
  else if (LINE_outputF && !LINE_outputR && !LINE_outputL) {
   // Forward();
    FLAG_lineF = true;
  }
  else if (LINE_outputF && LINE_outputR && LINE_outputL || LINE_outputF && LINE_outputL || LINE_outputL) {
    operationStartTime=millis();
    FLAG_lineL = true;
  }
  else if (LINE_outputF && LINE_outputR || LINE_outputR) {
    operationStartTime=millis();
    FLAG_lineR = true;
  }

  FlagOutputs(FLAG_objectDetection,FLAG_fireDetection,FLAG_lineF,FLAG_lineL,FLAG_lineR);

  // Object Detection Flags ---------
  if (FLAG_objectDetection){
    if (FLAG_objectF){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (OBJECT_outputF <= 15 && OBJECT_outputF != 0) { 
          Right();
        } else if (digitalRead(PIN_objectTF) == HIGH){
          Left();
        }else{
          FLAG_objectF=false;
          FLAG_objectDetection=false;
        }
      }
    }
    if (FLAG_objectR){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if ((OBJECT_outputR <= 13 && OBJECT_outputR != 0)||(digitalRead(PIN_objectTR) == HIGH)) { 
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
        if ((OBJECT_outputL <= 13 && OBJECT_outputL != 0)||(digitalRead(PIN_objectTL) == HIGH)) { 
          Right();
        } else{
          FLAG_objectL=false;
          FLAG_objectDetection=false;
        }
      }
    }
    if (FLAG_objectB){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (OBJECT_outputB <= 10 && OBJECT_outputB != 0) { 
          Right();
        } else{
          FLAG_objectB=false;
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
        if (EXT1.digitalRead(PIN_EXT_flameF)==HIGH) {
          Stop();
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
        if (EXT1.digitalRead(PIN_EXT_flameF)==LOW && tempTime <= operationStartTime + MAX_FLAME_TURN_DURATION) { 
          FlameLeft(); }
        else { 
          FLAG_fireL = false; 
          FLAG_fireDetection = false;
        }
      }
    }
    if (FLAG_fireR){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (EXT1.digitalRead(PIN_EXT_flameF)==LOW && tempTime <= operationStartTime + MAX_FLAME_TURN_DURATION) { FlameRight(); }
        else { 
          FLAG_fireR = false; 
          FLAG_fireDetection = false;
        }
      }
    }
  }

  // Line Following Flags (Turn until middle) -------
  else if (!FLAG_objectDetection && !FLAG_fireDetection && (FLAG_lineF || FLAG_lineL || FLAG_lineR)) {
    if (FLAG_lineL){
      tempTime = millis();
      if (tempTime >= operationStartTime + OFFSET) {
        if (
          digitalRead(PIN_lineF) == LOW && 
          tempTime <= operationStartTime + MAX_LINE_TURN_DURATION ) { 
            SlightLeft(); 
            //Serial.println("hihihihi left");
        } 
        else {
          FLAG_lineL = false; 
          //Serial.println("I GAVE UP LEFT");
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
           // Serial.println("hihihihi right"); 
        } 
        else {
          //Serial.println("I GAVE UP RIGHT");
          FLAG_lineR = false; 
        }
      }
    }
    if (FLAG_lineF) {
      if (digitalRead(PIN_lineF) == HIGH) { 
        Forward();
       // Serial.println("hihihihi"); 
      } else {
        FLAG_lineF = false;
      }
    }
  } 
  else {
//Serial.println("DID U REACH THIS?"); 
    Forward();
  }

  if (FLAG_objectDetection) { FLAG_objectDetection = false; }
}

void ObjectAvoidanceMode () {
  FLAG_objectDetection = true;
  FLAG_lineL = false;
  FLAG_lineR = false;
}

void ObjectSensorOutputs(long R, long L,long F, long B, int TF, int TL, int TR){ 
  Serial.print("Distance Front ----: ");
  Serial.println(F);  
  Serial.print("Distance Front Right----: ");
  Serial.println(R);  
  Serial.print("Distance Front Left----: ");
  Serial.println(L); 
  Serial.print("Distance Back----: ");
  Serial.println(B); 
  Serial.print("IR Object Detected Top Front --------:");
  Serial.println(TF);
  Serial.print("IR Object Detected Top Left --------:");
  Serial.println(TL);
  Serial.print("IR Object Detected Top Right --------:");
  Serial.println(TR);
}

void FireDetectionMode () {
  FLAG_objectDetection = false;
  FLAG_fireDetection = true;
  FLAG_lineL = false;
  FLAG_lineR = false;
}

void FireSensorOutputs(PCF8574 EXT, int F, int MR, int ML, int R, int L){ //int R, int L
  Serial.print("IR FIRE LEFT --------:");
  Serial.println(EXT1.digitalRead(L));
  Serial.print("IR FIRE MIDDLE LEFT --------:");
  Serial.println(EXT1.digitalRead(ML));
  Serial.print("IR FIRE MIDDLE --------:");
  Serial.println(EXT1.digitalRead(F));
  Serial.print("IR FIRE MIDDLE RIGHT --------:");
  Serial.println(EXT1.digitalRead(MR));
  Serial.print("IR FIRE RIGHT --------:");
  Serial.println(EXT1.digitalRead(R));
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
  Serial.print("line detection middle --------:");
  Serial.println(line);
  Serial.print("line detection right--------:");
  Serial.println(lineR);
  Serial.print("line detection left--------:");
  Serial.println(lineL);
}

void FanOn () {
  EXT1.digitalWrite(PIN_EXT_fan, HIGH);
}

void FanOff () {
  EXT1.digitalWrite(PIN_EXT_fan, LOW);
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
  analogWrite(PWMAL, turnLow);  // Left wheels forward
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
  analogWrite(PWMBR, turnLow); 
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
