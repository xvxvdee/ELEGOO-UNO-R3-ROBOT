int STBY=3; //standby power system and it must be high for the motor control board to be enabled
int BIN1=8; //high for forward movement and low for backward movement
int AIN1=7; // high for forward movement and low for backward movement
int PWMAR=10; // right motors A pin
int PWMBR=9; // right motors B pin
int PWMAL=5; // left motors A pin
int PWMBL=6; // left motors B pin
int high = 80;
int low = 0;
int trigPinL = 13; // Connect the Trig pin of the left Ultrasonic Sensor to pin 9 on Arduino
int echoPinL = 12; // Connect the Echo pin of the left Ultrasonic Sensor to pin 10 on Arduino
int trigPinR = 8; // Connect the Trig pin of the right Ultrasonic Sensor to pin 8 on Arduino
int echoPinR = 7; // Connect the Echo pin of the right Ultrasonic Sensor to pin 7 on Arduino
int trigPinF = 3; // Connect the Trig pin of the front-facing Ultrasonic Sensor to pin 3 on Arduino
int echoPinF = 4; // Connect the Echo pin of the front-facing Ultrasonic Sensor to pin 4 on Arduino
int echoPinFR = A0; // Connect the Trig pin of the right front-facing Ultrasonic Sensor to pin A0 on Arduino
int trigPinFR = A1; // Connect the Echo pin of the right front-facing Ultrasonic Sensor to pin A1 on Arduino
int echoPinFL = A2; // Connect the Trig pin of the left front-facing Ultrasonic Sensor to pin A2 on Arduino
int trigPinFL = A3; // Connect the Echo pin of the left front-facing Ultrasonic Sensor to pin A3 on Arduino
int echoPinB = A5; // Connect the Echo pin of the left front-facing Ultrasonic Sensor to pin A5 on Arduino
int trigPinB = A4; // Connect the Trig pin of the left front-facing Ultrasonic Sensor to pin A4 on Arduino
  long durationR, distanceR, durationL, distanceL, durationF, distanceF, durationFR, distanceFR, distanceFL, durationFL, durationB, distanceB;
void setup () {
  Serial.begin(9600);   // initialize the serial communications:

  pinMode (AIN1, OUTPUT);
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
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(trigPinFR, OUTPUT);
  pinMode(echoPinFR, INPUT);
  pinMode(trigPinFL, OUTPUT);
  pinMode(echoPinFL, INPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
  int delayTime = 200 ;
}

void loop () {
  // Add this code where you want to read from the sensor
  distanceR = findDistance(trigPinR, echoPinR);
  distanceL = findDistance(trigPinL, echoPinL);

  distanceF = findDistance(trigPinF, echoPinF);
  distanceFR = findDistance(trigPinFR, echoPinFR);
  distanceFL = findDistance(trigPinFL, echoPinFL);

  distanceB = findDistance(trigPinB, echoPinB);
  
  Serial.print("Distance Right--: ");
  Serial.println(distanceR);  // Tools>Serial Monitor
  Serial.print("Distance Left----: ");
  Serial.println(distanceL);  // Tools>Serial Monitor
  Serial.print("Distance Front----: ");
  Serial.println(distanceF);  // Tools>Serial Monitor
  Serial.print("Distance Front Right----: ");
  Serial.println(distanceFR);  // Tools>Serial Monitor
  Serial.print("Distance Front Left----: ");
  Serial.println(distanceFL);  // Tools>Serial Monitor
  Serial.print("Distance Back----: ");
  Serial.println(distanceB);  // Tools>Serial Monitor

 
  if (distanceF <= 30) { // 30 might be too much
    Backward();
    delay(300);
    Right();
    delay(1200);
  } else if (distanceFR <= 30) {
    Backward();
    delay(300);
    Left();
    delay(500);
  } else if (distanceFL <= 30) {
    Backward();
    delay(300);
    Right();
    delay(500);
  } else if (distanceR > 40 && distanceL > 40) {
    Backward();
    delay(300);
    Right();
    delay(1200);
  } else if (distanceR > 40 || distanceR < 3) {
    Backward();
    delay(300);
    Left();
    delay(750);
  } else if (distanceL > 40 || distanceL < 3) {
    Backward();
    delay(300);
    Right();
    delay(750);
  // }else if(distanceB < 3 || distanceB > 40){
  //   Forward();
  //   delay(300);
  // }
  //else if(distance > 40){

  }
  else {
    Forward();
  }

}

void Right () {
  analogWrite(PWMAR, high); // right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, high);  // left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void Left () {
  analogWrite(PWMAR, low); // right wheels forward
  analogWrite(PWMBR, high); 
  analogWrite(PWMAL, low);  // left wheels backwards
  analogWrite(PWMBL, high); 
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

long findDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return ((duration/2) / 29.1);
}
