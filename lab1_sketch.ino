int STBY=3; //standby power system and it must be high for the motor control board to be enabled
// int PWMA=5; //must be high for the right wheels to be enabled
// int PWMB=6; //must be high for the left wheels to be enabled
int BIN1=8; //high for forward movement and low for backward movement
int AIN1=7; // high for forward movement and low for backward movement
int PWMAR=10; // right motors A pin
int PWMBR=9; // right motors B pin
int PWMAL=5; // left motors A pin
int PWMBL=6; // left motors B pin
//int delayTime=3000;
int high = 100;
int low = 0;
int trigPinL = 13; // Connect the Trig pin of the left Ultrasonic Sensor to pin 9 on Arduino
int echoPinL = 12; // Connect the Echo pin of the left Ultrasonic Sensor to pin 10 on Arduino
int trigPinR = 8; // Connect the Trig pin of the right Ultrasonic Sensor to pin 8 on Arduino
int echoPinR = 7; // Connect the Echo pin of the right Ultrasonic Sensor to pin 7 on Arduino
  long durationR, distanceR, durationL, distanceL;
void setup () {
  Serial.begin(9600);   // initialize the serial communications:

  pinMode (AIN1, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (STBY, OUTPUT);
  // pinMode (PWMA, OUTPUT);
  // pinMode (PWMB, OUTPUT);
  pinMode(PWMAR, OUTPUT); 
  pinMode(PWMBR, OUTPUT); 
  pinMode (PWMAL, OUTPUT);
  pinMode (PWMBL, OUTPUT);

  // digitalWrite (PWMA, HIGH);
  // digitalWrite (PWMB, HIGH);
  // digitalWrite(PWM2A, HIGH); 
  // digitalWrite(PWM2B, HIGH); 
  // digitalWrite(AIN2, HIGH);
  // digitalWrite(BIN2, HIGH);


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
  int delayTime = 200 ;
}

void loop () {
  // Add this code where you want to read from the sensor
  digitalWrite(trigPinR, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinR, LOW);
  durationR = pulseIn(echoPinR, HIGH);
  distanceR = (durationR/2) / 29.1;

  digitalWrite(trigPinL, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinL, LOW);
  durationL = pulseIn(echoPinL, HIGH);
  distanceL = (durationL/2) / 29.1;
  
  Serial.print("Distance Right--: ");
  Serial.println(distanceR);  // Tools>Serial Monitor
  Serial.print("Distance Left----: ");
  Serial.println(distanceL);  // Tools>Serial Monitor

  if (distanceR > 40) {
    Backward();
    delay(300);
    Stop();
    delay(1000);
    Left();
    delay(500);
  } else if (distanceL > 40) {
    Backward();
    delay(300);
    Stop();
    delay(1000);
    Right();
    delay(500);
  }
  else {
    Forward();
  }

}

void Right () {
  // digitalWrite (AIN1, HIGH);
  // digitalWrite (BIN1, LOW); // Move

  // digitalWrite(PWM2A, LOW); // right wheels foreward
  // digitalWrite(PWM2B, HIGH); 
  // digitalWrite(AIN2, LOW); // left wheels backwards
  // digitalWrite(BIN2, HIGH);

  analogWrite(PWMAR, high); // right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, high);  // left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}

void Left () {
  // digitalWrite (AIN1, LOW);
  // digitalWrite (BIN1, HIGH); // Move

  // digitalWrite(AIN2, HIGH);  // left wheels forward
  // digitalWrite(BIN2, LOW); 
  // digitalWrite(PWM2A, HIGH); // right wheels backwards
  // digitalWrite(PWM2B, LOW); 

  analogWrite(PWMAR, low); // right wheels forward
  analogWrite(PWMBR, high); 
  analogWrite(PWMAL, low);  // left wheels backwards
  analogWrite(PWMBL, high); 
  digitalWrite (STBY, HIGH);

}

void Forward () {
  // digitalWrite (AIN1, HIGH);
  // digitalWrite (BIN1, HIGH); // Move
  // digitalWrite(PWM2A, LOW); // right wheels foreward
  // digitalWrite(PWM2B, HIGH); 
  analogWrite(PWMAR, low); // right wheels forward
  analogWrite(PWMBR, high); 
  // digitalWrite(AIN2, HIGH);  // left wheels forward
  // digitalWrite(BIN2, LOW); 
  analogWrite(PWMAL, high);  // left wheels forward
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
  // delay (delayTime); // Stationary
  // digitalWrite (STBY, LOW);
  // delay (delayTime);
}

void Backward () {
  // digitalWrite (AIN1, LOW);
  // digitalWrite (BIN1, LOW); // Move

  // digitalWrite(PWM2A, HIGH); // right wheels backwards
  // digitalWrite(PWM2B, LOW); 
  // digitalWrite(AIN2, LOW); // left wheels backwards
  // digitalWrite(BIN2, HIGH);
  analogWrite(PWMAR, high); // right wheels backwards
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, low);  // left wheels backwards
  analogWrite(PWMBL, high); 
  digitalWrite (STBY, HIGH);
  // delay (delayTime); // Stationary
  // digitalWrite (STBY, LOW);
  // delay (delayTime);
}

void Stop () {
  analogWrite(PWMAR, low); // right wheels stop
  analogWrite(PWMBR, low); 
  analogWrite(PWMAL, low);  // left wheels stop
  analogWrite(PWMBL, low); 
  digitalWrite (STBY, HIGH);
}
