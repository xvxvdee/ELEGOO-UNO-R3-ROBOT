nt STBY=3; //standby power system and it must be high for the motor control board to be enabled
// int PWMA=5; //must be high for the right wheels to be enabled
// int PWMB=6; //must be high for the left wheels to be enabled
int BIN1=8; //high for forward movement and low for backward movement
int AIN1=7; // high for forward movement and low for backward movement
int PWM2A=10; // right motors A pin
int PWM2B=9; // right motors B pin
int AIN2=2; // left motors A pin
int BIN2=4; // left motors B pin
//int delayTime=3000;
int motorSpeed = 90;
int trigPinL = 13; // Connect the Trig pin of the left Ultrasonic Sensor to pin 9 on Arduino
int echoPinL = 12; // Connect the Echo pin of the left Ultrasonic Sensor to pin 10 on Arduino
int trigPinR = 8; // Connect the Trig pin of the right Ultrasonic Sensor to pin 8 on Arduino
int echoPinR = 7; // Connect the Echo pin of the right Ultrasonic Sensor to pin 7 on Arduino
  long duration, distance;
void setup () {
  Serial.begin(9600);   // initialize the serial communications:

  pinMode (AIN1, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (STBY, OUTPUT);
  // pinMode (PWMA, OUTPUT);
  // pinMode (PWMB, OUTPUT);
  pinMode(PWM2A, OUTPUT); 
  pinMode(PWM2B, OUTPUT); 
  pinMode (AIN2, OUTPUT);
  pinMode (BIN2, OUTPUT);

  // digitalWrite (PWMA, HIGH);
  // digitalWrite (PWMB, HIGH);
  digitalWrite(PWM2A, HIGH); 
  digitalWrite(PWM2B, HIGH); 
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN2, HIGH);


  //Adjust Speed
  // analogWrite(PWMA, motorSpeed); 
  // analogWrite(PWMB, motorSpeed); 
  analogWrite(PWM2A, motorSpeed); 
  analogWrite(PWM2B, motorSpeed);   
  analogWrite(AIN2, motorSpeed);
  analogWrite(BIN2, motorSpeed);

  // Initialize the Ultrasonic Sensor pins
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  int delayTime = 200 ;
}

void loop () {
  // Add this code where you want to read from the sensor
  //ong duration, distance;
  digitalWrite(trigPinL, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinL, LOW);
  duration = pulseIn(echoPinL, HIGH);
  distance = (duration/2) / 29.1;
  
  Serial.print("Distance--: ");
  Serial.println(distance);  // Tools>Serial Monitor

  if (distance > 20) {
    Backward();
    delay(100);
    Right();
    delay(200);
  } else {
    Forward();
  }

}

void Left () {
  // digitalWrite (AIN1, HIGH);
  // digitalWrite (BIN1, LOW); // Move

  digitalWrite(PWM2A, HIGH); // right wheels foreward
  digitalWrite(PWM2B, LOW); 
  digitalWrite(AIN2, HIGH); // left wheels backwards
  digitalWrite(BIN2, LOW);
  digitalWrite (STBY, HIGH);
}

void Right () {
  // digitalWrite (AIN1, LOW);
  // digitalWrite (BIN1, HIGH); // Move

  digitalWrite(AIN2, LOW);  // left wheels forward
  digitalWrite(BIN2, HIGH); 
  digitalWrite(PWM2A, LOW); // right wheels backwards
  digitalWrite(PWM2B, HIGH); 
  digitalWrite (STBY, HIGH);

}

void Forward () {
  // digitalWrite (AIN1, HIGH);
  // digitalWrite (BIN1, HIGH); // Move
  digitalWrite(PWM2A, HIGH); // right wheels foreward
  digitalWrite(PWM2B, LOW); 
  digitalWrite(AIN2, LOW);  // left wheels forward
  digitalWrite(BIN2, HIGH); 
  digitalWrite (STBY, HIGH);
  // delay (delayTime); // Stationary
  // digitalWrite (STBY, LOW);
  // delay (delayTime);
}

void Backward () {
  // digitalWrite (AIN1, LOW);
  // digitalWrite (BIN1, LOW); // Move

  digitalWrite(PWM2A, LOW); // right wheels backwards
  digitalWrite(PWM2B, HIGH); 
  digitalWrite(AIN2, HIGH); // left wheels backwards
  digitalWrite(BIN2, LOW);
  digitalWrite (STBY, HIGH);
  // delay (delayTime); // Stationary
  // digitalWrite (STBY, LOW);
  // delay (delayTime);
}
