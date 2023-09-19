int STBY=3; //standby power system and it must be high for the motor control board to be enabled
int PWMA=5; //must be high for the right wheels to be enabled
int PWMB=6; //must be high for the lest wheels to be enabled
int BIN1=8; //high for forward movement and low for backward movement
int AIN1=7; // high for forward movement and low for backward movement
//int delayTime=3000;
int motorSpeed = 90;
int trigPin = 13; // Connect the Trig pin of the Ultrasonic Sensor to pin 9 on Arduino
int echoPin = 12; // Connect the Echo pin of the Ultrasonic Sensor to pin 10 on Arduino
  long duration, distance;
void setup () {
  Serial.begin(9600);   // initialize the serial communications:

  pinMode (AIN1, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (STBY, OUTPUT);
  pinMode (PWMA, OUTPUT);
  pinMode (PWMB, OUTPUT);

  digitalWrite (PWMA, HIGH);
  digitalWrite (PWMB, HIGH);

  //Adjust Speed
  analogWrite(PWMA, motorSpeed); 
  analogWrite(PWMB, motorSpeed); 

  // Initialize the Ultrasonic Sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  int delayTime = 200 ;
}

void loop () {
  // Add this code where you want to read from the sensor
  //ong duration, distance;
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  
  Serial.print("Distance--: ");
  Serial.println(distance);  // Tools>Serial Monitor

  if (distance > 20){
    Backward();
    delay(100);
    Right();
    delay(200);
  }else{
    Forward();
  }

}

void Left () {
  digitalWrite (AIN1, HIGH);
  digitalWrite (BIN1, LOW); // Move
  digitalWrite (STBY, HIGH);
}

void Right () {
  digitalWrite (AIN1, LOW);
  digitalWrite (BIN1, HIGH); // Move
  digitalWrite (STBY, HIGH);

}

void Forward () {
  digitalWrite (AIN1, HIGH);
  digitalWrite (BIN1, HIGH); // Move
  digitalWrite (STBY, HIGH);
  // delay (delayTime); // Stationary
  // digitalWrite (STBY, LOW);
  // delay (delayTime);
}

void Backward () {
  digitalWrite (AIN1, LOW);
  digitalWrite (BIN1, LOW); // Move
  digitalWrite (STBY, HIGH);
  // delay (delayTime); // Stationary
  // digitalWrite (STBY, LOW);
  // delay (delayTime);
}
