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
  long durationR, distanceR, durationL, distanceL;
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

  if (distanceR > 40 && distanceL > 40) {
    Backward();
    delay(300);
    Right();
    delay(1200);
  } else if (distanceR > 40) {
    Backward();
    delay(300);
    Left();
    delay(750);
  } else if (distanceL > 40) {
    Backward();
    delay(300);
    Right();
    delay(750);
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
