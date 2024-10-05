#include <Servo.h>
#include <SoftwareSerial.h>

Servo myservo;
Servo myservo2;
Servo myservo3;
Servo myservo4;



int S_A ;  //speed motor a
int M_A1 ; //motor a = +
int M_A2 ; //motor a = -
int M_B1 ; //motor b = -
int M_B2 ; //motor b = +
int S_B ;  //speed motor b
int pump;
 
int R_S ; //Right sensor
int S_S ; //Center sensor
int L_S ; //Left sensor 

int x = 0, y = 0;
int servo[4];
bool Follow, Pump = 0;

SoftwareSerial Bluetooth(2, 3);  // RX, TX for Bluetooth
String dataIn = "";

void setup() {
   myservo.attach(6);
   myservo2.attach(9);
   myservo3.attach(10);  
   myservo4.attach(11);

  pinMode(M_B1, OUTPUT);
  pinMode(M_B2, OUTPUT);
  pinMode(M_A1, OUTPUT);
  pinMode(M_A2, OUTPUT);
  pinMode(S_B, OUTPUT);
  pinMode(S_A, OUTPUT);
  
  pinMode(L_S, INPUT);
  pinMode(S_S, INPUT);
  pinMode(R_S, INPUT);

  analogWrite(S_A, 150); 
  analogWrite(S_B, 150); 

    Bluetooth.begin(9600); // Initialize Bluetooth communication
}
 
void loop() {
  control_robotic_arm();
  if(Follow){
      line_Follow();
}
  else{
     RC_control();
    if(Pump){
      digitalWrite(pump, HIGH);
    }  
    else{
      digitalWrite(pump, LOW);
    }

  }
   
}

void recive_date(){
  // Check for incoming data
   if (Bluetooth.available() > 0) {
       dataIn = Bluetooth.readString();  // Read the data as string
   }

   // Process servo commands
   for (int x = 0; x < 4; x++) {
       char i = (char)(x + '0');  // Convert x to corresponding character
       if (dataIn[0] == i) {
           String dataInS = dataIn.substring(1);
           servo[x] = dataInS.toInt();
       }
   }

   // Update servo positions
   myservo.write(servo[0]);   // Move shoulder servo
   myservo2.write(servo[1]);  // Move bow servo
   myservo3.write(servo[2]);  // Move hand servo
   myservo4.write(servo[3]);  // Move gripper servo

   // Process joystick x and y values
   if (dataIn[0] == 'x') {
       String dataInS = dataIn.substring(1);
       x = dataInS.toInt();  // Update x value of the joystick
   }
   if (dataIn[0] =='y') {
       String dataInS = dataIn.substring(1);
       y = dataInS.toInt();  // Update y value of the joystick
   }

   // Handle "Follow" and "Pump" commands (you need to add functionality here)
   if (dataIn == "Follow") {
       // Implement "Follow" mode functionality
       Follow = 1;
   }
   if (dataIn == "Pump") {
       // Implement "Pump" mode functionality
       Pump = 1;
   }
}
void line_Follow(){
  String united = String(digitalRead(L_S)) + String(digitalRead(S_S)) + String(digitalRead(R_S));
  int   iunited = united.toInt();
  switch (iunited) {

      case 110:
        turnLeft();
        break;

      case 011:
        turnRight();
        break;
        
      case 100:
        turnLeft();
        break;

      case 001:
        turnRight();
        break;

      case 010:
        digitalWrite(6, HIGH);
        break;
        
      default:
;
      }
}
void control_robotic_arm(){
 myservo.write(servo[0]);
 myservo2.write(servo[1]); 
 myservo3.write(servo[2]);
 myservo4.write(servo[3]); 

}
void RC_control(){
  int speedA = map(abs(y - 512), 0, 512, 0, 255);
  int speedB = map(abs(x - 512), 0, 512, 0, 255);
  if (y > 74 ) { // Move forward
    forword();
    analogWrite(S_A, speedA); // Speed control
  } else if (y< 74) { // Move backward
      backward();
    analogWrite(S_A, speedA); 
    // Speed control
  } 

  // Left/Right control
  if (x < 137) { // Turn left
      turnLeft();
    analogWrite(S_B, speedB); // Speed control
  } else if (x  > 137) { // Turn right
    turnRight();
    analogWrite(S_B, speedB); // Speed control
  } 
}
void forword(){
  digitalWrite(M_A1, LOW);
  digitalWrite(M_A2, HIGH);
  digitalWrite(M_B1, HIGH);
  digitalWrite(M_B2, LOW);  
}
void backward(){
  digitalWrite(M_A1, HIGH);
  digitalWrite(M_A2, LOW);
  digitalWrite(M_B1, LOW);
  digitalWrite(M_B2, HIGH);  
}
void turnRight(){
  digitalWrite(M_A1, LOW);
  digitalWrite(M_A2, LOW);
  digitalWrite(M_B1, HIGH);
  digitalWrite(M_B2, LOW);  
} 
void turnLeft(){
  digitalWrite(M_A1, LOW);
  digitalWrite(M_A2, HIGH);
  digitalWrite(M_B1, LOW);
  digitalWrite(M_B2, LOW);
}
void Stop(){
  digitalWrite(M_A1, LOW);
  digitalWrite(M_A2, LOW);
  digitalWrite(M_B1, LOW);
  digitalWrite(M_B2, LOW);
}
 

