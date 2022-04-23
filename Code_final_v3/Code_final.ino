/*
     Datei: Project_V2.1.ino
     Autoren: Michiel Kindt, Maxime Roellinger, Enzo Job
     Datum: 24.05.2021
     Version: 2.3

   Funktion:
     • Das Programm dient zur Steuerung des Golf Ball Sammler.
*/

#include <Pixy2.h>

Pixy2 pixy;   // This is the main Pixy object

#define echoPinLeft 14                          // attach pin A0 Arduino to pin Echo of HC-SR04
#define trigPinLeft 15                          // attach pin A1 Arduino to pin Trig of HC-SR04
#define echoPinFront 16                         // attach pin A2 Arduino to pin Echo of HC-SR04
#define trigPinFront 17                         // attach pin A3 Arduino to pin Trig of HC-SR04
/*
  #define echoPinRight 18                         // attach pin A4 Arduino to pin Echo of HC-SR04
  #define trigPinRight 19                         // attach pin A5 Arduino to pin Trig of HC-SR04
*/

float distanceLeft;                             // variable for the distance measurement
float distanceFront;                            // variable for the distance measurement
/*
  float distanceFront;                            // variable for the distance measurement
*/

int PWM_Left = 10; // MCU PWM Pin 10 to ENA on L298n Board
int Forward_Left = 9;  // MCU Digital Pin 9 to IN1 on L298n Board
int Backward_Left = 8;  // MCU Digital Pin 8 to IN2 on L298n Board

int PWM_Right = 5;  // MCU PWM Pin 5 to ENB on L298n Board
int Forward_Right = 6;  // MCU Digital pin 6 to IN3 on L298n Board
int Backward_Right = 7;  // MCU Digital pin 7 to IN4 on L298n Board

int PWM_Brush = 3;  // MCU PWM Pin 3 to ENB on L298n Board
int Forward_Brush = 2;  // MCU Digital pin 2 to IN3 on L298n Board
int Backward_Brush = 4;  // MCU Digital pin 4 to IN4 on L298n Board

int pwmSlow = 70;
int pwmMove = 110;
int pwmBrush = 100;
int avoidingDistanceFront = 30;
int avoidingDistanceSide = 30;

int field_width = 316;
int field_height = 208;
int margin = 30;
bool obj_round = false;
bool stopTime = 0 ;

void setup() {
  Serial.begin(115200);                         // Serial Communication is starting with 115200 of baudrate speed
  Serial.print("Starting...\n");
  pixy.init();

  pinMode(trigPinLeft, OUTPUT);                 // Sets the trigPinLeft as an OUTPUT
  pinMode(echoPinLeft, INPUT);                  // Sets the echoPinLeft as an INPUT
  pinMode(trigPinFront, OUTPUT);                // Sets the trigPinFront as an OUTPUT
  pinMode(echoPinFront, INPUT);                 // Sets the echoPinFront as an INPUT
  /*
    pinMode(trigPinRight, OUTPUT);                // Sets the trigPinFront as an OUTPUT
    pinMode(echoPinRight, INPUT);                 // Sets the echoPinFront as an INPUT
  */

  pinMode(PWM_Left, OUTPUT); //Set all the L298n Pin to output
  pinMode(Forward_Left, OUTPUT);
  pinMode(Backward_Left, OUTPUT);

  pinMode(PWM_Right, OUTPUT);
  pinMode(Forward_Right, OUTPUT);
  pinMode(Backward_Right, OUTPUT);

  pinMode(PWM_Brush, OUTPUT);
  pinMode(Forward_Brush, OUTPUT);
  pinMode(Backward_Brush, OUTPUT);
}

void Left_Motor(int pwm, boolean forward) {
  analogWrite(PWM_Left, pwm); //set pwm control, 0 for stop, and 255 for maximum speed
  if (forward)  {
    digitalWrite(Forward_Left, HIGH);
    digitalWrite(Backward_Left, LOW);
  }
  else  {
    digitalWrite(Forward_Left, LOW);
    digitalWrite(Backward_Left, HIGH);
  }
}

void Right_Motor(int pwm, boolean forward) {
  analogWrite(PWM_Right, pwm); //set pwm control, 0 for stop, and 255 for maximum speed
  if (forward)  {
    digitalWrite(Forward_Right, LOW);
    digitalWrite(Backward_Right, HIGH);
  }
  else  {
    digitalWrite(Forward_Right, HIGH);
    digitalWrite(Backward_Right, LOW);
  }
}

void moveForward(int pwm) {
  Right_Motor(pwm, true);
  Left_Motor(pwm, true);
}

void moveBack(int pwm) {
  Right_Motor(pwm, false);
  Left_Motor(pwm, false);
}

void turnLeft(int pwm) {
  Right_Motor(pwm, true);
  Left_Motor(pwm, false);
}

void turnRight(int pwm) {
  Right_Motor(pwm, false);
  Left_Motor(pwm, true);
}

void Brush_Motor(int pwm, boolean forward) {
  analogWrite(PWM_Brush, pwm); //set pwm control, 0 for stop, and 255 for maximum speed
  if (forward)  {
    digitalWrite(Forward_Brush, HIGH);
    digitalWrite(Backward_Brush, LOW);
  }
  else {
    digitalWrite(Forward_Brush, LOW);
    digitalWrite(Backward_Brush, HIGH);
  }
}

void isBall() {
  float meas = pixy.ccc.blocks[0].m_width / pixy.ccc.blocks[0].m_height;
  if (meas < 2 && meas > 0.5) {
    obj_round = true;
  }
  else {
    obj_round = false;
  }
}

void loop() {
  //Stop after 180 seconds
  if (millis () > 180000) {
    moveForward(0);
    stopTime = 1;
  }

  if (stopTime == 0) {
    digitalWrite(trigPinLeft, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinLeft, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinLeft, LOW);
    distanceLeft  = pulseIn(echoPinLeft, HIGH) / 58.00;    // Scales the value in cm

    digitalWrite(trigPinFront, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPinFront, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPinFront, LOW);
    distanceFront = pulseIn(echoPinFront, HIGH) / 58.00;    // Scales the value in cm

    /*
      digitalWrite(trigPinRight, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPinRight, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPinRight, LOW);
      distanceRight = pulseIn(echoPinRight, HIGH) / 58.00;    // Scales the value in cm
    */

    Serial.print("Left: ");
    Serial.println(distanceLeft);

    Serial.print("Front: ");
    Serial.println(distanceFront);

    /*
      Serial.print("Front: ");
      Serial.println(distanceRight);
    */

    Brush_Motor(pwmBrush, true);

    if (distanceFront < avoidingDistanceFront) {
      if (distanceLeft < avoidingDistanceSide) {
        moveBack(pwmSlow);
        delay(500);
        turnRight(pwmMove);
        delay(1300);
      }
      else {
        moveBack(pwmSlow);
        delay(500);
        turnLeft(pwmMove);
        delay(1300);
      }
    }
    else {
      pixy.ccc.getBlocks();   // grab blocks!
      // isBall();
      if (pixy.ccc.numBlocks) // If there are detect blocks and the objects are round
      {
        if (pixy.ccc.blocks[0].m_x > field_width / 2 + margin) {
          Serial.println("L'objet est vers la droite!");
          turnRight(pwmMove);
        }
        else if (pixy.ccc.blocks[0].m_x < field_width / 2 - margin) {
          Serial.println("L'objet est vers la gauche!");
          turnLeft(pwmMove);
        }
        else {
          Serial.println("L'objet est au milieu!");
          moveForward(pwmMove);
          delay(200);
        }
      }
      else {
        moveForward(pwmMove);
        delay(500);
      }
    }
  }
}
