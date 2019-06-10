/*
  (EN) Line follower robot
  (FR) Robot suiveur de ligne

  by ArnauldDev and jfcolombel
  for https://disciplines.ac-toulouse.fr/sii/concours-roboteck

  Line tracking, direction correction, correction of the direction
  If the sensor on the left detects the line then it is that the robot is slightly to the right.
  If the right sensor detects the line then it is that the robot slightly to the left.

  ## Suivi de ligne, correction de la direction
  Si le capteur de gauche détecte la ligne alors c'est que le robot par légérement vers la droite.
  Si le capteur de droite détecte la ligne alors c'est que le robot par légérement vers la gauche.
*/

/* At startup the robot is placed in the center of the line => Go straight */
const int motorSpeedAdjustPin = A0;      // read potentiometre analog for General_Motor_speed adjust for left and right motors

/* Robot stop at the end of course */
const int sensorStopPin = 8; // capteur de fin de course

/* Infrared line sensor */
const int lineSensorLeftPin = 2;  // Left
const int lineSensorRightPin = 5; // Right

/* Direction Motor */
#define FORWARD 1  // Forward and Brake/Slow decay Avant et frein
#define BACKWARD 0 // Backward and Coast/Fast decay Arrière et Roue libre

/* Left Motor */
const int motorLeftAIN1Pin = 6; // PWM
const int motorLeftAIN2Pin = 7; // direction

/* Right Motor */
const int motorRightBIN1Pin = 3; // PWM
const int motorRightBIN2Pin = 4; // direction

int motor_speed = 0;

/* Function prototype for Motor */
void freeMotorLeft();
void MotorLeftForward(uint8_t motor_speed);
void MotorLeft(uint8_t motor_dir, uint8_t motor_speed);
void stopMotorLeft(); // Frein moteur

void freeMotorRight();
void MotorRightForward(uint8_t motor_speed);
void MotorRight(uint8_t motor_dir, uint8_t motor_speed);
void stopMotorRight(); // Frein moteur

/* Function prototype for Robot */
bool robotIsOnTheCenterOfLine();     // le robot est au centre de la ligne
bool robotIsOnTheLeftOfLine();       // le robot est à gauche de la ligne
bool robotIsOnTheRightOfLine();      // le robot est à droite de la ligne
bool robotIsArriveAtTheEndOfRace();  // le robot est arrivée à la fin de la cours
void robotGoStraight(uint8_t speed); // faire aller le robot tout droit
void robotStop();                    // mettre à l'arret le robot
void robotTurnLeft(uint8_t speed);   // Turn Robot to the left direction
void robotTurnRight(uint8_t speed);  // Turn Robot to the right direction

/** Arduino section **********************************************************/
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  pinMode(sensorStopPin, INPUT_PULLUP);

  // initialize digital pin as an output.
  pinMode(motorLeftAIN2Pin, OUTPUT);
  pinMode(motorRightBIN2Pin, OUTPUT);
  digitalWrite(motorLeftAIN2Pin, LOW);  // turn the LED off by making the voltage LOW
  digitalWrite(motorRightBIN2Pin, LOW); // turn the LED off by making the voltage LOW

  robotStop();
  Serial.println("ROBOT_START_WAITING...");
  delay(5000);
  
//  while ((digitalRead(lineSensorLeftPin) == 0) && (digitalRead(lineSensorRightPin) == 0))
  while (robotIsOnTheLeftOfLine() || robotIsOnTheRightOfLine())
  {
    /* attendre que l'on soit sur la position de départ sur la ligne droite */
  }
}

void loop()
{
  /* Adjust general speed motor */
  motor_speed = analogRead(motorSpeedAdjustPin);  // read potentiometre analog for motor speed adjust to max turn speed
  motor_speed = 50 + map(motor_speed, 0, 1023, 0, 205); // scale it to use it with the servo (value between 0 and 255)
  Serial.print("motor_speed: "); // 121 est le max sinon ont sortie de la piste
  Serial.println(motor_speed);

  /* Aller tout droit */
  if (robotIsOnTheCenterOfLine()) {
    Serial.println("ROBOT_GO_STRAIGHT");
    //robotGoStraight(motor_speed);
    MotorLeftForward(motor_speed-25);
    MotorRightForward(motor_speed-25);
  }

  /* Tourner à droite */
  if (robotIsOnTheLeftOfLine()) {
    stopMotorLeft();
    Serial.println("ROBOT_TURN_RIGHT");
    //delay(50);
    MotorRightForward(motor_speed);
  }

  /* Tourner à gauche */
  if (robotIsOnTheRightOfLine()) {
    stopMotorRight();
    Serial.println("ROBOT_TURN_LEFT");
    //delay(50);
    MotorLeftForward(motor_speed);
  }

  /* Arreter le robot */
  if (robotIsArriveAtTheEndOfRace()) {
    robotStop();
    Serial.println("ROBOT_STOP");
    delay(10000);
  }
}

/** Robot section **********************************************************/

/* Make the robot go straight */
// faire aller le robot tout droit
void robotGoStraight(uint8_t speed)
{
  MotorLeftForward(speed);
  MotorRightForward(speed);
}

void robotStop()
{
//  MotorLeftForward(0);
//  MotorRightForward(0);
  freeMotorLeft();
  freeMotorRight();
}

/* The robot at the end of the race */
// le robot est arrivée à la fin de la cours
bool robotIsArriveAtTheEndOfRace()
{
  bool ret_value = false;

  if (digitalRead(sensorStopPin) == 1)
  {
    ret_value = true;
  }

  return ret_value;
}

/* The robot is on the center of the line */
// le robot est au centre de la ligne
bool robotIsOnTheCenterOfLine()
{
  bool ret_value = false;

  if ((digitalRead(lineSensorLeftPin) == 1) && (digitalRead(lineSensorRightPin) == 1))
  {
    ret_value = true;
  }

  return ret_value;
}

/* The robot is on the left of the line */
// le robot est à gauche de la ligne
bool robotIsOnTheLeftOfLine()
{
  bool ret_value = false;

  if ((digitalRead(lineSensorLeftPin) == 1) && (digitalRead(lineSensorRightPin) == 0))
  {
    ret_value = true;
  }

  return ret_value;
}

/* The robot is on the right of the line */
// le robot est à droite de la ligne
bool robotIsOnTheRightOfLine()
{
  bool ret_value = false;

  if ((digitalRead(lineSensorLeftPin) == 0) && (digitalRead(lineSensorRightPin) == 1))
  {
    ret_value = true;
  }

  return ret_value;
}

/** Motor section **********************************************************/

/* Left Motor */
// Free = roue libre
void freeMotorLeft()
{
  analogWrite(motorLeftAIN1Pin, 0);
  digitalWrite(motorLeftAIN2Pin, LOW);
}

// forward or backward = marche avant ou arrière
void MotorLeft(uint8_t motor_dir, uint8_t motor_speed)
{
  if (motor_dir)
  {
    // avance
    analogWrite(motorLeftAIN1Pin, motor_speed); // PWM
    digitalWrite(motorLeftAIN2Pin, LOW);        // turn the direction forward
  }
  else
  {
    // recule
    digitalWrite(motorLeftAIN1Pin, HIGH);       // turn the direction backforward
    analogWrite(motorLeftAIN2Pin, motor_speed); // PWM
  }
}

// forward = marche avant
void MotorLeftForward(uint8_t motor_speed)
{
  digitalWrite(motorLeftAIN2Pin, LOW); // turn the direction forward
  analogWrite(motorLeftAIN1Pin, motor_speed);
}

// Frein
void stopMotorLeft()
{
  digitalWrite(motorLeftAIN1Pin, HIGH);
  digitalWrite(motorLeftAIN2Pin, HIGH);
}

/* left wheel speed */
// vitesse roue gauche
uint8_t adjustMotorSpeedLeft(uint8_t motor_speed)
{
  uint8_t ret_speed = 0;

  // TODO:

  return ret_speed;
}

/* Right Motor */
// Free wheel = roue libre
void freeMotorRight()
{
  analogWrite(motorRightBIN1Pin, 0);
  digitalWrite(motorRightBIN2Pin, LOW);
}

// forward or backward = marche avant ou arrière
void MotorRight(uint8_t motor_dir, uint8_t motor_speed)
{
  if (motor_dir)
  {
    // avance
    analogWrite(motorRightBIN1Pin, motor_speed); // PWM
    digitalWrite(motorRightBIN2Pin, LOW);        // turn the direction forward
  }
  else
  {
    // recule
    digitalWrite(motorRightBIN1Pin, HIGH);       // turn the direction backforward
    analogWrite(motorRightBIN2Pin, motor_speed); // PWM
  }
}

// forward = marche avant
void MotorRightForward(uint8_t motor_speed)
{
  // avance
  digitalWrite(motorRightBIN2Pin, LOW); // turn the direction forward
  analogWrite(motorRightBIN1Pin, motor_speed);
}

// Frein
void stopMotorRight()
{
  digitalWrite(motorRightBIN1Pin, HIGH);
  digitalWrite(motorRightBIN2Pin, HIGH);
}
