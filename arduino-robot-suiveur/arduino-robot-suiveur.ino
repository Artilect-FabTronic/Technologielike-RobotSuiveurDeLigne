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

enum robot_fsm_enum
{
  ROBOT_START_WAITING,
  ROBOT_GO_STRAIGHT,
  ROBOT_ADJUST_LEFT,
  ROBOT_ADJUST_RIGHT,
  ROBOT_STOP
};

robot_fsm_enum fsm_robot_state = ROBOT_START_WAITING;

/* General_Motor_speed */
//int valspeedPotentiometre = 0; // Initialise la variable qui va recueillir la valeur du potentiomètre
const int motorTurnSpeedAdjustPin = A0;  // read potentiometre analog for motor speed adjust to max turn speed
const int motorGoStraightAdjustPin = A1; // read potentiometre analog for General_Motor_speed adjust for left and right motors
const int motorSpeedAdjustPin = A2;      // read potentiometre analog for General_Motor_speed adjust for left and right motors

int turn_speed_motor = 0;
int motor_speed = 0;
int go_straight_speed_motor = 0;

/* Robot starter cord */
const int sensorStarterPin = A5; // cordon de démarrage du robot

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

/* Function prototype for temporisation */
void tempoLoading(unsigned long duration);
void tempoTask();

/** Arduino section **********************************************************/
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  pinMode(sensorStarterPin, INPUT_PULLUP); // https://www.locoduino.org/spip.php?article122

  // initialize digital pin as an output.
  pinMode(motorLeftAIN2Pin, OUTPUT);
  pinMode(motorRightBIN2Pin, OUTPUT);
  digitalWrite(motorLeftAIN2Pin, LOW);  // turn the LED off by making the voltage LOW
  digitalWrite(motorRightBIN2Pin, LOW); // turn the LED off by making the voltage LOW
}

void loop()
{
  /* Adjust general speed motor */
  motor_speed = analogRead(motorSpeedAdjustPin);   // reads the value of the potentiometer (value between 0 and 1023)
  motor_speed = map(motor_speed, 0, 1023, 0, 255); // scale it to use it with the servo (value between 0 and 255)

  /* Difference motorSpeedAdjust in order to go straight */
  go_straight_speed_motor = analogRead(motorGoStraightAdjustPin);          // reads the value of the potentiometer (value between 0 and 1023)
  go_straight_speed_motor = map(go_straight_speed_motor, 0, 1023, 0, 100); // scale it to use it with the servo (value between 0 and 180)

  /* Adjust turn speed */
  turn_speed_motor = analogRead(motorTurnSpeedAdjustPin);    // reads the value of the potentiometer (value between 0 and 1023)
  turn_speed_motor = map(turn_speed_motor, 0, 1023, 0, 100); // scale it to use it with the servo (value between 0 and 180)

  // TODO: ajouter pour plus tard un afficheur 7 segment pour indiquer l'état en cours
  switch (fsm_robot_state)
  {
  case ROBOT_START_WAITING:
    Serial.println("[ROBOT_START_WAITING]");
    /* Waiting here until pull the string */
    if (digitalRead(sensorStarterPin) == true)
    {
      fsm_robot_state = ROBOT_GO_STRAIGHT;
    }
    break;

  case ROBOT_GO_STRAIGHT:
    Serial.println("[ROBOT_GO_STRAIGHT]");
    robotGoStraight(motor_speed);

    if (robotIsArriveAtTheEndOfRace())
    {
      fsm_robot_state = ROBOT_STOP;
    }

    if (robotIsOnTheRightOfLine())
    {
      fsm_robot_state = ROBOT_ADJUST_LEFT;
    }

    if (robotIsOnTheLeftOfLine())
    {
      fsm_robot_state = ROBOT_ADJUST_RIGHT;
    }
    break;

  case ROBOT_ADJUST_LEFT:
    Serial.println("[ROBOT_ADJUST_LEFT]");
    robotTurnLeft(motor_speed);

    if (robotIsOnTheCenterOfLine())
    {
      fsm_robot_state = ROBOT_GO_STRAIGHT;
    }

    if (robotIsArriveAtTheEndOfRace())
    {
      fsm_robot_state = ROBOT_STOP;
    }
    break;

  case ROBOT_ADJUST_RIGHT:
    Serial.println("[ROBOT_ADJUST_RIGHT]");
    robotTurnRight(motor_speed);

    if (robotIsOnTheCenterOfLine())
    {
      fsm_robot_state = ROBOT_GO_STRAIGHT;
    }

    if (robotIsArriveAtTheEndOfRace())
    {
      fsm_robot_state = ROBOT_STOP;
    }
    break;

  case ROBOT_STOP:
    Serial.println("[ROBOT_STOP]");

    /* End of the race */
    robotStop();

    while (true)
    {
      /* Do nothing forever */
    }
    break;

  default:
    // if nothing else matches, do the default
    Serial.println("[default]");
    fsm_robot_state = ROBOT_START_WAITING;
  }
}

/** Tempo section **********************************************************/

// Chargement d'une temporisation
void tempoLoading(unsigned long duration)
{
}

void tempoTask()
{
  unsigned long currentMillis = millis();

  // if (currentMillis - previousMillis >= interval)
  // {
  //   // save the last time
  //   previousMillis = currentMillis;
  // }
}

/** Robot section **********************************************************/

/* Make the robot go straight */
// faire aller le robot tout droit
void robotGoStraight(uint8_t speed)
{
  uint8_t left_speed;
  uint8_t right_speed;

  left_speed = speed; // speed * (1 - go_straight_speed_motor / 100);
                      //  right_speed = speed * (1 - go_straight_speed_motor / 100);
  right_speed = (float)speed * (1.0 - ((float)go_straight_speed_motor / 100.0));
  Serial.print("Adjust GoStraight: ");
  Serial.print(speed);
  Serial.print(" -> ");
  Serial.println(right_speed);

  MotorLeftForward(left_speed);
  MotorRightForward(right_speed);
  // MotorLeft(FORWARD, left_speed);
  // MotorRight(FORWARD, right_speed);
}

void robotStop()
{
  MotorLeftForward(0);
  MotorRightForward(0);
  // MotorLeft(FORWARD, 0);
  // MotorRight(FORWARD, 0);

  freeMotorLeft();
  freeMotorRight();
}

/* Turn Robot to the left direction */
void robotTurnLeft(uint8_t speed)
{
  uint8_t left_speed;

  // left_speed = speed * (1 - turn_speed_motor / 100);
  left_speed = (float)speed * (1.0 - ((float)turn_speed_motor / 100.0));
  Serial.print("Adjust TurnLeft: ");
  Serial.print(speed);
  Serial.print(" -> ");
  Serial.println(left_speed);

  MotorLeftForward(left_speed);
  MotorRightForward(speed);
  // MotorLeft(FORWARD, left_speed);
  // MotorRight(FORWARD, speed);
}

//if (speed > 50)
// {
// left_speed = speed - 50; // FIXME: Adjust
//}
//else
//{
//  if (speed > 25)
//  {
//    left_speed = speed - 25; // FIXME: Adjust
//  }
//  else
//  {
//    left_speed = 0; // FIXME: Adjust
//  }

// if (speed > turn_speed_motor)
// {
//   left_speed = speed - turn_speed_motor;
//     left_speed = speed * (1 - turn_speed_motor / 100);
//     }
// else
// {
//   left_speed = speed * (1 - turn_speed_motor / 100);

// }

/* Turn Robot to the right direction */
void robotTurnRight(uint8_t speed)
{
  uint8_t right_speed;

  right_speed = speed * (1 - turn_speed_motor / 100);
  MotorLeftForward(speed);
  MotorRightForward(right_speed);
  // MotorLeft(FORWARD, speed);
  // MotorRight(FORWARD, right_speed);
}

//if (speed > 50)
//{
//  right_speed = speed - 50; // FIXME: Adjust
//}
//else
//{
//  if (speed > 25)
//  {
//    right_speed = speed - 25; // FIXME: Adjust
//  }
//  else
//  {
//    right_speed = 0; // FIXME: Adjust
//  }
//}

// if (speed > turn_speed_motor)
// {
//   right_speed = speed - turn_speed_motor;
// }
// else
// {
//   right_speed = speed * (1 - turn_speed_motor / 100);
// }

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
  analogWrite(motorLeftAIN1Pin, 255);
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
  analogWrite(motorRightBIN1Pin, 255);
  digitalWrite(motorRightBIN2Pin, HIGH);
}
