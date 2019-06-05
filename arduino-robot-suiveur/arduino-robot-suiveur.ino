/*
  (EN) line follower robot
  (FR) Robot suiveur de ligne

  by ArnauldDev and jfcolombel
  for https://disciplines.ac-toulouse.fr/sii/concours-roboteck

  TODO: Add potentiometer for adjust max speed
*/

/* Robot starter cord */
const int sensorStarterPin = A1; // cordon de démarrage du robot

/* Robot stop at the end of course */
const int sensorStopPin = 8; // capteur de fin de course

/* Infrared line sensor */
const int lineSensorLeftPin = 4;  // Left
const int lineSensorRightPin = 5; // Right

/* Direction Motor */
#define FORWARD 1
#define BACKWARD 0

/* Left Motor */
const int motorLeftAIN1Pin = 6;         // PWM
const int motorLeftAIN2Pin = 7;         // direction
const int motorLeftSpeedAdjustPin = A0; // read potentiometre analog for motor speed adjust

/* Right Motor */
const int motorRightBIN1Pin = 3; // PWM
const int motorRightBIN2Pin = 4; // direction

/* Function prototype for Motor */
void freeMotorLeft();
void MotorLeft(uint8_t motor_dir, uint8_t motor_speed);
void stopMotorLeft(); // Frein moteur

void freeMotorRight();
void MotorRight(uint8_t motor_dir, uint8_t motor_speed);
void stopMotorRight(); // Frein moteur

/** Arduino section **********************************************************/
void setup()
{
  pinMode(sensorStarterPin, INPUT_PULLUP); // https://www.locoduino.org/spip.php?article122

  // initialize digital pin as an output.
  pinMode(motorLeftAIN2Pin, OUTPUT);
  pinMode(motorRightBIN2Pin, OUTPUT);
  digitalWrite(motorLeftAIN2Pin, LOW);  // turn the LED off by making the voltage LOW
  digitalWrite(motorRightBIN2Pin, LOW); // turn the LED off by making the voltage LOW

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  /* Starting here after pull the string */
  while (digitalRead(sensorStarterPin) == 0)
  {
    /* Do nothing until... */
  }
}

void loop()
{
  uint8_t motor_speed = 70;

  /* At startup the robot is placed in the center of the line => Go straight */
  robotGoStraight(motor_speed);

  /**
   * Line tracking, direction correction, correction of the direction
   * If the sensor on the left detects the line then it is that the robot is slightly to the right.
   * If the right sensor detects the line then it is that the robot slightly to the left.
   */
  // Suivi de ligne, correction de la direction
  // Si le capteur de gauche détecte la ligne alors c'est que le robot par légérement vers la droite.
  // Si le capteur de droite détecte la ligne alors c'est que le robot par légérement vers la gauche.
  if (robotIsOnTheRightOfLine())
  {
    robotTurnLeft(motor_speed);
  }

  if (robotIsOnTheLeftOfLine())
  {
    robotTurnRight(motor_speed);
  }

  /* End of the race */
  if (digitalRead(sensorStopPin) == 1)
  {
    robotStop();

    while (true)
    {
      /* Do nothing forever */
    }
  }

  delay(10);
}

/** Robot section **********************************************************/

/* Make the robot go straight */
// faire aller le robot tout droit
void robotGoStraight(uint8_t speed)
{
  MotorLeft(FORWARD, speed);
  MotorRight(FORWARD, speed);
}

void robotStop()
{
  MotorLeft(FORWARD, 0);
  MotorRight(FORWARD, 0);
  freeMotorLeft();
  freeMotorRight();
}

/* Turn Robot to the left direction */
void robotTurnLeft(uint8_t speed)
{
  uint8_t left_speed;
 
  if (speed > 50)
  {
    left_speed = speed - 50; // FIXME: Adjust
  }
  else
  {
    if (speed > 25)
    {
      left_speed = speed - 25; // FIXME: Adjust
    }
    else
    {
      left_speed = 0; // FIXME: Adjust
    }
  }

  MotorLeft(FORWARD, left_speed);
  MotorRight(FORWARD, speed);
}

/* Turn Robot to the right direction */
void robotTurnRight(uint8_t speed)
{
  uint8_t right_speed;

  if (speed > 50)
  {
    right_speed = speed - 50; // FIXME: Adjust
  }
  else
  {
    if (speed > 25)
    {
      right_speed = speed - 25; // FIXME: Adjust
    }
    else
    {
      right_speed = 0; // FIXME: Adjust
    }
  }

  MotorLeft(FORWARD, right_speed);
  MotorRight(FORWARD, speed);
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

// forward = marche FORWARD
void MotorLeft(uint8_t motor_dir, uint8_t motor_speed)
{
  if (motor_dir)
  {
    // avance
    digitalWrite(motorLeftAIN2Pin, LOW); // turn the direction forward
  }
  else
  {
    // recule
    digitalWrite(motorLeftAIN2Pin, HIGH); // turn the direction backforward
    motor_speed = 255 - motor_speed;
  }
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
// Free = roue libre
void freeMotorRight()
{
  analogWrite(motorRightBIN1Pin, 0);
  digitalWrite(motorRightBIN2Pin, LOW);
}

// forward = marche FORWARD
void MotorRight(uint8_t motor_dir, uint8_t motor_speed)
{
  if (motor_dir)
  {
    // avance
    digitalWrite(motorRightBIN2Pin, LOW); // turn the direction forward
  }
  else
  {
    // recule
    digitalWrite(motorRightBIN2Pin, HIGH); // turn the direction backforward
    motor_speed = 255 - motor_speed;
  }
  analogWrite(motorRightBIN1Pin, motor_speed);
}

// Frein
void stopMotorRight()
{
  analogWrite(motorRightBIN1Pin, 255);
  digitalWrite(motorRightBIN2Pin, HIGH);
}
