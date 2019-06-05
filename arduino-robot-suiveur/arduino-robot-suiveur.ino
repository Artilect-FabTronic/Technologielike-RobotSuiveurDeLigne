/*
  (EN) line follower robot
  (FR) Robot suiveur de ligne

  by ArnauldDev and jfcolombel
  for https://disciplines.ac-toulouse.fr/sii/concours-roboteck
*/

/* Direction Motor */
#define AVANT 1
#define ARRIERE 0

/* Left Motor */
const int motorLeftConsignePin = A0; // lecture potentiometre
const int motorLeftAIN1Pin = 6;      // PWM
const int motorLeftAIN2Pin = 7;      // direction

/* Right Motor */
const int motorRightConsignePin = A1; // lecture potentiometre
const int motorRightBIN1Pin = 3;      // PWM
const int motorRightBIN2Pin = 4;      // direction

/* Function prototype for Motor */
void freeMotorLeft();
void MotorLeft(uint8_t motor_dir, uint8_t motor_speed);
void stopMotorLeft(); // Frein moteur

void freeMotorRight();
void MotorRight(uint8_t motor_dir, uint8_t motor_speed);
void stopMotorRight(); // Frein moteur

void setup()
{
  pinMode(pushButton, INPUT);

  // initialize digital pin as an output.
  pinMode(motorLeftAIN2Pin, OUTPUT);
  pinMode(motorRightBIN2Pin, OUTPUT);
  digitalWrite(motorLeftAIN2Pin, LOW);  // turn the LED off by making the voltage LOW
  digitalWrite(motorRightBIN2Pin, LOW); // turn the LED off by making the voltage LOW

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop()
{
  /* Left Motor Task */
  int speed_motor_left = analogRead(motorLeftConsignePin);   // reads the value of the potentiometer (value between 0 and 1023)
  speed_motor_left = map(speed_motor_left, 0, 1023, 0, 255); // scale it to use it with the servo (value between 0 and 180)

  /* Right Motor Task */
  int speed_motor_right = analogRead(motorRightConsignePin);   // reads the value of the potentiometer (value between 0 and 1023)
  speed_motor_right = map(speed_motor_right, 0, 1023, 0, 255); // scale it to use it with the servo (value between 0 and 180)

  /* Change Motor direction */
  // sets PWM the value (range from 0 to 255):
  // and print motor left and motor right speed as : "Avant: 127, 255"
  if (digitalRead(pushButton))
  {
    Serial.print("Avant: "); // print out speed value
    MotorLeft(AVANT, speed_motor_left);
    MotorRight(AVANT, speed_motor_right);
  }
  else
  {
    Serial.print("Recul: "); // print out speed value
    MotorLeft(ARRIERE, speed_motor_left);
    MotorRight(ARRIERE, speed_motor_right);
  }

  Serial.print(speed_motor_left);    // print out speed value
  Serial.print(", ");                // print separator
  Serial.println(speed_motor_right); // print out speed value

  delay(10); // waits for the servo to get there
}

/* Left Motor */
// Free = roue libre
void freeMotorLeft()
{
  analogWrite(motorLeftAIN1Pin, 0);
  digitalWrite(motorLeftAIN2Pin, LOW);
}

// forward = marche avant
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

/* Right Motor */
// Free = roue libre
void freeMotorRight()
{
  analogWrite(motorRightBIN1Pin, 0);
  digitalWrite(motorRightBIN2Pin, LOW);
}

// forward = marche avant
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
