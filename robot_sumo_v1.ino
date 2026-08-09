/*
====================================================
SUMO ROBOT V5
RESPONSIVE ESCAPE + TARGET MEMORY + FSM SEARCH
====================================================
*/

// ================= MOTOR =================

#define ENA 14
#define IN1 26
#define IN2 27

#define ENB 25
#define IN3 32
#define IN4 33

// ================= IR =================

#define IR_BACK 35
#define IR_RIGHT 34
#define IR_LEFT 21
#define IR_FRONT_RIGHT 23
#define IR_FRONT_LEFT 22

// ================= HC-SR04 =================

#define TRIG_RIGHT 5
#define ECHO_RIGHT 17

#define TRIG_LEFT 18
#define ECHO_LEFT 19

// ================= PARAMETER =================

const int PWM_ESCAPE_BACK = 180;
const int PWM_ESCAPE_TURN = 190;
const int PWM_SEARCH = 130;
const int PWM_SEARCH_ROTATE = 135;
const int PWM_TRACK = 190;
const int PWM_APPROACH = 180;
const int PWM_ATTACK = 230;

const int MIN_TARGET_RANGE = 4;
const int TRACK_RANGE = 120;
const int ATTACK_RANGE = 30;

const unsigned long ULTRASONIC_TIMEOUT_US = 8000;
const unsigned long ULTRASONIC_INTERVAL_US = 12000;
const unsigned long SENSOR_FRESH_TIME = 80;
const unsigned long TARGET_HOLD_TIME = 350;

const unsigned long SEARCH_FORWARD_TIME = 60;
const unsigned long SEARCH_ROTATE_TIME = 140;
const bool DEBUG_DISTANCE = true;
const unsigned long DEBUG_INTERVAL = 200;

const unsigned long ESCAPE_FRONT_MIN_BACK_TIME = 100;
const unsigned long ESCAPE_FRONT_MAX_BACK_TIME = 180;
const unsigned long ESCAPE_FRONT_BRAKE_TIME = 40;
const unsigned long REAR_CONFIRM_TIME = 30;
const unsigned long ESCAPE_FRONT_TURN_TIME = 900;
const unsigned long ESCAPE_SIDE_TURN_TIME = 280;
const unsigned long ESCAPE_BACK_FORWARD_TIME = 140;

// ================= SENSOR DATA =================

struct LineSensors
{
  bool frontLeft;
  bool frontRight;
  bool left;
  bool right;
  bool back;
};

// ================= FSM =================

enum SearchState
{
  SEARCH_FORWARD,
  SEARCH_ROTATE
};

enum EscapeState
{
  ESCAPE_NONE,
  ESCAPE_FRONT_BRAKE,
  ESCAPE_FRONT_BACK,
  ESCAPE_FRONT_TURN,
  ESCAPE_SIDE_TURN,
  ESCAPE_BACK_FORWARD
};

enum Direction
{
  DIR_LEFT,
  DIR_RIGHT
};

enum LastSeen
{
  NONE,
  LEFT,
  RIGHT
};

SearchState searchState = SEARCH_ROTATE;
EscapeState escapeState = ESCAPE_NONE;
Direction escapeDirection = DIR_RIGHT;
LastSeen lastSeen = NONE;
bool targetWasCentered = false;
bool targetWasClose = false;

unsigned long searchTimer = 0;
unsigned long escapeTimer = 0;
unsigned long lastTargetTime = 0;
unsigned long debugTimer = 0;
unsigned long ultrasonicTimer = 0;
unsigned long leftDistanceTime = 0;
unsigned long rightDistanceTime = 0;
unsigned long rearWarningTimer = 0;

long lastLeftDist = 999;
long lastRightDist = 999;
bool readLeftNext = true;

// =================================================
// MOTOR
// =================================================

void stopMotor()
{
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void brakeMotor()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);
}

void forward(int pwm)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, pwm);
  ledcWrite(ENB, pwm);
}

void backward(int pwm)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, pwm);
  ledcWrite(ENB, pwm);
}

void rotateLeft(int pwm)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, pwm);
  ledcWrite(ENB, pwm);
}

void rotateRight(int pwm)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, pwm);
  ledcWrite(ENB, pwm);
}

void rotate(Direction direction, int pwm)
{
  if(direction == DIR_LEFT)
    rotateLeft(pwm);
  else
    rotateRight(pwm);
}

// =================================================
// SENSOR
// =================================================

bool lineDetected(int pin)
{
  int detectedCount = 0;

  for(int i = 0; i < 3; i++)
  {
    if(digitalRead(pin) == LOW)
      detectedCount++;

    delayMicroseconds(150);
  }

  return detectedCount >= 2;
}

LineSensors readLineSensors()
{
  LineSensors line;

  line.frontLeft = lineDetected(IR_FRONT_LEFT);
  line.frontRight = lineDetected(IR_FRONT_RIGHT);
  line.left = lineDetected(IR_LEFT);
  line.right = lineDetected(IR_RIGHT);
  line.back = lineDetected(IR_BACK);

  return line;
}

long readDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration =
      pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT_US);

  if(duration == 0)
    return 999;

  long distance = duration * 0.034 / 2;

  if(distance < MIN_TARGET_RANGE || distance > TRACK_RANGE)
    return 999;

  return distance;
}

void updateUltrasonic()
{
  unsigned long nowUs = micros();

  if(nowUs - ultrasonicTimer < ULTRASONIC_INTERVAL_US)
    return;

  if(readLeftNext)
  {
    lastLeftDist = readDistance(TRIG_LEFT, ECHO_LEFT);
    leftDistanceTime = millis();
  }
  else
  {
    lastRightDist = readDistance(TRIG_RIGHT, ECHO_RIGHT);
    rightDistanceTime = millis();
  }

  readLeftNext = !readLeftNext;
  ultrasonicTimer = micros();
}

// =================================================
// ESCAPE
// =================================================

void resetSearch()
{
  searchState = SEARCH_FORWARD;
  searchTimer = millis();
}

void beginEscape(EscapeState state, Direction direction)
{
  escapeState = state;
  escapeDirection = direction;
  escapeTimer = millis();
  rearWarningTimer = 0;
  lastSeen = NONE;
}

bool handleEscape(const LineSensors &line)
{
  unsigned long now = millis();

  if(escapeState == ESCAPE_NONE)
  {
    if(line.frontLeft || line.frontRight)
    {
      // Belok menjauhi sisi depan yang menyentuh garis.
      Direction turnDirection =
          (line.frontRight && !line.frontLeft) ? DIR_LEFT : DIR_RIGHT;

      beginEscape(ESCAPE_FRONT_BRAKE, turnDirection);
      Serial.println("ESCAPE FRONT");
    }
    else if(line.left)
    {
      beginEscape(ESCAPE_SIDE_TURN, DIR_RIGHT);
      Serial.println("ESCAPE LEFT");
    }
    else if(line.right)
    {
      beginEscape(ESCAPE_SIDE_TURN, DIR_LEFT);
      Serial.println("ESCAPE RIGHT");
    }
    else if(line.back)
    {
      beginEscape(ESCAPE_BACK_FORWARD, DIR_RIGHT);
      Serial.println("ESCAPE BACK");
    }
    else
    {
      return false;
    }
  }

  switch(escapeState)
  {
    case ESCAPE_FRONT_BRAKE:
      brakeMotor();

      if(now - escapeTimer >= ESCAPE_FRONT_BRAKE_TIME)
      {
        escapeState = ESCAPE_FRONT_BACK;
        escapeTimer = now;
      }
      break;

    case ESCAPE_FRONT_BACK:
      if(line.back && rearWarningTimer == 0)
        rearWarningTimer = now;
      else if(!line.back)
        rearWarningTimer = 0;

      if(rearWarningTimer != 0 &&
         now - rearWarningTimer >= REAR_CONFIRM_TIME)
      {
        stopMotor();
        escapeState = ESCAPE_FRONT_TURN;
        escapeTimer = now;
      }
      else if((now - escapeTimer >= ESCAPE_FRONT_MIN_BACK_TIME &&
               !line.frontLeft && !line.frontRight) ||
              now - escapeTimer >= ESCAPE_FRONT_MAX_BACK_TIME)
      {
        escapeState = ESCAPE_FRONT_TURN;
        escapeTimer = now;
        rotate(escapeDirection, PWM_ESCAPE_TURN);
      }
      else
      {
        backward(PWM_ESCAPE_BACK);
      }
      break;

    case ESCAPE_FRONT_TURN:
      rotate(escapeDirection, PWM_ESCAPE_TURN);

      // Selesaikan satu rotasi tanpa di-reset oleh sensor garis lain.
      if(now - escapeTimer >= ESCAPE_FRONT_TURN_TIME)
      {
        escapeState = ESCAPE_NONE;
        resetSearch();
        Serial.println("ESCAPE TURN DONE");
      }
      break;

    case ESCAPE_SIDE_TURN:
      rotate(escapeDirection, PWM_ESCAPE_TURN);

      if(now - escapeTimer >= ESCAPE_SIDE_TURN_TIME)
      {
        escapeState = ESCAPE_NONE;
        resetSearch();
      }
      break;

    case ESCAPE_BACK_FORWARD:
      if(line.frontLeft || line.frontRight)
      {
        stopMotor();
        escapeState = ESCAPE_FRONT_TURN;
        escapeTimer = now;
      }
      else
      {
        forward(PWM_ESCAPE_BACK);

        if(now - escapeTimer >= ESCAPE_BACK_FORWARD_TIME && !line.back)
        {
          escapeState = ESCAPE_NONE;
          resetSearch();
        }
      }
      break;

    case ESCAPE_NONE:
      break;
  }

  return true;
}

// =================================================

void setup()
{
  Serial.begin(115200);

  pinMode(IR_BACK, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);
  pinMode(IR_FRONT_LEFT, INPUT);

  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  stopMotor();
  Serial.println("STARTING...");
  delay(1000);

  searchTimer = millis();
}

// =================================================

void loop()
{
  LineSensors line = readLineSensors();

  if(handleEscape(line))
    return;

  // Baca satu ultrasonik bergantian agar IR kembali diperiksa setiap loop.
  updateUltrasonic();

  unsigned long now = millis();
  long leftDist = lastLeftDist;
  long rightDist = lastRightDist;

  bool leftTarget =
      leftDist != 999 &&
      leftDistanceTime != 0 &&
      now - leftDistanceTime <= SENSOR_FRESH_TIME;

  bool rightTarget =
      rightDist != 999 &&
      rightDistanceTime != 0 &&
      now - rightDistanceTime <= SENSOR_FRESH_TIME;

  if(DEBUG_DISTANCE && now - debugTimer >= DEBUG_INTERVAL)
  {
    Serial.print("DIST L:");
    Serial.print(leftDist);
    Serial.print(" R:");
    Serial.println(rightDist);
    debugTimer = now;
  }

  if(leftTarget || rightTarget)
  {
    lastTargetTime = now;
    resetSearch();

    if(leftTarget)
      lastLeftDist = leftDist;

    if(rightTarget)
      lastRightDist = rightDist;
  }

  if(leftTarget && rightTarget)
  {
    targetWasCentered = true;
    targetWasClose =
        leftDist < ATTACK_RANGE && rightDist < ATTACK_RANGE;
    lastSeen = (leftDist < rightDist) ? LEFT : RIGHT;

    if(targetWasClose)
    {
      Serial.println("ATTACK");
      forward(PWM_ATTACK);
    }
    else
    {
      Serial.println("TARGET FRONT");
      forward(PWM_APPROACH);
    }

    return;
  }

  if(leftTarget)
  {
    targetWasCentered = false;
    targetWasClose = false;
    lastSeen = LEFT;
    Serial.println("TRACK LEFT");
    rotateLeft(PWM_TRACK);
    return;
  }

  if(rightTarget)
  {
    targetWasCentered = false;
    targetWasClose = false;
    lastSeen = RIGHT;
    Serial.println("TRACK RIGHT");
    rotateRight(PWM_TRACK);
    return;
  }

  // Abaikan dropout ultrasonik singkat agar serangan tidak langsung putus.
  if(lastTargetTime != 0 && now - lastTargetTime < TARGET_HOLD_TIME)
  {
    Serial.println("TARGET HOLD");

    if(targetWasCentered)
      forward(targetWasClose ? PWM_ATTACK : PWM_APPROACH);
    else if(lastSeen == LEFT)
      rotateLeft(PWM_TRACK);
    else if(lastSeen == RIGHT)
      rotateRight(PWM_TRACK);
    else
      forward(PWM_APPROACH);

    return;
  }

  switch(searchState)
  {
    case SEARCH_FORWARD:
      forward(PWM_SEARCH);

      if(now - searchTimer >= SEARCH_FORWARD_TIME)
      {
        searchState = SEARCH_ROTATE;
        searchTimer = now;
      }
      break;

    case SEARCH_ROTATE:
      if(lastSeen == RIGHT)
        rotateRight(PWM_SEARCH_ROTATE);
      else
        rotateLeft(PWM_SEARCH_ROTATE);

      if(now - searchTimer >= SEARCH_ROTATE_TIME)
      {
        searchState = SEARCH_FORWARD;
        searchTimer = now;
      }
      break;
  }
}