#include <PID_v1.h>
#include <Servo.h>

#define STATUS_IDLE   0
#define STATUS_XPOS   1
#define STATUS_YPOS   2
#define STATUS_XPATH  3
#define STATUS_YPATH  4

#define XSERVO_PIN    5
#define YSERVO_PIN    6

#define OUTPUT_MIN    -127
#define OUTPUT_MAX    128
#define ANGLE_MIN     0
#define ANGLE_MAX     180

#define UPDATE_PERIOD_MS 1000

double xPos, xOutput, xPath;
double yPos, yOutput, yPath;
int status;
int tmpValue;
long updateMillis;

PID xPID(&xPos, &xOutput, &xPath, 0.1, 0.5, 0, DIRECT);
PID yPID(&yPos, &yOutput, &yPath, 0.1, 0.5, 1, DIRECT);
Servo xServo;
Servo yServo;

void readData()
{
  while (Serial.available())
  {
    int ch = Serial.read();
    switch (status)
    {
      case STATUS_IDLE:
        if (ch == 'C')
        {
          tmpValue = 0;
          status = STATUS_XPOS;
        }
        else if (ch == 'P')
        {
          tmpValue = 0;
          status = STATUS_XPATH;
        }
        break;
      case STATUS_XPOS:
        if (ch == ';')
        {
          xPos = tmpValue;
          tmpValue = 0;
          status = STATUS_YPOS;
        }
        else if (isDigit(ch))
        {
          tmpValue = (tmpValue*10) + (ch - '0');
        }
        else
          status= STATUS_IDLE;
        break;
      case STATUS_YPOS:
        if ((ch == '\r') || (ch == '\n'))
        {
          yPos = tmpValue;
          tmpValue = 0;
          status = STATUS_IDLE;
        }
        else if (isDigit(ch))
        {
          tmpValue = (tmpValue*10) + (ch - '0');
        }
        else
          status= STATUS_IDLE;
        break;
      case STATUS_XPATH:
        if (ch == ';')
        {
          xPath = tmpValue;
          tmpValue = 0;
          status = STATUS_YPATH;
        }
        else if (isDigit(ch))
        {
          tmpValue = (tmpValue*10) + (ch - '0');
        }
        else
          status= STATUS_IDLE;
        break;
      case STATUS_YPATH:
        if ((ch == '\r') || (ch == '\n'))
        {
          yPath = tmpValue;
          tmpValue = 0;
          status = STATUS_IDLE;
        }
        else if (isDigit(ch))
        {
          tmpValue = (tmpValue*10) + (ch - '0');
        }
        else
          status= STATUS_IDLE;
        break;
    }
  }
}

void writeServos()
{
  long delta = millis() - updateMillis;
  if (delta < UPDATE_PERIOD_MS)
    return;
    
  xPID.Compute();  
  yPID.Compute();  

  // map xOutput to angle
  double xAngle = map(xOutput, OUTPUT_MIN, OUTPUT_MAX, ANGLE_MIN, ANGLE_MAX);
  xServo.write(xAngle);

  Serial.print("X Servo: ");
  Serial.print(xPos);
  Serial.print(" - ");
  Serial.print(xPath);
  Serial.print(" -> ");
  Serial.print(xOutput);
  Serial.print(" -> ");
  Serial.print(xAngle);
  Serial.println();

  // map yOutput to angle
  double yAngle = map(yOutput, OUTPUT_MIN, OUTPUT_MAX, ANGLE_MIN, ANGLE_MAX);
  yServo.write(yAngle);

  Serial.print("Y Servo: ");
  Serial.print(yPos);
  Serial.print(" - ");
  Serial.print(yPath);
  Serial.print(" -> ");
  Serial.print(yOutput);
  Serial.print(" -> ");
  Serial.print(yAngle);
  Serial.println();
  Serial.println();

  updateMillis = millis();
}

void setup() {
  status = STATUS_IDLE;
  
  Serial.begin(115200);

  xPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  yPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);

  xServo.attach(XSERVO_PIN);
  yServo.attach(YSERVO_PIN);
 
  /*
  for (int angle=ANGLE_MIN; angle<ANGLE_MAX; angle++)
  {
    yServo.write(angle);
    delay(100);
  }
  for (int angle=ANGLE_MIN; angle<ANGLE_MAX; angle++)
  {
    xServo.write(angle);
    delay(100);
  }
  */
}

void loop() {
  readData();
  writeServos();
}
