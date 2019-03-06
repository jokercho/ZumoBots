// Do not remove the include below
#include "Zumo.h"

//#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <NewPing.h>


//#define LOG_SERIAL // write log output to serial port

#define LED 13
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12

// Reflectance Sensor Setting
#define NUM_SENSORS 2
unsigned int sensor_values[NUM_SENSORS];
// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1000// microseconds 100/1200/2000
ZumoReflectanceSensorArray sensors(QTR_NO_EMITTER_PIN);

// Motor Settings
ZumoMotors motors;

#define SEARCH_SPEED      250
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define RIGHT_TRIGGER_PIN  2
#define RIGHT_ECHO_PIN     A1
#define LEFT_TRIGGER_PIN  A4
#define LEFT_ECHO_PIN     A5
#define CENTER_TRIGGER_PIN  6
#define CENTER_ECHO_PIN     A2
#define RIGHT_SENSOR 0
#define LEFT_SENSOR 1
#define CENTER_SENSOR 2

#define RIGHT 1
#define LEFT -1
#define DISTANCE_LIMT 600

enum ForwardSpeed { SearchSpeed, SustainedSpeed, FullSpeed };
ForwardSpeed _forwardSpeed;  // current forward speed setting



int rotation = RIGHT;
//int turn_speed = 250;

NewPing leftSensor(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, DISTANCE_LIMT/10);
NewPing rightSensor(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, DISTANCE_LIMT/10);
NewPing centerSensor(CENTER_TRIGGER_PIN, CENTER_ECHO_PIN, DISTANCE_LIMT/10);


// forward declaration
void setForwardSpeed(ForwardSpeed speed);

void waitForButtonAndCountDown(bool restarting);

int getForwardSpeed();

long detectObject(int);


void calibrateSensors();

void setup()
{
pinMode(RIGHT_TRIGGER_PIN, OUTPUT);
pinMode(RIGHT_ECHO_PIN, INPUT);
pinMode(LEFT_TRIGGER_PIN, OUTPUT);
pinMode(LEFT_ECHO_PIN, INPUT);
pinMode(CENTER_TRIGGER_PIN, OUTPUT);
pinMode(CENTER_ECHO_PIN, INPUT);


#ifdef LOG_SERIAL
  Serial.begin(115200);
#endif

  byte pins[] = {4,5};
  sensors.init(pins, 2);

  // uncomment if necessary to correct motor directions
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);

  pinMode(LED, HIGH);
  //buzzer.playMode(PLAY_AUTOMATIC);
  //calibrateSensors();
  waitForButtonAndCountDown(false);
  /*motors.setSpeeds(100,100);
  delay(1000);
  motors.setSpeeds(-100,-100);
  delay(1000);
  motors.setSpeeds(0,0);*/
}

void waitForButtonAndCountDown(bool restarting)
{
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#endif

  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);

  delay(5000);

  _forwardSpeed = SearchSpeed;
}

void loopTest()
{
	//sensors.read(sensor_values);

	unsigned int distanceObjRight = detectObject(RIGHT_SENSOR);
	unsigned int distanceObjLeft = detectObject(LEFT_SENSOR);
	unsigned int distanceObjCenter = detectObject(CENTER_SENSOR);
	if(distanceObjLeft > DISTANCE_LIMT || distanceObjLeft == 0) distanceObjLeft = DISTANCE_LIMT;
	if(distanceObjRight > DISTANCE_LIMT || distanceObjRight == 0) distanceObjRight = DISTANCE_LIMT;
	if(distanceObjCenter > DISTANCE_LIMT || distanceObjCenter == 0) distanceObjCenter = DISTANCE_LIMT;


	/*if( (distanceObjLeft < 600) && (distanceObjRight>distanceObjLeft) )
	{
		if((distanceObjRight - distanceObjLeft) < 10)
			motors.setSpeeds(0,0);
		else
		    motors.setSpeeds(SEARCH_SPEED * LEFT, -(SEARCH_SPEED) * LEFT);
	}
	else if( (distanceObjRight < 600) && (distanceObjLeft > distanceObjRight) )
	{
		if((distanceObjLeft-distanceObjRight) < 10)
			motors.setSpeeds(0,0);
		else
			motors.setSpeeds(SEARCH_SPEED * RIGHT, -(SEARCH_SPEED) * RIGHT);
	}*/


	#ifdef LOG_SERIAL
		Serial.print("LeftSensor: ");
		Serial.println(distanceObjLeft);
		Serial.print("RightSensor: ");
		Serial.println(distanceObjRight);
		Serial.print("CenterSensor: ");
		Serial.println(distanceObjCenter);
		/*Serial.print("Sensor[0]:");
		Serial.println(sensor_values[0]);
		Serial.print("Sensor[2]:");
		Serial.println(sensor_values[2]);
		Serial.print("Sensor[5]:");
		Serial.println(sensor_values[5]);*/
		delay(1000);
	#endif


}

void loop()
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown(true);
  }
  int distanceObjRight = detectObject(RIGHT_SENSOR);
  int distanceObjLeft = detectObject(LEFT_SENSOR);
  int distanceObjCenter = detectObject(CENTER_SENSOR);
  //if(distanceObjLeft > DISTANCE_LIMT || distanceObjLeft < 0) distanceObjLeft = DISTANCE_LIMT;
  //if(distanceObjRight > DISTANCE_LIMT || distanceObjRight < 0) distanceObjRight = DISTANCE_LIMT;
  if(distanceObjLeft == 0) distanceObjLeft = DISTANCE_LIMT;
  if(distanceObjRight == 0) distanceObjRight = DISTANCE_LIMT;
  if(distanceObjCenter == 0) distanceObjCenter = DISTANCE_LIMT;

  sensors.read(sensor_values);

  if( ((sensor_values[0] < QTR_THRESHOLD) && (sensor_values[1] < QTR_THRESHOLD)) )
  {
	  motors.setSpeeds(-100, -400);
	  delay(500);
	  sensors.read(sensor_values);
  }

  if(( (distanceObjLeft < DISTANCE_LIMT)||(distanceObjRight < DISTANCE_LIMT) || (distanceObjCenter < 100)) && (sensor_values[0] > QTR_THRESHOLD) && (sensor_values[1] > QTR_THRESHOLD))
  {
	setForwardSpeed(FullSpeed);
	int speed = getForwardSpeed();
	if(distanceObjRight>distanceObjLeft)
	{
		rotation = LEFT;
	  	if(((distanceObjRight - distanceObjLeft) < 10) || (distanceObjCenter < 100) )
			motors.setSpeeds(speed, speed);
		else
			motors.setSpeeds(speed-200, speed);
	}
	else if(distanceObjLeft>distanceObjRight)
	{
		rotation = RIGHT;
		if( ((distanceObjLeft - distanceObjRight) < 10) || (distanceObjCenter < 100))
			motors.setSpeeds(speed, speed);
		else
			motors.setSpeeds(speed, speed-200);
	}

	//start_turn_time = 0;
  }
  else
  {
	setForwardSpeed(SearchSpeed);
	int speed = getForwardSpeed();
	motors.setSpeeds(speed*rotation, -speed*rotation);
  }


  if (sensor_values[0] < QTR_THRESHOLD)
  {
	  rotation = RIGHT;
  }
  else if (sensor_values[1] < QTR_THRESHOLD)
  {
	  rotation = LEFT;
  }

}

void setForwardSpeed(ForwardSpeed speed)
{
  _forwardSpeed = speed;
}

int getForwardSpeed()
{
  int speed;
  switch (_forwardSpeed)
  {
    case FullSpeed:
      speed = FULL_SPEED;
      break;
    case SustainedSpeed:
      speed = SUSTAINED_SPEED;
      break;
    default:
      speed = SEARCH_SPEED;
      break;
  }
  return speed;
}



long detectObject(int sensor)
{
	/*int triggerPin = LEFT_TRIGGER_PIN;
	int echoPin = LEFT_ECHO_PIN;
	if(sensor == RIGHT_SENSOR)
	{
		triggerPin = RIGHT_TRIGGER_PIN;
		echoPin = RIGHT_ECHO_PIN;
	}

	digitalWrite(triggerPin, LOW);
	delayMicroseconds(2);
	digitalWrite(triggerPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(triggerPin, LOW);
	long detectedInMm = pulseIn(echoPin, HIGH) * 0.344 / 2.0;

	digitalWrite(triggerPin, HIGH);
	return detectedInMm;*/
	unsigned long timeMicroSec = 0;
	if(sensor == LEFT_SENSOR)
	{
		timeMicroSec = leftSensor.ping();
	}
	else if(sensor == RIGHT_SENSOR)
	{
		timeMicroSec = rightSensor.ping();
	}
	else if(sensor == CENTER_SENSOR)
	{
		timeMicroSec = centerSensor.ping();
	}

	long detectedInMm = timeMicroSec * 0.344 / 2.0;
	return detectedInMm;
}


void calibrateSensors()
{
  // read calibrated sensor values and obtain a measure of the line position.
  // Note: the values returned will be incorrect if the sensors have not been properly
  // calibrated during the calibration phase.
  //unsigned int position = reflectanceSensors.readLine(sensorValues);
	while(true)
	{
	  sensors.read(sensor_values);

	  // To get raw sensor values instead, call:
	  //reflectanceSensors.read(sensorValues);

	  //for (byte i = 0; i < NUM_SENSORS; i++)
	  {
		  Serial.print("Sensor Left :");
		  Serial.println(sensor_values[0]);
		  Serial.print("Sensor Right:");
		  Serial.println(sensor_values[1]);
		  //Serial.print("Sensor[5]:");
		  //Serial.println(sensor_values[5]);

	  }
	  //Serial.print("    ");
	  //Serial.println(position);

	  delay(1000);
	}
}
