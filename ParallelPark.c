//Final Project
//MTE 121
//Akam Dhillon, Kevin Kim, Sarah Elisabeth Cleghorn, Vedant Grover
//Parallel Parking Simulation

#define COLOR S1
#define US S4
#define GYRO S3

const int MAXDISTANCE = 80;  // Maximum width between cars
const float ENC_MULTIPLIER = 180 / (PI*2.75); //cm to encoder conversion
const int CLEARANCELENGTH = 800; //maximum distance in front of first object/car
const int WHEELGAP = 12; //distance between left and right wheels

//follow path follows black line and outputs the distance between the two objects
void followPath()
{
	while (SensorValue[COLOR] != (int)(colorBlack))
	{
		motor[motorD] = 1;
		motor[motorA] = 20;
		wait1Msec(40);
	}
	motor[motorA] = motor[motorD] = 0;
	wait1Msec(100);
	while (SensorValue[COLOR] == (int)(colorBlack))
	{
		motor[motorD] = 10;
		motor[motorA] = 5;
		wait1Msec(40);
	}
	motor[motorA] = motor[motorD] = 0;
	wait1Msec(100);
}

float objectDistance(float &sideClear)
{
	float initialDist = 0;
	float finalDist = 0;

	//Read first object
	while (SensorValue[US] > MAXDISTANCE)
	{
		followPath();
	}
	motor[motorA] = motor[motorD] = 0;
	wait1Msec(2000);
	sideClear = SensorValue[US];

	//drive until object no longer detected
	while (SensorValue[US] < MAXDISTANCE)
	{
		followPath();
	}
	motor[motorA] = motor[motorD] = 0;
	wait1Msec(2000);

	initialDist = nMotorEncoder[motorA];

	while (SensorValue[US] > MAXDISTANCE && fabs(nMotorEncoder[motorA] - initialDist) < CLEARANCELENGTH)
	{
		followPath();
	}
	motor[motorA] = motor[motorD] = 0;
	wait1Msec(2000);

	finalDist = nMotorEncoder[motorA];

	return fabs(initialDist - finalDist);

}

bool isAble(int clearance)
{
  displayString(5,"%f", clearance);
  // if clearance is smaller than minimum needed space than return false
  if (clearance < CLEARANCELENGTH)
  {
    return false;
  }
  else
  {
    return true;
  }
}

/*
Power Multiplier Function:
	1. input values
		a. distance from car beside
		b. distance between cars
	2. Do calculations
*/

float powerMultiplier(float sideClear, float clearance)
{
	float x = ((WHEELGAP / 2) + sideClear) / 2; //x length of chord
	float y = (clearance - 10) / 2; //y length of chord
	float turnChord = sqrt((x*x) + (y*y)); //chord length
	float turningRadius = sqrt(turnChord / (2 - sqrt(2))); //turning radius based on ready position of robot
	return (turningRadius + (WHEELGAP/2)) / turningRadius; //returns ratio of power between wheels
}

void rotateRobot(int angle, int motorPower)
{
	resetGyro(GYRO);
	if (angle > 0)
	{
		motor[motorA] = -motorPower;
		motor[motorD] = motorPower;
	}
	else
	{
		motor[motorA] = motorPower;
		motor[motorD] = -motorPower;
	}
	angle = abs(angle);

	while(abs(getGyroDegrees(GYRO)) < angle)
	{}
	motor[motorA] = motor[motorD] = 0;
}

void followPathAfter()
{
	rotateRobot(90, 10);

	motor[motorA] = motor[motorD] = 10;

	while (SensorValue[COLOR] != (int)(colorBlack))
	{}

	rotateRobot(-90, 10);

	// rotate the ultrasonic to face front and check for any objects nearby
	while (SensorValue[COLOR] != (int)(colorRed))
	{
		while (SensorValue[COLOR] != (int)(colorBlack))
		{
			motor[motorD] = 5;
			motor[motorA] = 10;
			wait1Msec(100);
		}
		while (SensorValue[COLOR] == (int)(colorBlack))
		{
			motor[motorD] = 10;
			motor[motorA] = 5;
			wait1Msec(100);
		}
	}
	motor[motorA] = motor[motorD] = 0;
	wait1Msec(100);
}

void adjustPark(float targetGyroPosition)
{
		// read in gyro value at beginning of parking action in int main
		// save to targetGyroPosition
		//(double check gyro degrees are int NOT float **)

		const float US_LIMIT = 200;
		const float BUMPER_DIST = 5;
		float frontDist = 0, backDist = 0;
		while (nMotorEncoder[motorB]%360 != 0)
		{
			motor[motorB] = 50;
		}
		motor[motorB] = 0;

		frontDist = SensorValue[US];
		wait1Msec(50);

		while (nMotorEncoder[motorB]%360 +180 != 0)
		{
			motor[motorB] = -50;
		}
		motor[motorB] = 0;

		backDist = SensorValue[US];
		wait1Msec(50);

		//if front dist > US limit, set _x_ cm from back bumper
		//if back dist > US limit, set _x_ cm from front bumper
		//else set bumper (frontDist+backDist)/2 cm from each bump

		int fwdBkwd = 1; //defaults to fwd motor power

		if (frontDist > US_LIMIT)//no car in front
		{
			if (backDist > BUMPER_DIST)
				{fwdBkwd = -1;}//move bkwd

			while(backDist != BUMPER_DIST)
					{motor[motorA] = motor[motorD] = 5*fwdBkwd;}
			motor[motorA] = motor[motorD]= 0;
		}
		else if (backDist > US_LIMIT)//no car behind
		{
			if (frontDist < BUMPER_DIST)
				{fwdBkwd = -1;}//move bkwd

			while(frontDist != BUMPER_DIST)
					{motor[motorA] = motor[motorD] = 5*fwdBkwd;}
			motor[motorA] = motor[motorD]= 0;
		}
		else // assume car in front AND behind
		{
			float bumperSpacing = (frontDist+backDist)/2;

			if (frontDist < bumperSpacing)
			{fwdBkwd = -1;}//move bkwd

			while (frontDist != bumperSpacing)
				{motor[motorA] = motor[motorD] = 5*fwdBkwd;}
			motor[motorA] = motor[motorD]= 0;
		}
		wait1Msec(50);

		int finalGyroPosition = getGyroDegrees(GYRO);
		bool aligned = false;
		if (fabs(finalGyroPosition - targetGyroPosition) < 5)
		{
			aligned = true;
		}

}

task main()
{
	SensorType[COLOR] = sensorEV3_Color;
	SensorMode[COLOR] = modeEV3Color_Color;
	SensorType[US] = sensorEV3_Ultrasonic;
	SensorType[GYRO] = sensorEV3_Gyro;

	const float SLOW_BACKUP = -10;
	const int backUpLimit = 15;
	const float halfPoint = 45;

	int count = 0;
	int targetGyroPosition = 0;

	// minimum space needed to park
  float clearance = 0;
  float multiplier = 0;
  float sideClear = 0;

	clearance = objectDistance(sideClear);

	if(isAble(clearance))
	{
		multiplier = powerMultiplier(sideClear, clearance);
		displayString(2, "Is Able");
		displayString(6, "%f", SensorValue[GYRO];
		displayString(7, "Gyro");

		resetGyro(GYRO);
		while (fabs(SensorValue[GYRO]) < halfPoint)
		{
			motor[motorA] = SLOW_BACKUP;
			motor[motorD] = SLOW_BACKUP * multiplier;
		}
		motor[motorA] = motor[motorD] = 0;
		wait1Msec(200);

		while (fabs(SensorValue[GYRO]) > 0)
		{
			motor[motorA] = SLOW_BACKUP * multiplier;
			motor[motorD] = SLOW_BACKUP;
		}
		motor[motorA] = motor[motorD] = 0;
		wait1Msec(200);

	}
	displayString(3, "%f", clearance);


}
