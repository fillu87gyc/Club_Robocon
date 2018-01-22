#ifndef MOTOR_INCLUDED
#define MOTOR_INCLUDED

#include "mbed.h"

#define NUMBER 4

class Motor {
public:
	Motor();
	void init();
	void drive(int ch);
	void off();
	void allDrive();
	void drive_LP(int ch);
	void allDrive_LP();
	static const int FORWARD = 1;
	static const int BACK = 2;
	int duty[NUMBER];
	int dir[NUMBER];
	int abs(int a) {
		if (a > 0)
			return a;
		else
			return -a;
	}
private:
	int dir_old[NUMBER];
	int duty_old[NUMBER];
	static const int UP_MOTOR_SPEED = 50;	//50
	static const int DOWN_MOTOR_SPEED = 20;	//20
	static const int DIFF = 200;			//200
	static const int MAGNIFICATION = 3;		//0への収束速度を決める(倍率)
	static const int MOTOR_STOP_DUTY = 0;
};

#endif
