#include "Motor.h"
PwmOut p[] = {  //  配列を用意します
		PwmOut(p26),  //  配列の1番目の要素をLED4で初期化したDigitalOutに
		PwmOut(p25),  //  ..
		PwmOut(p24),  //  ..
		PwmOut(p23)   //  ..
		};

DigitalOut d[] = { //  配列の1番目の要素をLED4で初期化したDigitalOutに
		DigitalOut(P2_6), // ..
		DigitalOut(P2_7),  //  ..
		DigitalOut(P2_8),  //  ..
		DigitalOut(P2_10)   //  ..
		};

Motor::Motor()    //コンストラクタ
{

}

void Motor::init() {
	for (int i = 0; i < 4; i++) {
		p[i].write(0);
		d[i].write(0);
		p[i].period_us(83);
		duty[i] = 0;
		dir[i] = 0;
		duty_old[i] = 0;
		dir_old[i] = 0;
	}
}

void Motor::drive(int ch) {
	//0 1
	//2 3
	if (ch % 2 == 0) {
		//右側車輪
		if (dir[ch] == BACK) {
			///Formt
			d[ch] = 0;
			p[ch].write(duty[ch] / 1000.0);
		} else if (dir[ch] == FORWARD) {
			//back
			d[ch] = 1;
			p[ch].write((1000 - duty[ch]) / 1000.0);
		} else {
			d[ch] = 1;
			p[ch].write(1.0f);
		}
	} else {
		//左側車輪
		if (dir[ch] == FORWARD) {
			///Formt
			d[ch] = 0;
			p[ch].write(duty[ch] / 1000.0);
		} else if (dir[ch] == BACK) {
			//back
			d[ch] = 1;
			p[ch].write((1000 - duty[ch]) / 1000.0);
		} else {
			d[ch] = 1;
			p[ch].write(1.0f);
		}
	}
}
void Motor::drive_LP(int ch) {
	if (duty[ch] == MOTOR_STOP_DUTY) {
		dir[ch] = dir_old[ch];
		duty[ch] = duty_old[ch] = (duty_old[ch]	- DOWN_MOTOR_SPEED * MAGNIFICATION);
		drive(ch);
		return;
	}
	if (dir[ch] == dir_old[ch]) {
		if (duty[ch] > duty_old[ch]) {
			//加速したい
			duty[ch] = duty_old[ch] = (duty_old[ch] + UP_MOTOR_SPEED);
			drive(ch);
			return;
		} else {
			//減速
			duty[ch] = duty_old[ch] = (duty_old[ch] - DOWN_MOTOR_SPEED);
			drive(ch);
			return;
		}
	} else {
		duty[ch] = duty_old[ch] = duty_old[ch] - DOWN_MOTOR_SPEED * MAGNIFICATION;
		if (duty[ch] < DIFF) {
			dir_old[ch] = dir[ch];
			duty_old[ch] = duty[ch];
			drive(ch);
			return;
		} else {
			dir[ch] = dir_old[ch];
			drive(ch);
			return;
		}
	}
}
void Motor::off() {
	for (int i = 0; i < 4; i++) {
		dir[i] = 0;
		duty[i] = 0;
		duty_old[i] = 0;
		dir_old[i] = 0;
		drive(i);
	}
}

void Motor::allDrive() {
	for (int i = 0; i < 4; ++i) {
		drive(i);
	}
}
void Motor::allDrive_LP() {
	for (int i = 0; i < 4; ++i) {
		drive_LP(i);
	}
}

