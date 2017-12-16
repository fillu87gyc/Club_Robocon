/*
 packet protocol
 --------+---------------+----------
 while   |   0xEB        | header
 1byte	 |   joystick_LX | 0 ~ 100 (Median:50)
 2byte   |   joystick_LY | 0 ~ 100 (Median:50)
 3byte   | 	 joystick_RX | 0 ~ 100 (Median:50)
 4byte   |   joystick_RY | 0 ~ 100 (Median:50)
 5byte   |   button1     | bit
 6byte   |   button2     | bit
 7byte   |   Sum         | unchar
 8byte   |   0xAF        | ender
 --------+---------------+-----------

 button bit assign
 -+-----+-------+
 | bit0 | Select |
 | bit1 | L3     |
 | bit2 | R3     |
 | bit3 | Start  |
 | bit4 | Up     |
 | bit5 | Rigth  |
 | bit6 | Down   |
 | bit7 | Left   |
 -+-----+-------+--
 | bit0 | L2     |
 | bit1 | R2     |
 | bit2 | L1     |
 | bit3 | R1     |
 | bit4 | △		|
 | bit5 | Circle |
 | bit6 | Cross  |
 | bit7 | Square |
 -+------+-------+

 */

#include "mbed/mbed.h"
#include "myLIB/MySerial.h"
#include "myLIB/button.h"
#include "Motor/Motor.h"

typedef unsigned char unchar;

//Define the number
const int Ly = 0, Ry = 1, Lx = 2, Rx = 3, sw1 = 4, sw2 = 5, Sum = 6, end = 7;
//make instance
//Serial
Serial slave(p13, p14);
MySerial FEP01TJ(&slave);
//debug
DigitalOut led1(p6), led2(p7), led3(p8);
//valve attach
Motor motor;

Button dualshock;

//global
unchar re_data[8] = { 0 };
double joystick[4] = { 0.0, 0.0, 0.0, 0.0 };

inline unchar CheckSum(unchar *data) {
	return data[Lx] + data[Ly] + data[Rx] + data[Ry] + data[sw1] + data[sw2];
}

bool uartRead() {
	int count = 0;
	while (1) {
		if (FEP01TJ.input() == 0xEB)
			break;
		else {
			count++;
			if (count > 15)
				return false;
		}
	}
	re_data[Lx] = FEP01TJ.input();   //Lx 飛ばす
	re_data[Ly] = FEP01TJ.input();   //Ly
	re_data[Rx] = FEP01TJ.input();   //Rx 飛ばす
	re_data[Ry] = FEP01TJ.input();   //Ry
	re_data[sw1] = FEP01TJ.input();   //sw1
	re_data[sw2] = FEP01TJ.input();   //sw2
	re_data[Sum] = FEP01TJ.input();   //CheckSum
	re_data[end] = FEP01TJ.input();   //end
#ifdef print_debugger
#ifdef Serial_debugger
	pc.printf("%d %d %d %d 0x%X 0x%X 0x%X 0x%X sum = 0x%x\n\r", re_data[Lx], re_data[Ly], re_data[Rx], re_data[Ry], re_data[sw1], re_data[sw2], re_data[Sum], re_data[end], CheckSum(re_data));
#endif
#endif
	if ((re_data[Sum] == CheckSum(re_data)) && (re_data[end] == 0xAF))
		return true;
	else
		return false;

}

void input_val() {
	joystick[Ly] = 2.0 * ((re_data[Ly] / 100.0) - 0.5);
	joystick[Ry] = 2.0 * ((re_data[Ry] / 100.0) - 0.5);
	joystick[Lx] = 2.0 * ((re_data[Lx] / 100.0) - 0.5);
	joystick[Rx] = 2.0 * ((re_data[Rx] / 100.0) - 0.5);
	dualshock.set_Data((re_data[sw2] << 8) + re_data[sw1]);
}

void arm_ctrl() {
	double velocity = 0.4;
	motor.dir3 = 0;
	if (dualshock.push(R1))motor.dir3 = 1;
	else if (dualshock.push(L1))motor.dir3 = 2;

	if (dualshock.push(Square))velocity = 1.0;

	motor.duty3 = (int)(999.9 * velocity);
	motor.drive(3);
}
void machine_move()
{
//	motor.dir1 = (joystick[Ly] > 0.0) ? 1 : 2;
//	motor.dir2 = (joystick[Ry] > 0.0) ? 1 : 2;
	if(joystick[Ly]>0.0)motor.dir1 = 1;
	else motor.dir1 = 2;
	if(joystick[Ry]>0.0)motor.dir2 = 1;
	else motor.dir2 = 2;

	motor.duty1 = motor.abs( joystick[Ly] * 700);
	motor.duty2 = motor.abs(joystick[Ry] * 700);
	if (dualshock.push(Cross))
	{
		motor.duty1 *= 0.5;
		motor.duty2 *= 0.5;
	}
	motor.drive_LP(1);
	motor.drive_LP(2);
}

void setup() {
	slave.baud(38400);
	motor.init();
	led1.write(1);
}
void loop() {
	int failed_counter = 0;
	if (uartRead()) {
		led2.write(1);
		input_val();
		machine_move();
		arm_ctrl();
		wait_ms(20);
	}
	else {
		led2.write(0);
		failed_counter++;
		if (failed_counter > 2) {
			//通信が復帰することなく3回連続失敗
			motor.off();
		}
	}
}
int main() {
	setup();
	for (;;)loop();
}
