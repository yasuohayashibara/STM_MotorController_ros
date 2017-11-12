/* 
 * File:   B3MServoMotor.cpp
 * Author: mirai
 * 
 * Created on 2017/07/03, 14:33
 */

#include <string>
#include <boost/math/constants/constants.hpp>
#include "B3MServoMotors.h"

#define COMM_PORT "/dev/ttyUSBKondo" 

B3MServoMotors::B3MServoMotors()
{
    if (b3m_init(&_b3m, COMM_PORT) < 0) {
        fprintf(stderr, "ERROR:Com port open error\n" );
    }
}

B3MServoMotors::~B3MServoMotors()
{
	b3m_close(&_b3m);
}

/*!
 * @brief set distination angle of joints
 * @param [in] angles_rad angle of joint id 0.1,2,... 
 */
void B3MServoMotors::setJointAnglesRad(std::vector<double> angles_rad)
{
	assert(_num_servos);
	const int LEN = 3;		// data length for a joint (angle: 2byte, id: 1byte)
	char	sendbuf[128] = {};

	// パケット作成
	sendbuf[0]  = (unsigned char)(6+LEN*_num_servos);
	sendbuf[1]  = (unsigned char)B3M_CMD_WRITE;
	sendbuf[2]  = (unsigned char)0x00;

	int index = 0;
	for(unsigned int i = 0; i < _servo_ids.size(); i ++){
		short id = _servo_ids[i];
		unsigned short data = convertRadtoDeg100(angles_rad[id - 1]);
		sendbuf[3+index*LEN] = (unsigned char)id;
		sendbuf[4+index*LEN] = (unsigned char)(data&0x00FF);
		sendbuf[5+index*LEN] = (unsigned char)((data&0xFF00)>>8);
		index ++;
	}
	sendbuf[_num_servos*LEN+3]  = B3M_SERVO_DESIRED_POSITION;
	sendbuf[_num_servos*LEN+4]  = (unsigned char)_num_servos;
	
	// チェックサムの計算
	unsigned char sum = sendbuf[0];
	for(int i = 1; i <= _num_servos*LEN+4; i++ ){
		sum += sendbuf[i];
	}
	sendbuf[_num_servos*LEN+5] = sum;

	for(int i = 0; i < _num_servos*LEN+6; i ++) _b3m.swap[i] = sendbuf[i];
	b3m_write(&_b3m, _num_servos*LEN+6);
}

void B3MServoMotors::setJointAngleRad(int id, double angle_rad)
{
	int deg100 = convertRadtoDeg100(angle_rad);
	b3m_set_angle(&_b3m, id, deg100);
}

double B3MServoMotors::getJointAngleRad(int id)
{
	int deg100 = 0;
	b3m_get_angle(&_b3m, id, &deg100);
	double angle_rad = convertDeg100toRad(deg100);
	return angle_rad;
}

void B3MServoMotors::setServoOn(int id)
{
	b3m_servo_mode(&_b3m, id, B3M_OPTIONS_RUN_NORMAL);
}

void B3MServoMotors::setServoOff(int id)
{
	b3m_servo_mode(&_b3m, id, B3M_OPTIONS_RUN_FREE);
}
/*!
 * @brief servo on id:1-30
 */
void B3MServoMotors::setServoAllOn()
{
	ServoMotors::setServoAllOn();
	const int LEN = 3;
	const int NUM_SERVO = 30;
	char sendbuf[128] ={};
	int servo_no;

	// パケット作成
	sendbuf[0]  = (unsigned char)(6+NUM_SERVO*LEN);
	sendbuf[1]  = (unsigned char)B3M_CMD_WRITE;
	sendbuf[2]  = (unsigned char)0x00;
	for( servo_no = 1; servo_no <= NUM_SERVO; servo_no++){
		sendbuf[3+(servo_no-1)*LEN] = (unsigned char)servo_no;
		sendbuf[4+(servo_no-1)*LEN] = (unsigned char)B3M_OPTIONS_RUN_NORMAL;
		sendbuf[5+(servo_no-1)*LEN] = (unsigned char)0x00;
	}
	sendbuf[NUM_SERVO*LEN+3]  = (unsigned char)B3M_SERVO_SERVO_MODE;
	sendbuf[NUM_SERVO*LEN+4]  = (unsigned char)NUM_SERVO;

	// チェックサムの計算
	unsigned char sum = sendbuf[0];
	for(int i = 1; i <= NUM_SERVO*LEN+4; i++ ){
		sum += sendbuf[i];
	}
	sendbuf[NUM_SERVO*LEN+5] = sum;
	
	for(int i = 0; i < NUM_SERVO*LEN+6; i ++) _b3m.swap[i] = sendbuf[i];
	b3m_write(&_b3m, NUM_SERVO*LEN+6);
}

/*!
 * @brief servo off id:1-30
 */
void B3MServoMotors::setServoAllOff()
{
	ServoMotors::setServoAllOff();
	const int LEN = 3;
	const int NUM_SERVO = 30;
	char sendbuf[128] = {};
	int servo_no;

	// パケット作成
	sendbuf[0]  = (unsigned char)(6+NUM_SERVO*LEN);
	sendbuf[1]  = (unsigned char)B3M_CMD_WRITE;
	sendbuf[2]  = (unsigned char)0x00;
	for( servo_no = 1; servo_no <= NUM_SERVO; servo_no++){
		sendbuf[3+(servo_no-1)*LEN] = (unsigned char)servo_no;
		sendbuf[4+(servo_no-1)*LEN] = (unsigned char)B3M_OPTIONS_RUN_FREE;
		sendbuf[5+(servo_no-1)*LEN] = (unsigned char)0x00;
	}
	sendbuf[NUM_SERVO*LEN+3]  = (unsigned char)B3M_SERVO_SERVO_MODE;
	sendbuf[NUM_SERVO*LEN+4]  = (unsigned char)NUM_SERVO;

	// チェックサムの計算
	unsigned char sum = sendbuf[0];
	for(int i = 1; i <= NUM_SERVO*LEN+4; i++ ){
		sum += sendbuf[i];
	}
	sendbuf[NUM_SERVO*LEN+5] = sum;
	
	for(int i = 0; i < NUM_SERVO*LEN+6; i ++) _b3m.swap[i] = sendbuf[i];
	b3m_write(&_b3m, NUM_SERVO*3+6);	
}

int B3MServoMotors::convertRadtoDeg100(double rad)
{
	double PI = boost::math::constants::pi<double>();
	return (int)(rad * 180.0 / PI * 100.0);
}

double B3MServoMotors::convertDeg100toRad(int deg100)
{
	double PI = boost::math::constants::pi<double>();
	return (double)(deg100 * PI / 180.0 / 100.0);
}

void B3MServoMotors::dataStoreAll()
{
	char sendbuf[128] = {};

	// パケット作成
	sendbuf[0]  = (unsigned char)0x05;
	sendbuf[1]  = (unsigned char)B3M_CMD_DATA_STOCK;
	sendbuf[2]  = (unsigned char)0x00;
	sendbuf[3]  = (unsigned char)0xFF;
	
	// チェックサムの計算
	unsigned char sum = sendbuf[0];
	for(int i = 1; i < 4; i++ ){
		sum += sendbuf[i];
	}
	sendbuf[4] = sum;								// チェックサム
	// 送信
	for(int i = 0; i < 5; i ++) _b3m.swap[i] = sendbuf[i];
	b3m_write(&_b3m, 5);
}
