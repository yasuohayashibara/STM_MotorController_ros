/* 
 * File:   B3MServoMotor.h
 * Author: mirai
 *
 * Created on 2017/07/03, 14:33
 */

#ifndef B3MSERVOMOTORS_H
#define B3MSERVOMOTORS_H

#include "ServoMotors.h"
#include "b3m_servo.h"
#include <iostream>

class B3MServoMotors : public ServoMotors
{
public:
    B3MServoMotors();
    ~B3MServoMotors();
    void setJointAnglesRad(std::vector<double> angles_rad);
    void setJointAngleRad(int id, double angle_rad);
    double getJointAngleRad(int id);
    void setServoOn(int id);
    void setServoOff(int id);
    void setServoAllOn();
    void setServoAllOff();
    void dataStoreAll();
private:
    B3MData _b3m;
    int convertRadtoDeg100(double rad);
    double convertDeg100toRad(int deg100);
};

#endif /* B3MSERVOMOTOR_H */

