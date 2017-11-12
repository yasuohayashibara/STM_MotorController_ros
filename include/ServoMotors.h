/* 
 * File:   ServoMotors.h
 * Author: mirai
 *
 * Created on 2017/07/03, 14:17
 */

#ifndef SERVOMOTORS_H
#define SERVOMOTORS_H

#include <vector>

class ServoMotors
{
public:
    ServoMotors() : _num_servos(0), _is_all_servo_on(false) {}
    virtual ~ServoMotors();
    virtual void setJointAnglesRad(std::vector<double> angles_rad) {}
    virtual void setJointAngleRad(int id, double angle_rad) {}    
    virtual double getJointAngleRad(int id) { return 0.0; }
    virtual void setServoOn(int id) {}
    virtual void setServoOff(int id) {}
    virtual void setServoAllOn() { _is_all_servo_on = true; }
    virtual void setServoAllOff() { _is_all_servo_on = false; }
    bool getServoOn() { return _is_all_servo_on; }
    void setServoIDs(std::vector<int> servo_ids) {
        _servo_ids = servo_ids;
        _num_servos = servo_ids.size();
    }
protected:
    std::vector<int> _servo_ids;
    int _num_servos;
    bool _is_all_servo_on;
};

#endif /* SERVOMOTOR_H */

