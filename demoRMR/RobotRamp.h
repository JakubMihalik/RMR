#ifndef ROBOTRAMP_H
#define ROBOTRAMP_H

#endif // ROBOTRAMP_H

#define RAMP_INCREMENT_VALUE 100.0
#define RAMP_DECREMENT_VALUE 200.0
//#define MAX_SPEED            3.1415926536 / 3.0
#define MAX_SPEED            400.0

class RobotRamp
{
private:
    double desiredSpeed;
    double currentSpeed;

public:
    RobotRamp();
    ~RobotRamp();
    double setDesiredSpeed(double speed);
    double getDesiredSpeed();
    void setCurrentSpeed(double speed);
    double getCurrentSpeed();
};
