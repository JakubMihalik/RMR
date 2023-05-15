#include "Controller.h"
#include "BugAlgorithm.h"
#include <cmath>
Controller::Controller(Robot* robot, OdometryData* odData, double desiredX, double desiredY)
{
    this->robot = robot;
    this->odData = odData;
    this->desiredX = desiredX;
    this->desiredY = desiredY;
    this->checkpoints.push_back({desiredX, desiredY});
    this->fStopLidar = false;
    this->fRotating = false;
    this->controllerOutput = {0.0, 0.0, 0.0};
}

Controller::~Controller()
{
    delete this->robot;
    delete this->odData;
}

ControllerOutput Controller::regulate()
{
    ErrorValue ev = Controller::calculateErrors();

    double distance = sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    double reqFwdSpeed = 1000 * distance;
    double reqRotSpeed = 5 * ev.theta;

    double rotConst = PI / 32;
    double fwdConst = 5;


    // Je v v pozadovanom priestore
    if (abs(ev.x) < 0.03 && abs(ev.y) < 0.03)
    {
        robot->setTranslationSpeed(0);

        if (this->checkpoints.size() > 1)
        {
            this->checkpoints.pop_back();
            std::cout << "Checkpoint reached. Poping out..." << std::endl;
        }
        else
        {
            this->b_finishReached = true;
            std::cout << "Finished" << std::endl;
        }

        return controllerOutput;
    }

    if (reqFwdSpeed - controllerOutput.forwardSpeed > fwdConst)
    {
        controllerOutput.forwardSpeed += fwdConst;
    }
    else
    {
        controllerOutput.forwardSpeed = reqFwdSpeed;
    }
    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, 750);


    if (controllerOutput.rotationSpeed - reqRotSpeed > rotConst)
    {
        controllerOutput.rotationSpeed -= rotConst;
    }
    else if (controllerOutput.rotationSpeed - reqRotSpeed < -rotConst)
    {
        controllerOutput.rotationSpeed += rotConst;
    }
    else
    {
        controllerOutput.rotationSpeed = reqRotSpeed;
    }
    controllerOutput.rotationSpeed = max(min(controllerOutput.rotationSpeed, PI / 2), -PI / 2);

    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.1;
    double radius = controllerOutput.forwardSpeed / denom;

    // Set stop lidar flag
    this->fStopLidar = abs(controllerOutput.rotationSpeed) >= PI / 12; // Toto osetrit nie na radar ale na to co robot robi

    // Set rotation flag
    this->fRotating = abs(controllerOutput.rotationSpeed) >= PI / 12;

//    robot->setArcSpeed(controllerOutput.forwardSpeed, radius);

    return controllerOutput;
}

ErrorValue Controller::calculateErrors()
{
    double eX = abs(this->checkpoints.back().x - this->odData->posX);
    double eY = abs(this->checkpoints.back().y - this->odData->posY);

    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(this->checkpoints.back().y - this->odData->posY,
                          this->checkpoints.back().x - this->odData->posX) - this->odData->rotation * PI / 180;
    if (eTheta > PI) {
        eTheta -= 2*PI;
    } else if (eTheta < -PI) {
        eTheta += 2*PI;
    }
    ErrorValue e = {eX, eY, eTheta};
//    std::cout << "Going for point: " << checkpoints.back().x << ", " << checkpoints.back().y << std::endl;
    return e;
}

void Controller::setCheckpoints(std::vector<Point>& checkpoints)
{
    this->checkpoints = std::move(checkpoints);
}

void Controller::regulateDynamic(LaserMeasurement lidar)
{
    /** Zaporna hodnota RADIUS toci smerom do lava
     *  Zatacanie do lava (ku stene pre pravy senzor) (-) zaporna hodnota
     *  Zatacanie do prava (od steny pre pravy senzor)(+) kladna hodnota
    **/

    // Find closest point
    LaserData point = {1, 0, DBL_MAX};
    for (int i{0}; i < lidar.numberOfScans; i++)
    {
        if (   (lidar.Data[i].scanDistance > 130 && lidar.Data[i].scanDistance < 3000)
            && (lidar.Data[i].scanDistance < 640 || lidar.Data[i].scanDistance > 700))
        {
            if (lidar.Data[i].scanDistance < point.scanDistance)
            {
                point = lidar.Data[i];

                // Upravime uhol na rozsah -180 az 180
//                if (point.scanAngle > 180)
//                {
//                    point.scanAngle = 360 - point.scanAngle;
//                }
            }
        }
    }

    if (point.scanDistance != DBL_MAX)
    {
        float wantedRobotDistance = 400.0;
        float angleShift = (-90);
        angleShift += (point.scanDistance - wantedRobotDistance)/5;
        float newAngle = point.scanAngle + angleShift;
        newAngle=newAngle>180 ? newAngle-360:newAngle<-180?newAngle+360:newAngle;
    //    newAngle=newAngle>PI ? newAngle-PI*2:newAngle<-PI?newAngle+PI*2:newAngle;




        float x = odData->posX*1000 + point.scanDistance*cos(deg2rad(odData->rotation+(360-point.scanAngle)));
        float y = odData->posY*1000 + point.scanDistance*sin(deg2rad(odData->rotation+(360-point.scanAngle)));
    //    x_wall_follow = (x*cos(PI/2)+y*sin(PI/2))/1000;
    //    y_wall_follow = (-x*sin(PI/2)+y*cos(PI/2))/1000;
        //    dx = x_wall_follow - state.x;
        //     dy = y_wall_follow - state.y;
        float x_wall_follow = (odData->posX*1000 + point.scanDistance*cos(deg2rad(odData->rotation+(360-newAngle))))/1000;
        float y_wall_follow = (odData->posY*1000 + point.scanDistance*sin(deg2rad(odData->rotation+(360-newAngle))))/1000;
        m_angle = radiansToDegrees(atan2(odData->posY - y_wall_follow, odData->posX - x_wall_follow));
        m_distance = std::sqrt(std::pow(odData->posX - x_wall_follow, 2) + std::pow(odData->posY - y_wall_follow, 2));
        regulateWallFollow(x_wall_follow, y_wall_follow);
    }

//    if (point.scanDistance != DBL_MAX)
//    {
//        /*const static*/ double desiredDistance = point.scanDistance + ROBOT_DIAMETER_MM;

//        std::cout << "Found point: " << point.scanAngle << std::endl;
//        std::cout << "With distance: " << point.scanDistance << std::endl;

//        // Vypocitame poziciu najblizsieho bodu [mm]
//        double wallX = odData->posX + (point.scanDistance - ROBOT_DIAMETER_MM) * std::cos(degreesToRadians(odData->rotation + point.scanAngle));
//        double wallY = odData->posY + (point.scanDistance - ROBOT_DIAMETER_MM) * std::sin(degreesToRadians(odData->rotation + point.scanAngle));

//        // Vypocitame vektor k najblizsiemu bodu [mm]
//        Point vector = {wallX - odData->posX,
//                        wallY - odData->posY};

//        // Otocime vektor o 90 stupnov [mm]
//        Point rotatedVector = {vector.x * std::cos(PI/2) - vector.y * std::sin(PI/2),
//                               vector.x * std::sin(PI/2) + vector.y * std::cos(PI/2)};

//        // Nascaleujeme vektor o pozadovanu vzdialenost
//        Point scaledVector = {rotatedVector.x * desiredDistance / point.scanDistance,
//                              rotatedVector.y * desiredDistance / point.scanDistance};

//        // Vypocitame bod na sledovanie [mm]
//        double pointX = odData->posX + scaledVector.x;
//        double pointY = odData->posY + scaledVector.y;

//        // Vypocitame uhol a vzdialenost bodu kvoli vykreslovaniu
//        m_angle = radiansToDegrees(atan2(odData->posY - pointY, odData->posX - pointX));
//        m_distance = std::sqrt(std::pow(odData->posX - pointX, 2) + std::pow(odData->posY - pointY, 2));
////        m_angle = radiansToDegrees(point.scanAngle);
////        m_distance = 500;


//        // A prevedieme na metre
//        pointX /= 1000.0;
//        pointY /= 1000.0;

//        // Regulate
//        regulateWallFollow(pointX, pointY);
//    }
}


std::pair<double, double> Controller::control_step(LaserMeasurement lidar) {
    std::pair<double, double> output;

    if (bug_state == GO_TO_TARGET) {
       // Check for obstacle
       bool obstacle_detected = false;

       // scanDistance is in [mm]
       for (int i{0}; i < lidar.numberOfScans; i++)
       {
           if (   (lidar.Data[i].scanDistance > 130 && lidar.Data[i].scanDistance < 3000)
               && (lidar.Data[i].scanDistance < 640 || lidar.Data[i].scanDistance > 700))
           {
               if (lidar.Data[i].scanDistance < ROBOT_DIAMETER_MM)
               {
                   obstacle_detected = true;
                   break;
               }
           }
       }

       // odData are in [m]
       if (obstacle_detected) {
           bug_state = OBSTACLE_AVOIDANCE;
           hit_point_x = odData->posX;
           hit_point_y = odData->posY;
       }
   }
   if (bug_state == OBSTACLE_AVOIDANCE) {
       std::cout<<"AVOIDING OBSTACLE\n";

        // Find closest point
        LaserData point = {1, 0, DBL_MAX};
        for (int i{0}; i < lidar.numberOfScans; i++)
        {
           if (   (lidar.Data[i].scanDistance > 130 && lidar.Data[i].scanDistance < 3000)
               && (lidar.Data[i].scanDistance < 640 || lidar.Data[i].scanDistance > 700))
           {
               if (lidar.Data[i].scanDistance < point.scanDistance)
               {
                   point = lidar.Data[i];
                   point.scanAngle = (360 - point.scanAngle);
               }
           }
        }

        double desired_angle = deg2rad(point.scanAngle);
        double target_angle = std::atan2(this->checkpoints.back().y - odData->posY, this->checkpoints.back().x - odData->posX);

        if (turn_direction == 0)
        {
            double obstacle_x = point.scanDistance * cos(deg2rad(point.scanAngle));
            double obstacle_y = point.scanDistance * sin(deg2rad(point.scanAngle));

//            std::cout << "Closest obstacle at: Distance: " << point.scanDistance << " Angle: " << point.scanAngle << "\n";

            // scanAngle is in [360 - 0] exact opposite to robots rotating angle [0 - 360]
            double obstacle_angle = std::atan2(obstacle_y - odData->posY, obstacle_x - odData->posX);

            // Compute the difference and normalize to the range [-PI, PI]
            double angle_diff2 = obstacle_angle - target_angle;
            angle_diff2 = atan2(sin(angle_diff2), cos(angle_diff2));

            // Determine the direction to turn
            if (angle_diff2 < 0) {
                // Obstacle is to the left of the target, turn right
                turn_direction = +1;
            } else {
                // Obstacle is to the right of the target, turn left
                turn_direction = -1;
            }
        }

        desired_angle += turn_direction * PI / 2;

        if (desired_angle > PI) {
            desired_angle -= 2 * PI;
        }


        double angular_speed = 5 * (desired_angle);
        double linear_speed = max(0, min(1.0, point.scanDistance / 1000.0)) * 750;

        std::cout << "Desired angle: " << desired_angle << "\n";
        std::cout << "Linear speed: " << linear_speed << " Angular speed: " << angular_speed << "\n";

        if (pathToGoalIsFree(lidar)) {
            bug_state = GO_TO_TARGET;
        }

        // Calculate the angle between the vector from the robot to the target and the vector from the robot to the hit point
//        double hit_point_angle = std::atan2(hit_point_y - odData->posY, hit_point_x - odData->posX);

//        double angle_diff = target_angle - hit_point_angle;
//        angle_diff = std::fmod(angle_diff, 2 * PI);


//        double distance_to_target = std::sqrt(std::pow(this->checkpoints.back().x - odData->posX, 2) + std::pow(this->checkpoints.back().y - odData->posY, 2));

//        // Check if the path to the target is clear (Line of Sight)
//        double required_heading = std::atan2(this->checkpoints.back().y - odData->posY, this->checkpoints.back().x - odData->posX);
//        required_heading = required_heading * 180 / PI;
//        if (required_heading < 0) required_heading += 360;



        // Check if we can leave the obstacle
//        if (angle_diff > PI && pathToGoalIsFree(lidar)) {
//            // The path to the goal is clear and the robot has moved around the obstacle,
//            // so it can leave the obstacle and start moving towards the goal again.
//            bug_state = GO_TO_TARGET;
//        }

        output = {linear_speed, angular_speed};
    }

   if (bug_state == GO_TO_TARGET) {
       std::cout<<"GOING TO TARGET\n";
       // Calculate the remaining distance to the target
       double distance_to_target = std::sqrt(std::pow(this->checkpoints.back().x - odData->posX, 2) + std::pow(this->checkpoints.back().y - odData->posY, 2));

       // Check if the robot has reached the target
       if (distance_to_target < 0.05) {  // 50 mm tolerance
           output = {0, 0};
       }

       // Calculate the required heading to the target
       double required_heading = std::atan2(this->checkpoints.back().y - odData->posY, this->checkpoints.back().x - odData->posX);

       // Calculate the heading error
       double heading_error = required_heading - deg2rad(odData->rotation);
       heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));  // Normalize the angle to be between -pi and pi

       regulation(distance_to_target, heading_error);
       // Calculate the control commands
       double linear_speed = 100 * distance_to_target;
       double angular_speed = PI * heading_error;

       output = {linear_speed, angular_speed};
   }

   return output;
}

bool Controller::pathToGoalIsFree(LaserMeasurement lidar)
{
    // Calculate the required heading to the target
    double heading_to_target = std::atan2(this->checkpoints.back().y - odData->posY, this->checkpoints.back().x - odData->posX);
    double distance_to_target = std::sqrt(std::pow(this->checkpoints.back().x - odData->posX, 2) + std::pow(this->checkpoints.back().y - odData->posY, 2));

    // Convert to degrees
    heading_to_target = heading_to_target * 180 / PI;

    if (heading_to_target < 0) heading_to_target += 360;

    double heading_range = 3.0;
    for (int i = 0; i < lidar.numberOfScans; i++) {
        if (   (lidar.Data[i].scanDistance > 130 && lidar.Data[i].scanDistance < 3000)
            && (lidar.Data[i].scanDistance < 640 || lidar.Data[i].scanDistance > 700))
        {
            // Pozicia scanu
            double scanAngle = 360 - lidar.Data[i].scanAngle;
            double laserX = odData->posX + (lidar.Data[i].scanDistance / 1000) * cos(scanAngle * PI / 180);
            double laserY = odData->posY + (lidar.Data[i].scanDistance / 1000) * sin(scanAngle * PI / 180);

            if (this->checkpoints.back().x + 0.1 < laserX &&
                this->checkpoints.back().x - 0.1 > laserX &&
                this->checkpoints.back().y + 0.1 < laserY &&
                this->checkpoints.back().y - 0.1 > laserY) {
                return true;
            }

        }
    }
    return false;
}

void Controller::obstacleAvoidance(LaserMeasurement laserData)
{
    // detect edges
    std::vector<int> edge_indices;
    double threshold = 0.1 * 1000;

    for (int i = 1; i < laserData.numberOfScans; i++) {
        double diff = abs(laserData.Data[i].scanDistance - laserData.Data[i-1].scanDistance);
        if (diff > threshold) {
            edge_indices.push_back(i);
        }
    }
}

void Controller::controll(std::pair<double, double> control_commands) {

//    if (abs(control_commands.second) > deg2rad(10))
//    {
//        robot->setRotationSpeed(control_commands.second);
//        return;
//    }
//    robot->setTranslationSpeed(control_commands.first);

    robot->setArcSpeed(control_commands.first, control_commands.first/control_commands.second);
}

void Controller::regulation(double distance, double theta) {
    double reqFwdSpeed = 500 * distance;
    double reqRotSpeed = 1 * theta;

    double rotConst = 0.1;
    double fwdConst = 3;


    // Je v v pozadovanom priestore
//    if (abs(ev.x) < 0.03 && abs(ev.y) < 0.03)
//    {
//        robot->setTranslationSpeed(0);
//        this->controllerOutput.forwardSpeed = 0;
//        this->controllerOutput.rotationSpeed = 0;
//        return;
//    }


    if (reqFwdSpeed - controllerOutput.forwardSpeed > fwdConst)
    {
        controllerOutput.forwardSpeed += fwdConst;
    }
    else
    {
        controllerOutput.forwardSpeed = reqFwdSpeed;
    }
    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, 750);


    if (controllerOutput.rotationSpeed - reqRotSpeed > rotConst)
    {
        controllerOutput.rotationSpeed -= rotConst;
    }
    else if (controllerOutput.rotationSpeed - reqRotSpeed < -rotConst)
    {
        controllerOutput.rotationSpeed += rotConst;
    }
    else
    {
        controllerOutput.rotationSpeed = reqRotSpeed;
    }
//    controllerOutput.rotationSpeed = max(min(controllerOutput.rotationSpeed, PI / 2), -PI / 2);

    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.1;
    double radius = controllerOutput.forwardSpeed / denom;

    // Set stop lidar flag
    if(abs(controllerOutput.rotationSpeed) >= PI / 12)
    {
        this->fRotating = true;
        this->fStopLidar = true;
    }
    else
    {
        this->fRotating = false;
        this->fStopLidar = false;
    }
//    if (reqRotSpeed / 5 > PI / 2)
//    {
//        robot->setRotationSpeed(PI / 6);
//    }
//    else


////    {

    if (abs(theta) > 0.05) {
        controllerOutput.forwardSpeed = 0;
        robot->setRotationSpeed(controllerOutput.rotationSpeed);
        return;
    }

    if (controllerOutput.rotationSpeed == 0) {
        robot->setTranslationSpeed(controllerOutput.forwardSpeed);
    } else {
        robot->setArcSpeed(controllerOutput.forwardSpeed, controllerOutput.forwardSpeed/controllerOutput.rotationSpeed);
    }
}


void Controller::regulateWallFollow(double reqX, double reqY)
{
    // Vypocitame chyby
    ErrorValue ev;
    double eX = reqX - this->odData->posX;
    double eY = reqY - this->odData->posY;

    // Calculate the difference between the current heading and the desired heading
    double eTheta = atan2(eY, eX) - this->odData->rotation * PI / 180;
    eTheta = fmod(eTheta+PI, 2*PI)-PI;
//    if (eTheta > PI) {
//        eTheta -= 2*PI;
//    } else if (eTheta < -PI) {
//        eTheta += 2*PI;
//    }
    ev = {eX, eY, eTheta};

    // Parametre regulacie
    double distance = sqrt(pow(ev.x, 2) + pow(ev.y, 2));
    double reqFwdSpeed = 500 * distance;
    double reqRotSpeed = 1 * ev.theta;

    double rotConst = 0.1;
    double fwdConst = 3;


    // Je v v pozadovanom priestore
//    if (abs(ev.x) < 0.03 && abs(ev.y) < 0.03)
//    {
//        robot->setTranslationSpeed(0);
//        this->controllerOutput.forwardSpeed = 0;
//        this->controllerOutput.rotationSpeed = 0;
//        return;
//    }


    if (reqFwdSpeed - controllerOutput.forwardSpeed > fwdConst)
    {
        controllerOutput.forwardSpeed += fwdConst;
    }
    else
    {
        controllerOutput.forwardSpeed = reqFwdSpeed;
    }
    controllerOutput.forwardSpeed = min(controllerOutput.forwardSpeed, 750);


    if (controllerOutput.rotationSpeed - reqRotSpeed > rotConst)
    {
        controllerOutput.rotationSpeed -= rotConst;
    }
    else if (controllerOutput.rotationSpeed - reqRotSpeed < -rotConst)
    {
        controllerOutput.rotationSpeed += rotConst;
    }
    else
    {
        controllerOutput.rotationSpeed = reqRotSpeed;
    }
//    controllerOutput.rotationSpeed = max(min(controllerOutput.rotationSpeed, PI / 2), -PI / 2);

    double denom = controllerOutput.rotationSpeed != 0 ? controllerOutput.rotationSpeed : 0.1;
    double radius = controllerOutput.forwardSpeed / denom;

    // Set stop lidar flag
    if(abs(controllerOutput.rotationSpeed) >= PI / 12)
    {
        this->fRotating = true;
        this->fStopLidar = true;
    }
    else
    {
        this->fRotating = false;
        this->fStopLidar = false;
    }
//    if (reqRotSpeed / 5 > PI / 2)
//    {
//        robot->setRotationSpeed(PI / 6);
//    }
//    else


//    {

    if (abs(ev.theta) > 0.05) {
        controllerOutput.forwardSpeed = 0;
        robot->setRotationSpeed(controllerOutput.rotationSpeed);
        return;
    }

    if (controllerOutput.rotationSpeed == 0) {
        robot->setTranslationSpeed(controllerOutput.forwardSpeed);
    } else {
        robot->setArcSpeed(controllerOutput.forwardSpeed, controllerOutput.forwardSpeed/controllerOutput.rotationSpeed);
    }
//    }

    // Vypis
    std::cout << "eTheta: " << ev.theta << std::endl;
    std::cout << "Request: [" << reqX << ", " << reqY << "]" << std::endl;
    std::cout << "Forward speed: " << controllerOutput.forwardSpeed << std::endl;
    std::cout << "Radius: " << radius << std::endl;
    std::cout << "Request rotation: " << reqRotSpeed / 5 << std::endl;
    // Koniec
    std::cout << std::endl;
}

void Controller::turnLeft(int speed, int radius)
{
    robot->setArcSpeed(speed, radius);
}

void Controller::turnRight(int speed, int radius)
{
    robot->setArcSpeed(speed, -radius);
}

void Controller::moveForward(int speed)
{
    robot->setArcSpeed(speed, 0);
}
