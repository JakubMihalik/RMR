//#include <cmath>
//#include <vector>
//#include <utility>

//constexpr double PI = 3.14159265358979323846;
//constexpr double SAFE_DISTANCE = 0.2; // 20 cm

//enum RobotState {MOVE_TO_GOAL, FOLLOW_OBSTACLE};

//class Bug2 {
//public:
//    Bug2(double goalX, double goalY)
//    : goalX(goalX), goalY(goalY), robotState(MOVE_TO_GOAL) {}

//    void updateRobotState(double posX, double posY, double rotation);
//    void processLidarData(const std::vector<std::pair<double, double>>& lidarData);
//    void controlRobot();

//private:
//    double goalX;
//    double goalY;
//    double posX;
//    double posY;
//    double rotation;
//    RobotState robotState;
//    std::vector<std::pair<double, double>> lidarDataXY;

//    bool obstacleDetected();
//    void moveToGoal();
//    void followObstacle();
//    bool canMoveToGoal();

//    double distance(double x1, double y1, double x2, double y2) {
//        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
//    }
//};

//void Bug2::updateRobotState(double posX, double posY, double rotation) {
//    this->posX = posX;
//    this->posY = posY;
//    this->rotation = rotation;
//}

//void Bug2::processLidarData(const std::vector<std::pair<double, double>>& lidarData) {
//    // Assuming lidarData contains 277 points with distance in meters * 1000 and angle from 360 to 0.
//    lidarDataXY.clear();

//    for (const auto& point : lidarData) {
//        double angle_rad = point.second * PI / 180.0;
//        double x = point.first * cos(angle_rad) / 1000.0;
//        double y = point.first * sin(angle_rad) / 1000.0;

//        lidarDataXY.emplace_back(x, y);
//    }
//}

//bool Bug2::obstacleDetected() {
//    for (const auto& point : lidarDataXY) {
//        if (distance(posX, posY, point.first, point.second) < SAFE_DISTANCE) {
//            return true;
//        }
//    }

//    return false;
//}

//void Bug2::moveToGoal() {
//    // TODO: Implement the logic to move the robot towards the goal using your arcSpeed controller.
//}

//void Bug2::followObstacle() {
//    // TODO: Implement the logic to make the robot follow the obstacle's contour using your arcSpeed controller.
//}

//bool Bug2::canMoveToGoal() {
//    // TODO: Implement the logic to check if the robot can move directly towards the goal without hitting any obstacles.
//}

//void Bug2::controlRobot() {
//    if (robotState == MOVE_TO_GOAL) {
//        if (obstacleDetected()) {
//            robotState = FOLLOW_OBSTACLE;
//        } else {
//            moveToGoal();
//        }
//    } else if (robotState == FOLLOW_OBSTACLE) {
//        if (canMoveToGoal()) {
//            robotState = MOVE_TO_GOAL;
//        } else {
//            followObstacle();
//        }
//    }
//}
