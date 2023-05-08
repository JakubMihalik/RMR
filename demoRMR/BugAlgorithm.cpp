#include "BugAlgorithm.h"

BugAlgorithm::BugAlgorithm(Controller* controller, OdometryData* odData, Robot* robot)
{
    this->robotState = MOVE_TO_GOAL;
    this->controller = controller;
    this->odData = odData;
    this->robot = robot;
    this->start = {odData->posX, odData->posY};
    this->goal = {controller->checkpoints.front().x, controller->checkpoints.front().y};
    this->dbScan = new DBScan(0.1, 5);
}

// robot sa toci jak jebnuty, treba nieco upravit pravdepodobne je stale najdeny ten isty point co je tiez pekna picovina

bool BugAlgorithm::obstacleDetection()
{
    double closestDistance = (std::numeric_limits<double>::max)();

    for (int i{0}; i< this->_laserData.numberOfScans; i++)
    {
        double robotAngleRad = DEG2RAD(odData->rotation);
        double lidarAngleRad = DEG2RAD((360.0 - _laserData.Data[i].scanAngle)) - robotAngleRad;
        double lidarDistance = _laserData.Data[i].scanDistance/1000;
        if (abs(lidarAngleRad) < SAFE_ANGLE) {
            if (lidarDistance < this->SAFE_DISTANCE)
            {
                if (lidarDistance < closestDistance)
                {
                    closestDistance = lidarDistance;
                    this->startingFollowDistanceToGoal = distance(controller->checkpoints.front().x, controller->checkpoints.front().y, odData->posX, odData->posY);
                    this->startingFollowPosition = {odData->posX, odData->posY};

                    std::cout<<"Obstacle nearby! Distance: "<< lidarDistance << std::endl;
                    std::cout<<"Obstacle nearby! Angle: "<< lidarAngleRad << std::endl;
                    return true;
                }
            }
        }
    }
    return false;
}

void BugAlgorithm::processRawLidarData(LaserMeasurement laserData)
{
    this->lidarDataXY.clear();
    this->obstacles.clear();

    for (int i {0}; i<laserData.numberOfScans; i++) {
        double robotAngleRad = odData->rotation * PI / 180.0;
        double lidarAngleRad = (360.0 - laserData.Data[i].scanAngle) * PI / 180.0;
        double x = odData->posX + (laserData.Data[i].scanDistance / 1000.0) * cos(lidarAngleRad + robotAngleRad);
        double y = odData->posY + (laserData.Data[i].scanDistance / 1000.0) * sin(lidarAngleRad + robotAngleRad);
        if (laserData.Data[i].scanDistance != 0)
        {
            obstacles.push_back({x, y});
        }
        this->lidarDataXY.push_back({x, y});
    }
}

void BugAlgorithm::followObstacle()
{
    Controller::ErrorValue ev;
    if (this->robotState == MOVE_TO_GOAL)
    {
//        if (!isObstacleInPath())
//        {
            ev = controller->calculateErrors(this->goal, 0, 0);
            controller->regulate(ev);
//        }
//        else
//        {
//            this->robotState = FOLLOW_OBSTACLE;
            this->robot->setTranslationSpeed(0);
//        }
    }
//    else
//    {
//        if (this->robotState == FOLLOW_OBSTACLE) {
//            if (!isObstacleInPath())
//            {
//                this->robotState = MOVE_TO_GOAL;
//            }
//            else
//            {
//                reachingPos = findNextObstaclePoint({this->odData->posX, this->odData->posY}, this->goal, this->obstacles, 0.1);
//                ev = controller->calculateErrors({reachingPos.x-SAFE_DISTANCE, reachingPos.y-SAFE_DISTANCE}, 0, 0);
//                controller->regulate(ev);
//            }
//        }
//    }
}



void BugAlgorithm::detectCorners()
{
    std::ofstream myfile("D:\\FEI-STU\\RMR\\demoRMR-all-all-in\\example.csv", std::ios::trunc);

    std::vector<DBPoint> dbPoints;
    for (int i = 0; i < this->_laserData.numberOfScans; ++i)
    {
        if ((_laserData.Data[i].scanDistance < 3000.0 && _laserData.Data[i].scanDistance > 130) && (_laserData.Data[i].scanDistance < 640.0 || _laserData.Data[i].scanDistance > 700.0))  {
            dbPoints.push_back({lidarDataXY[i].x,lidarDataXY[i].y, -1});
        }
    }
    dbScan->fit(dbPoints);
    for (auto i = 0; i < dbPoints.size(); i++)
    {
        myfile << dbPoints[i].x << ',' << dbPoints[i].y << ',' << dbPoints[i].cluster << '\n';
    }

    myfile.close();
    std::cout<<"DBSCAN FINSIHED\n";
    std::vector<DBPoint> edge_points;
    int num_clusters = 0;
    for (auto& point : dbPoints) {
        if (point.cluster > num_clusters) {
            num_clusters = point.cluster;
        }
    }
    for (int cluster_id = 1; cluster_id <= num_clusters; ++cluster_id) {
        // Find points in current cluster
        std::vector<DBPoint> points;
        for (auto& point : dbPoints) {
            if (point.cluster == cluster_id) {
                points.push_back(point);
            }
        }

        // Find topmost, bottommost, leftmost, and rightmost points
        auto minmax_y = std::minmax_element(points.begin(), points.end(), [](const DBPoint& a, const DBPoint& b) {
            return a.y < b.y;
        });
        DBPoint top_point = *minmax_y.first;
        DBPoint bottom_point = *minmax_y.second;
        auto minmax_x = std::minmax_element(points.begin(), points.end(), [](const DBPoint& a, const DBPoint& b) {
            return a.x < b.x;
        });
        DBPoint left_point = *minmax_x.first;
        DBPoint right_point = *minmax_x.second;

        // Compute bounding box
        double x_min = left_point.x;
        double x_max = right_point.x;
        double y_min = min(top_point.y, bottom_point.y);
        double y_max = max(top_point.y, bottom_point.y);

//        BoundingBox bbox = {x_min, x_max, y_min, y_max};

        double theta = odData->rotation * PI / 180.0;
        double x_min_transformed = x_min * cos(theta) - y_min * sin(theta);
        double x_max_transformed = x_max * cos(theta) - y_max * sin(theta);
        double y_min_transformed = x_min * sin(theta) + y_min * cos(theta);
        double y_max_transformed = x_max * sin(theta) + y_max * cos(theta);

        // Print transformed bounding box for debugging
        std::cout << "Transformed bounding box for cluster " << cluster_id << ": x_min=" << x_min_transformed << ", x_max=" << x_max_transformed << ", y_min=" << y_min_transformed << ", y_max=" << y_max_transformed << std::endl;

        // Append transformed bounding box to output vector
        BoundingBox bbox = {x_min_transformed, x_max_transformed, y_min_transformed, y_max_transformed};
        // Print bounding box for debugging
        std::cout << "Bounding box for cluster " << cluster_id << ": x_min=" << bbox.x_min << ", x_max=" << bbox.x_max << ", y_min=" << bbox.y_min << ", y_max=" << bbox.y_max << std::endl;

        // Append topmost, bottommost, leftmost, and rightmost points to edge points
        edge_points.push_back(top_point);
        edge_points.push_back(bottom_point);
        edge_points.push_back(left_point);
        edge_points.push_back(right_point);
    }

    // Print edge points for debugging
    std::cout << "Edge points:" << std::endl;
    for (auto& point : edge_points) {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.cluster << ")" << std::endl;
    }
}

//void BugAlgorithm::detectCorners()
//{
//    ofstream myfile;
//    myfile.open ("D:\\FEI-STU\\RMR\\demoRMR-all-all-in\\example.csv");
//    this->cornerIndices.clear();
//    for (int i = 1; i < this->_laserData.numberOfScans; ++i)
//    {
//        if ((_laserData.Data[i].scanDistance < 3000.0 && _laserData.Data[i].scanDistance > 130) && (_laserData.Data[i].scanDistance < 640.0 || _laserData.Data[i].scanDistance > 700.0))  {
//            float distance1 = _laserData.Data[i-1].scanDistance/1000;
//            float distance2 = _laserData.Data[i].scanDistance/1000;

//            if (this->isBigJump(distance1, distance2, this->threshold)) {
//                this->cornerIndices.push_back(i);
//                myfile << lidarDataXY[i].x << "," << lidarDataXY[i].y << "\n";
//                std::cout<<"Corner: X: "<<lidarDataXY[i].x<<"Y: "<<lidarDataXY[i].y<<std::endl;
//            }
//        }
//    }
//    myfile.close();

//}


bool BugAlgorithm::isObstacleInPath()
{
    for (size_t i = 0; i < obstacles.size(); i++) {
        Point p1 = obstacles[i];
        Point p2 = obstacles[(i + 1) % obstacles.size()];
        if (doLineSegmentsIntersect(this->start, this->goal, p1, p2)) {
            this->obstacleIndex = i;
            this->robotState = FOLLOW_OBSTACLE;
            return true;
        }
    }
    return false;
}

bool BugAlgorithm::doLineSegmentsIntersect(Point start, Point goal, Point p2, Point q2) {
    // Heading line from start -> goal
    double a1 = goal.y - start.y;
    double b1 = start.x - goal.x;
    double c1 = a1 * start.x + b1 * start.y;

    double a2 = q2.y - p2.y;
    double b2 = p2.x - q2.x;
    double c2 = a2 * p2.x + b2 * p2.y;

    double det = a1 * b2 - a2 * b1;

    if (det == 0) {
        return false; // Line segments are parallel
    }
    else
    {
        double x = (b2 * c1 - b1 * c2) / det;
        double y = (a1 * c2 - a2 * c1) / det;
        bool isXWithinSegments = (x >= min(start.x, goal.x) && x <= max(start.x, goal.x) &&
                                     x >= min(p2.x, q2.x) && x <= max(p2.x, q2.x));
        bool isYWithinSegments = (y >= min(start.y, goal.y) && y <= max(start.y, goal.y) &&
                                     y >= min(p2.y, q2.y) && y <= max(p2.y, q2.y));
        return isXWithinSegments && isYWithinSegments;
    }
}

Point BugAlgorithm::findNextObstaclePoint(const Point& currentPos, const Point& goal, const std::vector<Point>& obstacles, double maxGoalDistanceIncrease) {
    Point nextPoint = reachingPos;
    double minTotalDistance = DBL_MAX;
    double maxTotalDistance = -DBL_MAX;
    double currentDistanceToGoal = this->distance(currentPos, goal);

    for (const Point& obstaclePoint : obstacles)
    {
        double distanceToObstaclePoint = distance(currentPos, obstaclePoint);
        double distanceToGoal = distance(obstaclePoint, this->goal);
        double totalDistance = distanceToObstaclePoint + distanceToGoal;
        if (totalDistance < minTotalDistance)
        {
            minTotalDistance = totalDistance;
            nextPoint = obstaclePoint;
        }


        // Check if the increase in the distance to the goal is within the allowed threshold
//        if (distanceToGoal <= currentDistanceToGoal) {
//            double totalDistance = distanceToObstaclePoint + distanceToGoal;
//            if (totalDistance < minTotalDistance) {
//                minTotalDistance = totalDistance;
//                nextPoint = obstaclePoint;
//            }
//        }
    }
    return nextPoint;
}

