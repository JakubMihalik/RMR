#include "DBScan.h"

DBScan::DBScan(double eps, int minSamples)
{
    this->eps = eps;
    this->minSamples = minSamples;
}

void DBScan::fit(std::vector<DBPoint>& points)
{
    int clusterId = 0;
    for (auto& point : points) {
        if (point.cluster == -1) {
            std::vector<int> neighbors = findNeighbours(points, &point);
            if (neighbors.size() < minSamples) {
                point.cluster = 0; // Label as noise
            } else {
                clusterId++;
                expandCluster(points, &point, neighbors, clusterId);
            }
        }
    }
}

void DBScan::expandCluster(std::vector<DBPoint>& points, DBPoint* point, const std::vector<int>& neighbors, int clusterId) {
    point->cluster = clusterId;
     std::queue<int> q;
     for (int idx : neighbors) {
         q.push(idx);
     }

     while (!q.empty()) {
         int idx = q.front();
         q.pop();
         DBPoint* currentPoint = &points[idx];

         if (currentPoint->cluster <= 0) {
             currentPoint->cluster = clusterId;

             std::vector<int> currentNeighbours = findNeighbours(points, currentPoint);
             if (currentNeighbours.size() >= minSamples) {
                 for (int neighbourIndex : currentNeighbours) {
                     q.push(neighbourIndex);
                 }
             }
         }
     }
}

std::vector<int> DBScan::findNeighbours(const std::vector<DBPoint>& points, const DBPoint* point) {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (distance(points[i], *point) <= eps) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}
