#include "DBScan.h"

DBScan::DBScan(double eps, int minSamples)
{
    this->eps = eps;
    this->minSamples = minSamples;
}

void DBScan::fit(std::vector<DBPoint>& points)
{
    int cluster_id = 0;
    for (auto& point : points) {
        if (point.cluster == -1) {
            std::vector<int> neighbors = regionQuery(points, &point);
            if (neighbors.size() < minSamples) {
                point.cluster = 0; // Label as noise
            } else {
                ++cluster_id;
                expandCluster(points, &point, neighbors, cluster_id);
            }
        }
    }
}

void DBScan::expandCluster(std::vector<DBPoint>& points, DBPoint* point, const std::vector<int>& neighbors, int cluster_id) {

}

std::vector<int> DBScan::regionQuery(const std::vector<DBPoint>& points, const DBPoint* point) {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if (distance(points[i], *point) <= eps) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}
