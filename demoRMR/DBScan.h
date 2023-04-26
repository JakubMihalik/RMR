// dbscan.h
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>

typedef struct {
    double x, y;
    int cluster;
} DBPoint;

class DBScan {
public:
    DBScan(double eps, int minSamples);

    void fit(std::vector<DBPoint>& points);

private:
    double eps;
    int minSamples;

    void expandCluster(std::vector<DBPoint>& points, DBPoint* point, const std::vector<int>& neighbors, int clusterId);
    std::vector<int> regionQuery(const std::vector<DBPoint>& points, const DBPoint* point);

    double distance(DBPoint p1, DBPoint p2) {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }
};

#endif // DBSCAN_H
