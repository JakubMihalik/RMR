#pragma once

#include <iostream>
#include <fstream>
#include <queue>

#define EMPTY  0
#define WALL   1
#define START  -1
#define FINISH 2

#define ROBOT_RADIUS 0.35
#define ROBOT_DIAMETER (ROBOT_RADIUS * 2)

#define FILES_PATH "C:\\RMR_Files"

typedef struct
{
	double x;
	double y;
} Point;

typedef struct
{
	int x;
	int y;
	int weight;
} Cell;

typedef enum
{
	UP,
	DOWN,
	LEFT,
	RIGHT,
	INIT
} Direction;

typedef struct
{
	int x;
	int y;
	int weight;
	const Direction dir;
} Neighbour;

typedef struct
{
	Direction previous;
	Direction current;
} DirectionMemory;

class PathPlanning
{
	// Constructor and destructor
public:
	PathPlanning(std::ifstream& mapFile, double realWidth, double realHeight, double mapResolution, double mapOriginX, double mapOriginY);
	~PathPlanning();

	// Methods
public:
	std::queue<Point> createCheckpoints(double startX, double startY, double finishX, double finishY);

	// Member variables
public:
	double realWidth, realHeight;
	double startX, startY;
	double finishX, finishY;
	int** occupancyMap;
	int height;
	int width;
	double mapResolution;
	int xStartIndex, yStartIndex;
	int xFinishIndex, yFinishIndex;
    double mapOriginX, mapOriginY;

private:
	std::ifstream& mapFile;
	std::ofstream newMap;
	DirectionMemory* dirMem;

private:
	void floodFill();
	void writeData();
	void floodFillQueue(int initialX, int initialY, int initialWeight = 2);
    std::queue<Point> planPath(std::queue<Point>& waypoints);
	Neighbour findMinNeighbour(const Neighbour& n1, const Neighbour& n2, const Neighbour& n3, const Neighbour& n4);
	void checkWeights(Neighbour& neighbour);
	void scaleObstacles();
};

