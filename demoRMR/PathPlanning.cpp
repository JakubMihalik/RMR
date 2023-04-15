#define _CRT_SECURE_NO_WARNINGS

#include "PathPlanning.h"
#include <string>

FILE* data_out;

PathPlanning::PathPlanning(std::ifstream& mapFile, double realWidth, double realHeight, double mapResolution, double mapOriginX, double mapOriginY) : mapFile(mapFile)
{
	this->realWidth = realWidth;
	this->realHeight = realHeight;
	this->mapResolution = mapResolution;

	this->height = 0;
	this->width = 0;
	this->occupancyMap = NULL;

	this->finishX = 0.0;
	this->finishY = 0.0;
	this->startX = 0.0;
	this->startY = 0.0;

	this->xStartIndex = -1;
	this->yStartIndex = -1;
	this->xFinishIndex = -1;
	this->yFinishIndex = -1;

	this->mapOriginX = mapOriginX;
	this->mapOriginY = mapOriginY;

	this->dirMem = new DirectionMemory();
	this->dirMem->current = INIT;
	this->dirMem->previous = INIT;

    this->newMap.open("C:\\RMR_Files\\map2D_shrink.txt");

    data_out = fopen("C:\\RMR_Files\\map_data.txt", "w+");
	if (data_out == NULL)
	{
		perror("Cannot open file");
		exit(-5);
	}
}

PathPlanning::~PathPlanning()
{
	for (int i = 0; i < this->height; ++i)
	{
		free(this->occupancyMap[i]);
	}
	free(this->occupancyMap);

	fclose(data_out);
	std::cout << "File closed, object destroyed\n";

	delete this->dirMem;
}

std::queue<Point> PathPlanning::createCheckpoints(double startX, double startY, double finishX, double finishY)
{
	this->finishX = finishX;
	this->finishY = finishY;
	this->startX = startX;
	this->startY = startY;

	std::queue<Point> planPoints;

	// Map reader
	if (this->mapFile.is_open())
	{
		int colMin = INT_MAX, colMax = -1, rowMin = INT_MAX, rowMax = -1;
		int currC = 0, currR = 0;
		int row = 0, col = 0;
		int totalCols = 0;
		char c;
		while (mapFile.get(c))
		{
			if (c == '\n')
			{
				row++;
				totalCols = col;
				col = 0;
				continue;
			}
			if (c == 'X')
			{
				currC = col;
				currR = row;

				if (currC < colMin)
					colMin = currC;
				if (currC > colMax)
					colMax = currC;

				if (currR < rowMin)
					rowMin = currR;
				if (currR > rowMax)
					rowMax = currR;
			}
			col++;
		}
		mapFile.clear();
		mapFile.seekg(0, std::ios::beg);

		this->height = rowMax - rowMin + 1;
		this->width = colMax - colMin + 1;
		std::cout << "Cols:" << totalCols << " Rows:" << row << std::endl;
		std::cout << "New dimensions [HxW]: " << this->height << "x" << this->width << std::endl;

		// Convert text file to array
		int** file2map = (int**)malloc(row * sizeof(int*));
		for (int i = 0; i < row; i++)
		{
			file2map[i] = (int*)malloc(totalCols * sizeof(int));
		}
		for (int iRow = 0; iRow < row; iRow++)
		{
			for (int iCol = 0; iCol < totalCols; iCol++)
			{
				if (mapFile.good())
				{
					mapFile.get(c);
					if (c == '\n')
					{
						iCol--;
						continue;
					}
					if (c == 'X')
						file2map[iRow][iCol] = 1;
					else
						file2map[iRow][iCol] = 0;
				}
				else
				{
					std::cerr << "File not good\n";
					exit(-5);
				}
			}
		}

		// Allocate new array
		this->occupancyMap = (int**)malloc(this->height * sizeof(int*));
		for (int i = 0; i < this->height; i++)
		{
			this->occupancyMap[i] = (int*)malloc(this->width * sizeof(int));
		}

		// Copy values to new array
		for (int r = 0; r < this->height; r++)
		{
			for (int c = 0; c < this->width; c++)
			{
				this->occupancyMap[r][c] = file2map[rowMin + r][colMin + c];
			}
		}

		// Free memory
		for (int i = 0; i < row; i++)
		{
			free(file2map[i]);
		}
		free(file2map);
	}

	scaleObstacles();
	floodFill();
    planPath(planPoints);
	writeData();

	for (int y{ 0 }; y < this->height; y++)
	{
		for (int x{ 0 }; x < this->width; x++)
		{
			fprintf(data_out, "%6d ", this->occupancyMap[y][x]);
		}
		fprintf(data_out, "\n");
	}
	fprintf(data_out, "\n");

	return planPoints;
}

void PathPlanning::scaleObstacles()
{
	// Horizontal scale Left->Right
	for (int y{ 0 }; y < this->height; y++)
	{
		for (int x{ 0 }; x < this->width; x++)
		{
			// Zistim ci som na konci steny
			if ((x + 1 < this->width) && this->occupancyMap[y][x] == WALL && this->occupancyMap[y][x + 1] == EMPTY)
			{
				// Zvacsujem stenu o polomer robota
				int i = 0;
				while (i < std::round(ROBOT_RADIUS / this->mapResolution))
				{
					// Kontrola ci mozem zvacsit stenu
					if ((x + i + 1) < this->width && this->occupancyMap[y][x + i + 1] == EMPTY)
					{
						this->occupancyMap[y][x + i + 1] = WALL;
						i++;
					}
					else
					{
						break;
					}
				}
				x += i;
			}
		}
	}

	// Horizontal scale Right->Left
	for (int y{ 0 }; y < this->height; y++)
	{
		for (int x{ this->width - 1 }; x >= 0; x--)
		{
			// Zistim ci som na konci steny
			if ((x - 1 >= 0) && this->occupancyMap[y][x] == WALL && this->occupancyMap[y][x - 1] == EMPTY)
			{
				// Zvacsujem stenu o polomer robota
				int i = 0;
				while (i < std::round(ROBOT_RADIUS / this->mapResolution))
				{
					// Kontrola ci mozem zvacsit stenu
					if ((x - i - 1) > 0 && this->occupancyMap[y][x - i - 1] == EMPTY)
					{
						this->occupancyMap[y][x - i - 1] = WALL;
						i++;
					}
					else
					{
						break;
					}
				}
				x -= i;
			}
		}
	}

	// Vertical scale Up->Down
	for (int x{ 0 }; x < this->width; x++)
	{
		for (int y{ 0 }; y < this->height; y++)
		{
			// Zistim ci som na konci steny
			if ((y + 1 < this->height) && this->occupancyMap[y][x] == WALL && this->occupancyMap[y + 1][x] == EMPTY)
			{
				// Zvacsujem stenu o polomer robota
				int i = 0;
				while (i < std::round(ROBOT_RADIUS / this->mapResolution))
				{
					// Kontrola ci mozem zvacsit stenu
					if ((y + i + 1) < this->height && this->occupancyMap[y + i + 1][x] == EMPTY)
					{
						this->occupancyMap[y + i + 1][x] = WALL;
						i++;
					}
					else
					{
						break;
					}
				}
				y += i;
			}
		}
	}

	// Vertical scale Down->Up
	for (int x{ 0 }; x < this->width; x++)
	{
		for (int y{ this->height - 1 }; y >= 0; y--)
		{
			// Zistim ci som na konci steny
			if ((y - 1 >= 0) && this->occupancyMap[y][x] == WALL && this->occupancyMap[y - 1][x] == EMPTY)
			{
				// Zvacsujem stenu o polomer robota
				int i = 0;
				while (i < std::round(ROBOT_RADIUS / this->mapResolution))
				{
					// Kontrola ci mozem zvacsit stenu
					if ((y - i - 1) >= 0 && this->occupancyMap[y - i - 1][x] == EMPTY)
					{
						this->occupancyMap[y - i - 1][x] = WALL;
						i++;
					}
					else
					{
						break;
					}
				}
				y -= i;
			}
		}
	}
}

void PathPlanning::floodFill()
{
#pragma region Initial calculations
	this->xStartIndex = std::round((this->startX - this->mapOriginX) / this->mapResolution);
	this->yStartIndex = this->height - std::round((this->startY - this->mapOriginY) / this->mapResolution);

	this->xFinishIndex = std::round((this->finishX - this->mapOriginX) / this->mapResolution);
	this->yFinishIndex = this->height - std::round((this->finishY - this->mapOriginY) / this->mapResolution);

    std::cout << "Finish position: " << this->finishX << ", " << this->finishY << std::endl;

	this->occupancyMap[this->yStartIndex][this->xStartIndex] = START;
#pragma endregion

	floodFillQueue(this->xFinishIndex, this->yFinishIndex);
}

void PathPlanning::writeData()
{
	if (this->newMap.is_open())
	{
		for (int iRow = 0; iRow < this->height; iRow++)
		{
			for (int iCol = 0; iCol < this->width; iCol++)
			{
				switch (this->occupancyMap[iRow][iCol])
				{
				case EMPTY:
					this->newMap << " ";
					break;
				case WALL:
					this->newMap << ".";
					break;
				case START:
					this->newMap << "S";
					break;
				case FINISH:
					this->newMap << "F";
					break;
				default:
					this->newMap << " ";
					break;
				}
			}
			this->newMap << "\n";
		}
		this->newMap.close();
	}
	else
	{
        std::cerr << "Cannot write to file" << std::endl;
	}
}

void PathPlanning::floodFillQueue(int x, int y, int weight)
{
	std::queue<Cell> acumulator;

	// Return checks
	if (x >= this->width || y >= this->height || x < 0 || y < 0) return;

	this->occupancyMap[y][x] = weight;
	acumulator.push({ x, y, weight });

	while (!acumulator.empty())
	{
		Cell front = acumulator.front();
		if (this->occupancyMap[front.y][front.x] == START) break;

		weight = front.weight + 1;

		if (front.y + 1 < this->height)
		{
			// Pridanie nizsieho suseda
			if (this->occupancyMap[front.y + 1][front.x] == EMPTY)
			{
				this->occupancyMap[front.y + 1][front.x] = weight;
				acumulator.push({ front.x, front.y + 1, weight });
			}
		}
		if (front.y - 1 >= 0)
		{
			// Pridanie horneho suseda
			if (this->occupancyMap[front.y - 1][front.x] == EMPTY)
			{
				this->occupancyMap[front.y - 1][front.x] = weight;
				acumulator.push({ front.x, front.y - 1, weight });
			}
		}
		if (front.x + 1 < this->width)
		{
			// Pridanie praveho suseda
			if (this->occupancyMap[front.y][front.x + 1] == EMPTY)
			{
				this->occupancyMap[front.y][front.x + 1] = weight;
				acumulator.push({ front.x + 1, front.y, weight });
			}
		}
		if (front.x - 1 >= 0)
		{
			// Pridanie laveho suseda
			if (this->occupancyMap[front.y][front.x - 1] == EMPTY)
			{
				this->occupancyMap[front.y][front.x - 1] = weight;
				acumulator.push({ front.x - 1, front.y, weight });
			}
		}
		acumulator.pop();
	}
}

std::queue<Point> PathPlanning::planPath(std::queue<Point>& waypoints)
{

	// Find smallest number in neighbours
	Neighbour upper = { -1, -1, INT_MAX, UP }, lower = { -1, -1, INT_MAX, DOWN }, left = { -1, -1, INT_MAX, LEFT }, right = { -1, -1, INT_MAX, RIGHT };
	int x = this->xStartIndex;
	int y = this->yStartIndex;

	bool b_foundFinish = false;

	// Debug
    std::cout << "Map dimensions [m] X, Y: " << this->width * this->mapResolution << ", " << this->height * this->mapResolution << std::endl;

	// Debug vypis checkpoitov do suboru
	std::ofstream checkpoints_data;
    checkpoints_data.open("C:\\RMR_Files\\checkpoints.csv");

	while (!b_foundFinish)
	{

		// Check upper
		if (y - 1 >= 0)
		{
			upper.weight = this->occupancyMap[y - 1][x];
			upper.x = x;
			upper.y = y - 1;

			// Handle walls and start values on occupancy map
			checkWeights(upper);
		}
		// Check lower
		if (y + 1 < this->height)
		{
			lower.weight = this->occupancyMap[y + 1][x];
			lower.x = x;
			lower.y = y + 1;
			// Handle walls and start values on occupancy map
			checkWeights(lower);
		}
		// Check right
		if (x + 1 < this->width)
		{
			right.weight = this->occupancyMap[y][x + 1];
			right.x = x + 1;
			right.y = y;
			// Handle walls and start values on occupancy map
			checkWeights(right);
		}
		// Check left
		if (x - 1 >= 0)
		{
			left.weight = this->occupancyMap[y][x - 1];
			left.x = x - 1;
			left.y = y;
			// Handle walls and start values on occupancy map
			checkWeights(left);
		}

		// Find minimum
		Neighbour min = PathPlanning::findMinNeighbour(upper, lower, left, right);

		// Save direction of movement
		if (dirMem->current == INIT && dirMem->previous == INIT)
		{
			dirMem->current = min.dir;
		}
		else if (dirMem->current != INIT && dirMem->previous == INIT)
		{
			dirMem->previous = dirMem->current;
			dirMem->current = min.dir;
		}
		else
		{
			dirMem->previous = dirMem->current;
			dirMem->current = min.dir;
		}

        double realX = x * this->mapResolution + this->mapOriginX;
        double realY = (this->height - y) * this->mapResolution + this->mapOriginY;

		if ((dirMem->current != INIT && dirMem->previous != INIT) && (dirMem->current != dirMem->previous))
        {
            // Save waypoint in real dimensions
            waypoints.push({ realX, realY });
            checkpoints_data << realX << "," << realY << std::endl;
		}

		// Check if finish was found
		if (this->occupancyMap[y][x] == FINISH)
		{
            b_foundFinish = true;

            Point back = waypoints.back();

            if ( (back.x != realX) && (back.y != realY))
            {
                // Save waypoint in real dimensions
                waypoints.push({ realX, realY });
                checkpoints_data << realX << "," << realY << std::endl;
            }
		}

		// Update location based on minimum element
		switch (min.dir)
		{
		case UP:
			y--;
			break;
		case DOWN:
			y++;
			break;
		case LEFT:
			x--;
			break;
		case RIGHT:
			x++;
			break;
		default:
			break;
		}
	}
	checkpoints_data.close();
	return waypoints;
}

Neighbour PathPlanning::findMinNeighbour(const Neighbour& n1, const Neighbour& n2, const Neighbour& n3, const Neighbour& n4)
{
	return std::min({ n1, n2, n3, n4 }, [](const Neighbour& n1, const Neighbour& n2)
		{
			return n1.weight < n2.weight;
		});
}

void PathPlanning::checkWeights(Neighbour& neighbour)
{
	if (this->occupancyMap[neighbour.y][neighbour.x] == WALL || this->occupancyMap[neighbour.y][neighbour.x] == START)
	{
		neighbour.weight = INT_MAX;
	}
}
