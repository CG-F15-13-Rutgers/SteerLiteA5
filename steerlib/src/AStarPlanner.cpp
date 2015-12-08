#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner() {}

	AStarPlanner::~AStarPlanner() {}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout << "\nIn A*";
		start = getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(start));
		goal = getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(goal));
		//create two nodes. one for start point and another for target point
		SteerLib::AStarPlannerNode startNode(start);
		SteerLib::AStarPlannerNode targetNode(goal);


		//create openSet and closedSet
		std::vector<SteerLib::AStarPlannerNode> openSet;
		std::vector<Util::Point> closedSet;

		//add startNode to openSet
		openSet.push_back(startNode);



		while (!openSet.empty()) {

			//PrintOpenSet(startNode.point,openSet); TESTING

			SteerLib::AStarPlannerNode* currentNode = new SteerLib::AStarPlannerNode();// will contain node with the lowest f cost
			*currentNode = openSet[0];

			int index = 0;
			for (int i = 1; i < openSet.size(); i++) {
				if (openSet[i].f < currentNode->f) {
					*currentNode = openSet[i];
					index = i;
				}
				else if (openSet[i].f == currentNode->f) {
					if (openSet[i].g > currentNode->g) {//set to use bigger g for breaking ties
						*currentNode = openSet[i];
						index = i;
					}

				}
			}

			openSet.erase(openSet.begin() + index);
			closedSet.push_back(currentNode->point);


			//PrintNode(*currentNode);//USED FOR DEBUGGING

			if (currentNode->point.operator==(targetNode.point)) {//check if targetNode was found
				//std::cout << "found targetNode" << std::endl;
				RetracePath(startNode, *currentNode, agent_path, gSpatialDatabase);
				std::cout << "Length of Solution Path = " << agent_path.size()<<std::endl;
	std::cout << "Number of Expanded Nodes = " << closedSet.size()<<std::endl;			
				return true;//path has been found
			}
			//generate currentNode's neighbors
			std::vector<SteerLib::AStarPlannerNode> neighbours = GetNeighbours(*currentNode, gSpatialDatabase);

			//printNeighbours(neighbours); USED FOR DEBUGGING

			for (int i = 0; i < neighbours.size(); i++) {
				//check if walkable by using the given traversable function above
				bool walkable = canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(neighbours[i].point));//basically converting the point to grid point and checking if that 'cell' is walkable

				int count = std::count(closedSet.begin(), closedSet.end(), neighbours[i].point);//traverses closedSet and returns 1 if neighbor is in it

				if ((!walkable) || (count != 0)) { // true if location is not walkable or closedSet contains this neighbor
												   //if (count!=0) {
												   //std::cout << "point skipped b/c it's in closed: " << neighbours[i].point.x << "," << neighbours[i].point.z << std::endl;
												   //} USED FOR DEBUGGING
					continue;
				}

				int diagonal = ((int)(neighbours[i].point.x + neighbours[i].point.z) % 2) == ((int)(currentNode->point.x + currentNode->point.z) % 2);
				double distFromCurrentToIJ = 0;
				//costs of moving diagonally vs sideways 
				if (diagonal) {

					distFromCurrentToIJ = 1;
					//distFromCurrentToIJ = 1;
				}
				else {
					distFromCurrentToIJ = 1;
				}

				double tempG = currentNode->g + distFromCurrentToIJ;
				double tempH = Manhattan(neighbours[i].point, targetNode.point);//INSERT HEURISTIC HERE, currently using Manhattan
				//double tempH = Euclidean(neighbours[i].point, targetNode.point); //uncomment to use Euclidian distance
				double tempF = tempG + tempH;
	
				double existingG;
				int x;
				bool processed = false;
				for (x = 0; x < openSet.size(); x++) { 
					if (openSet[x].point.operator==(neighbours[i].point)) {//found neighbor in openSet
						existingG = openSet[x].g;
						processed = true;// 
						break;
					}
				}

				if (processed) {

					if (tempG < existingG) {//replace f,g,h, parent
						openSet[x].f = tempF;
						openSet[x].g = tempG;
						openSet[x].h = tempH;

						openSet[x].parent = currentNode;

					}
				}
				else {
					//add to openlist
					//std::cout << "c = " << processed << std::endl; //USED FOR DEBUGGING

					neighbours[i].g = tempG;
					//std::cout << "g =" << neighbours[i].g << std::endl;//USED FOR DEBUGGING
					neighbours[i].h = tempH;
					//std::cout << "h =" << neighbours[i].h << std::endl;//USED FOR DEBUGGING
					neighbours[i].f = tempF;
					//std::cout << "f = " << neighbours[i].f << std::endl;//USED FOR DEBUGGING

					neighbours[i].parent = currentNode;
					openSet.push_back(neighbours[i]);
				}
			}

		}


		return false;
	}

	double AStarPlanner::Euclidean(Util::Point neighbor, Util::Point targetPoint)
	{
	 	return abs(sqrt(pow((neighbor.x - targetPoint.x), 2) + pow((neighbor.z - targetPoint.z), 2))) ;
	}

	double AStarPlanner::Manhattan(Util::Point neighbor, Util::Point targetPoint)
	{
		return  (abs(neighbor.x - targetPoint.x) + abs(neighbor.z - targetPoint.z));

	}

	void AStarPlanner::RetracePath(SteerLib::AStarPlannerNode startNode, SteerLib::AStarPlannerNode endNode, std::vector<Util::Point>& agentPath, SteerLib::GridDatabase2D* grid)
	{//need to get grid points
		SteerLib::AStarPlannerNode currentNode = endNode;
		
		while (!(currentNode.operator==(startNode))) {
			//std::cout << "retrace: " << currentNode.point.x << "," << currentNode.point.z << std::endl;//USED FOR DEBUGGING
			//std::cout << "grid location: " << temp.x << "," << temp.z << std::endl;//USED FOR DEBUGGING
			agentPath.push_back(currentNode.point);
			currentNode = *currentNode.parent;

		}

		std::reverse(agentPath.begin(), agentPath.end());
	}

	void AStarPlanner::PrintOpenSet(Util::Point start, std::vector<SteerLib::AStarPlannerNode> openSet)
	{
		std::cout << "PrintingOpenSet:" << std::endl;
		for (int i = 0; i < openSet.size(); i++) {
			std::cout << "(" << openSet[i].point.x << "," << openSet[i].point.z << ") " << "->f:" << openSet[i].f;
			if (openSet[i].point.operator!=(start)) {
				std::cout << "PARENT: " << "(" << openSet[i].parent->point.x << "," << openSet[i].parent->point.z << ")" << std::endl;
			}
		}
		std::cout << std::endl;
	}

	void AStarPlanner::PrintNode(SteerLib::AStarPlannerNode node)
	{
		std::cout << "printing currentNode: " << node.point.x << "," << node.point.z << std::endl;
	}

	int AStarPlanner::GetDistance(SteerLib::AStarPlannerNode nodeA, SteerLib::AStarPlannerNode nodeB)//manhattan
	{
		int distanceX = abs(nodeA.point.x - nodeB.point.x);// changed from int to float
		int distanceZ = abs(nodeA.point.z - nodeB.point.z);// int to float
		if (distanceX > distanceZ) {
			return 14 * distanceZ + 10 * (distanceX - distanceZ);
		}
		return 14 * distanceX + 10 * (distanceZ - distanceX);
	}


	std::vector<SteerLib::AStarPlannerNode> AStarPlanner::GetNeighbours(SteerLib::AStarPlannerNode currentNode, SteerLib::GridDatabase2D * grid)
	{
		//given a currentNode, let's use its Util::point point
		Util::Point currentPoint = currentNode.point;
		std::vector<SteerLib::AStarPlannerNode> neighbours;

		for (int x = -1; x <= 1; x++) {//check if inside grid
			for (int z = -1; z <= 1; z++) {
				if ((x == 0) && (z == 0))
					continue;
				double checkX = currentPoint.x + x;
				double checkZ = currentPoint.z + z;
				//check if checkX and checkY is inside of grid

				//if ((checkX >= 0) && (checkX < (int)(grid->getGridSizeX)) && (checkZ >= 0) && (checkZ < grid->getGridSizeZ)) {
				Util::Point n;
				n.x = checkX;
				n.z = checkZ;
				SteerLib::AStarPlannerNode starNode(n);
				neighbours.push_back(starNode);
				//}
			}
		}
		return neighbours;
	}
	void AStarPlanner::printNeighbours(std::vector<SteerLib::AStarPlannerNode> neighbours)//used for testing
	{
		std::cout << "neighbors.size(): " << neighbours.size() << std::endl;
		for (int i = 0; i < neighbours.size(); i++) {
			std::cout << "neighbor " << i << ": " << "( " << neighbours[i].point.x << "," << neighbours[i].point.z << " )" << std::endl;
		}
	}
}
