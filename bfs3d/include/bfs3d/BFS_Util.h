#ifndef _BFS_UTILS_
#define _BFS_UTILS_

#include <bfs3d/BFS_3D.h>
#include <vector>
//#include <distance_field/propagation_distance_field.h>

using namespace std;

//void setDistanceField(BFS_3D* bfs, const distance_field::PropagationDistanceField* df, int radius);

int getMin(int* numbers, int n);

void getMinNeighbor(sbpl_arm_planner::BFS_3D* bfs, int &nodeX, int &nodeY, int &nodeZ);

vector<vector<int> > getShortestPath(sbpl_arm_planner::BFS_3D* bfs, int currentX, int currentY, int currentZ, int endX, int endY, int endZ);

#endif
